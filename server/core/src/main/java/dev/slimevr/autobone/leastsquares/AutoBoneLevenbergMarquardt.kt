package dev.slimevr.autobone.leastsquares

import dev.slimevr.config.AutoBoneConfig
import io.eiren.util.logging.LogManager
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction
import org.apache.commons.math3.linear.Array2DRowRealMatrix
import org.apache.commons.math3.linear.ArrayRealVector
import org.apache.commons.math3.linear.EigenDecomposition
import org.apache.commons.math3.linear.RealMatrix
import org.apache.commons.math3.linear.RealVector
import org.apache.commons.math3.optim.ConvergenceChecker
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt
import org.apache.commons.math3.util.Pair as MathPair

/**
 * Solves [AutoBoneObjective] with Levenberg-Marquardt and reports parameter
 * covariance.
 *
 * The optimiser itself is commons-math3's `LevenbergMarquardtOptimizer`, which
 * is already a dependency of this module and is a port of MINPACK's `lmder`.
 * Writing one here would have meant writing and testing a QR solve and a
 * damping schedule to arrive at the same place.
 *
 * ## What replaces the tuning knobs
 *
 * `initialAdjustRate` and `adjustRateDecay` have no analogue: LM's damping
 * adapts from the agreement between the predicted and actual cost reduction.
 * `numEpochs` is replaced by tolerances, so the solve stops when it has stopped
 * making progress rather than when it runs out of a fixed budget. The
 * termination reason is reported rather than assumed.
 *
 * ## Jacobians
 *
 * Central finite differences in log space. The step size matters more than it
 * usually would: the skeleton solve underneath is `Float`, so residuals carry
 * roughly 1e-7 relative noise, and differencing amplifies it by `1/h`. Too
 * small a step buys nothing but noise. For central differences the error is
 * minimised near `h ≈ (3ε)^(1/3) ≈ 7e-3`, which is where the default sits.
 *
 * Central rather than forward differences costs `2n` evaluations per Jacobian
 * instead of `n + 1`. That is worth it here because the same Jacobian is reused
 * at the optimum to produce the covariance, and a covariance built from a noisy
 * one-sided derivative is a confident-looking wrong answer -- the exact failure
 * this work exists to remove.
 *
 * ## Rank deficiency, and why it is not an edge case here
 *
 * `AutoBone.adjustOffsets` is not an identifiable parameter set, and no
 * recording makes it one. `UPPER_CHEST` and `CHEST` are consecutive bones driven
 * by the same chest tracker, so they always move together and only their sum is
 * ever observable. `HEAD` and `HIPS_WIDTH` are the only two adjusted bones
 * outside the height constraint, and neither shows up in foot slide unless the
 * recording contains motion that separates them.
 *
 * Handed that problem raw, LM fails in a way that looks like success. The first
 * step is dominated by the null directions -- they have near-zero curvature, so
 * the normal equations ask for an enormous move along them -- and moving along
 * them changes the cost by nothing. A step with no cost reduction is exactly
 * what `costRelativeTolerance` is watching for, so the optimiser reports
 * convergence having moved the unconstrained parameters somewhere arbitrary and
 * the constrained ones not at all. Measured on a recording whose answer is known
 * by construction, that is the whole of the gap between this path and the greedy
 * search: see `AutoBoneHeadToHeadTests`.
 *
 * The fix is Tikhonov regularisation on the parameters, sized relative to the
 * data rather than in absolute units. A prior residual `sqrt(w)·(θ - θ₀)` per
 * parameter puts a floor of `w` under every eigenvalue of `JᵀJ`, so no direction
 * is free, and with `w = lmRegularisation · λ_max` the floor is a stated
 * fraction of the best-constrained direction rather than a number that has to be
 * retuned per recording.
 *
 * What it does to a parameter the recording cannot see is the behaviour that was
 * wanted anyway: it stays at its starting value, which is the population
 * default, rather than absorbing whatever the null-space step happened to hand
 * it.
 *
 * Two properties are deliberately preserved:
 *
 * - The prior rows have an **analytic** Jacobian, `sqrt(w)·I`, so they add
 *   nothing to the per-iteration cost. Sizing the weight needs one Jacobian at
 *   the start point, `2n` evaluations, charged once; that shows up in the
 *   reported evaluation count rather than being hidden.
 * - The covariance is computed from the **data rows only**. The prior makes the
 *   solve well posed; it must not make the recording look more informative than
 *   it is. A parameter the recording cannot determine still has to come back
 *   unbounded, which is the finding `#22` exists to keep.
 */
object AutoBoneLevenbergMarquardt {

	/**
	 * Eigenvalues of `JᵀJ` below this fraction of the largest are treated as
	 * structurally zero rather than inverted into an enormous variance.
	 */
	private const val SINGULARITY_THRESHOLD = 1e-12

	/**
	 * Squared eigenvector component above which a parameter counts as living in
	 * an unconstrained direction.
	 *
	 * 1% of a unit eigenvector. Low enough that a parameter genuinely caught in
	 * a null direction is always flagged, high enough that numerical dust in an
	 * otherwise well-determined parameter is not.
	 */
	private const val UNBOUNDED_PROJECTION = 0.01

	fun solve(
		objective: AutoBoneObjective,
		initialParameters: DoubleArray,
		config: AutoBoneConfig,
		/**
		 * Called once per LM iteration with the current parameters.
		 *
		 * LM has no epochs, but the GUI progress bar is driven by the epoch
		 * callback, and a solve that reports nothing until it finishes is a
		 * visible regression however fast it is.
		 */
		onIteration: ((Int, DoubleArray) -> Unit)? = null,
	): AutoBoneSolution {
		val n = objective.parameterCount
		val m = objective.residualCount
		require(initialParameters.size == n) { "expected $n parameters, got ${initialParameters.size}" }

		val jacobianStep = config.lmJacobianStep.toDouble()

		// Sized from the data at the start point, once, so the objective stays a
		// fixed function of theta for the whole solve -- finite differences
		// require that, and a weight that moved with the iterate would also make
		// "the cost went down" mean two different things on consecutive steps.
		val priorWeight = regularisationWeight(
			jacobian(objective, initialParameters, jacobianStep),
			config.lmRegularisation.toDouble(),
		)
		val priorScale = sqrt(priorWeight)
		val regularised = priorWeight > 0.0
		// Prior rows are appended, so the data rows keep their indices and
		// everything downstream can recover them by taking the first m.
		val mTotal = if (regularised) m + n else m

		if (regularised) {
			LogManager.info(
				"[AutoBone/LM] regularising: prior weight %.3e on each of $n parameters, holding directions the recording constrains less than %.0e of its best toward the starting lengths"
					.format(priorWeight, config.lmRegularisation),
			)
		}

		val model = MultivariateJacobianFunction { point: RealVector ->
			val parameters = point.toArray()
			val data = objective.residuals(parameters)
			val dataJacobian = jacobian(objective, parameters, jacobianStep)

			if (!regularised) {
				MathPair.create<RealVector, RealMatrix>(
					ArrayRealVector(data, false),
					Array2DRowRealMatrix(dataJacobian, false),
				)
			} else {
				val value = DoubleArray(mTotal)
				System.arraycopy(data, 0, value, 0, m)
				val rows = Array(mTotal) { DoubleArray(n) }
				for (i in 0 until m) rows[i] = dataJacobian[i]
				for (j in 0 until n) {
					value[m + j] = priorScale * (parameters[j] - initialParameters[j])
					// Analytic, so the prior costs no objective evaluations.
					rows[m + j][j] = priorScale
				}
				MathPair.create<RealVector, RealMatrix>(
					ArrayRealVector(value, false),
					Array2DRowRealMatrix(rows, false),
				)
			}
		}

		val builder = LeastSquaresBuilder()
			.start(initialParameters)
			.model(model)
			// The residuals are already the quantity to drive to zero, so the
			// target is the zero vector rather than a set of observations.
			.target(DoubleArray(mTotal))
			.maxEvaluations(config.lmMaxIterations * 4)
			.maxIterations(config.lmMaxIterations)
			.lazyEvaluation(false)

		if (onIteration != null) {
			// LM treats a checker as an *additional* stopping criterion, so one
			// that never reports convergence leaves its own tolerances in
			// charge and just gives a per-iteration hook.
			builder.checker(
				object : ConvergenceChecker<LeastSquaresProblem.Evaluation> {
					override fun converged(
						iteration: Int,
						previous: LeastSquaresProblem.Evaluation,
						current: LeastSquaresProblem.Evaluation,
					): Boolean {
						onIteration(iteration, current.point.toArray())
						return false
					}
				},
			)
		}

		val problem: LeastSquaresProblem = builder.build()

		val optimizer = LevenbergMarquardtOptimizer()
			.withCostRelativeTolerance(config.lmCostTolerance.toDouble())
			.withParameterRelativeTolerance(config.lmParameterTolerance.toDouble())

		var converged = true
		var message = "cost/parameter tolerance"
		val optimum = try {
			optimizer.optimize(problem)
		} catch (e: Exception) {
			// A hit iteration cap is a legitimate outcome to report, not a
			// reason to lose the work: fall back to reporting the start point
			// rather than throwing away the run.
			LogManager.warning("[AutoBone/LM] optimisation stopped early: ${e.message}")
			converged = false
			message = e.message ?: e.javaClass.simpleName
			null
		}

		val parameters = optimum?.point?.toArray() ?: initialParameters
		// Data rows only from here down. The prior is a device for making the
		// solve well posed; letting it into the fit statistics would report the
		// recording as more informative than it is, and letting it into the
		// covariance would report a parameter the recording cannot see as
		// determined to the width of the prior.
		val residuals = (optimum?.residuals?.toArray() ?: objective.residuals(parameters))
			.copyOf(m)
		val jacobian = optimum?.jacobian?.getSubMatrix(0, m - 1, 0, n - 1)
			?: Array2DRowRealMatrix(jacobian(objective, parameters, jacobianStep), false)

		val chiSquare = residuals.sumOf { it * it }
		// The usual unbiased estimate. With fewer residuals than parameters
		// there is nothing to estimate and the covariance is meaningless.
		val dof = m - n
		val residualVariance = if (dof > 0) chiSquare / dof else Double.NaN

		val analysis = analyse(jacobian, residualVariance, objective)

		return AutoBoneSolution.from(
			offsets = objective.adjustOffsets,
			parameters = parameters,
			heightIndex = objective.heightIndex,
			covariance = analysis.covariance,
			worstDirection = analysis.worstDirection,
			conditionNumber = analysis.conditionNumber,
			residualVariance = residualVariance,
			rms = sqrt(chiSquare / m),
			iterations = optimum?.iterations ?: 0,
			evaluations = objective.evaluations,
			converged = converged,
			message = message,
		)
	}

	/**
	 * Tikhonov weight: [relative] times the largest eigenvalue of the data
	 * `JᵀJ`, or 0 when regularisation is switched off.
	 *
	 * Relative rather than absolute because the data curvature here spans many
	 * orders of magnitude between recordings -- slide residuals scale with how
	 * fast the user moved and with the frame spacing -- so any fixed weight
	 * would be inert on one recording and dominant on the next. That is the
	 * failure mode `initialAdjustRate` has, and reproducing it under a new name
	 * would be no improvement.
	 *
	 * `λ_max` is bounded by the largest squared column norm times `n`, but it is
	 * cheaper here to take the exact value: the matrix is at most 10x10 and it
	 * has already been formed.
	 */
	private fun regularisationWeight(dataJacobian: Array<DoubleArray>, relative: Double): Double {
		if (relative <= 0.0) return 0.0
		val j = Array2DRowRealMatrix(dataJacobian, false)
		val jtj = j.transpose().multiply(j)
		val largest = try {
			EigenDecomposition(jtj).realEigenvalues.maxOrNull() ?: 0.0
		} catch (e: Exception) {
			LogManager.warning("[AutoBone/LM] could not size the prior from JᵀJ: ${e.message}")
			0.0
		}
		return if (largest > 0.0) relative * largest else 0.0
	}

	private class Analysis(
		val covariance: Array<DoubleArray>?,
		val worstDirection: AutoBoneSolution.Direction?,
		val conditionNumber: Double,
	)

	/**
	 * Covariance and identifiability from `JᵀJ`.
	 *
	 * Done through an eigendecomposition rather than a plain inverse because
	 * the eigenvalues are the interesting output in their own right. The
	 * smallest one names the parameter combination the recording failed to
	 * determine, which is what turns "your proportions look odd" into "record
	 * some knee flexion".
	 */
	private fun analyse(
		jacobian: RealMatrix,
		residualVariance: Double,
		objective: AutoBoneObjective,
	): Analysis {
		val n = jacobian.columnDimension
		val jtj = jacobian.transpose().multiply(jacobian)

		val eigen = try {
			EigenDecomposition(jtj)
		} catch (e: Exception) {
			LogManager.warning("[AutoBone/LM] could not decompose JᵀJ: ${e.message}")
			return Analysis(null, null, Double.NaN)
		}

		val eigenvalues = eigen.realEigenvalues
		val largest = eigenvalues.maxOrNull() ?: 0.0
		if (largest <= 0.0) return Analysis(null, null, Double.NaN)

		val smallest = eigenvalues.minOrNull() ?: 0.0
		val conditionNumber = if (smallest > 0.0) largest / smallest else Double.POSITIVE_INFINITY

		// Pseudo-inverse: directions carrying no information cannot be inverted,
		// so they are left out of the sum.
		//
		// Leaving them out is not the same as them not mattering, and getting
		// this wrong is worse than not reporting at all. A parameter that lives
		// mostly in a dropped direction has *unbounded* variance; omitting the
		// direction gives it a small one instead, so the least determined
		// parameters come back looking like the best determined. That is the
		// exact failure this covariance exists to prevent, so any parameter
		// with a real component in a dropped direction is flagged and reported
		// as infinite rather than as a number.
		val floor = largest * SINGULARITY_THRESHOLD
		val covariance = Array(n) { DoubleArray(n) }
		val unbounded = BooleanArray(n)
		var dropped = 0
		for (k in 0 until n) {
			val v = eigen.getEigenvector(k)
			val lambda = eigenvalues[k]
			if (lambda <= floor) {
				dropped++
				for (i in 0 until n) {
					if (v.getEntry(i) * v.getEntry(i) > UNBOUNDED_PROJECTION) unbounded[i] = true
				}
				continue
			}
			val scale = residualVariance / lambda
			for (i in 0 until n) {
				val vi = v.getEntry(i)
				for (j in 0 until n) {
					covariance[i][j] += scale * vi * v.getEntry(j)
				}
			}
		}
		if (dropped > 0) {
			val names = objective.adjustOffsets.indices
				.filter { unbounded[it] }
				.joinToString { objective.adjustOffsets[it].configKey }
			LogManager.warning(
				"[AutoBone/LM] $dropped of $n parameter directions are unconstrained by this " +
					"recording. Affected parameters are reported as unbounded rather than as a " +
					"number: $names",
			)
		}
		for (i in 0 until n) {
			if (unbounded[i]) covariance[i][i] = Double.POSITIVE_INFINITY
		}

		// The worst-determined direction is the eigenvector with the smallest
		// eigenvalue, and its 1σ is sqrt(s²/λ).
		val worstIndex = eigenvalues.indices.minByOrNull { eigenvalues[it] }
		val worst = worstIndex?.let { index ->
			val lambda = max(eigenvalues[index], floor)
			val v = eigen.getEigenvector(index)
			val components = LinkedHashMap<String, Double>()
			for (i in objective.adjustOffsets.indices) {
				components[objective.adjustOffsets[i].configKey] = v.getEntry(i)
			}
			if (objective.heightIndex >= 0) {
				components[AutoBoneSolution.HEIGHT] = v.getEntry(objective.heightIndex)
			}
			AutoBoneSolution.Direction(components, sqrt(residualVariance / lambda))
		}

		return Analysis(covariance, worst, conditionNumber)
	}

	/**
	 * Central-difference Jacobian, `m × n`.
	 *
	 * `2n` residual evaluations. Each one re-solves both skeletons over every
	 * sampled frame pair, so this is where essentially all the time goes.
	 */
	private fun jacobian(
		objective: AutoBoneObjective,
		parameters: DoubleArray,
		step: Double,
	): Array<DoubleArray> {
		val n = parameters.size
		val m = objective.residualCount
		val out = Array(m) { DoubleArray(n) }
		val probe = parameters.copyOf()

		for (j in 0 until n) {
			// Scale-relative step with an absolute floor, so a parameter that
			// happens to sit near ln(L) = 0 still gets a usable perturbation.
			val h = step * max(1.0, abs(parameters[j]))

			probe[j] = parameters[j] + h
			val plus = objective.residuals(probe)
			probe[j] = parameters[j] - h
			val minus = objective.residuals(probe)
			probe[j] = parameters[j]

			val scale = 1.0 / (2.0 * h)
			for (i in 0 until m) {
				out[i][j] = (plus[i] - minus[i]) * scale
			}
		}
		return out
	}
}
