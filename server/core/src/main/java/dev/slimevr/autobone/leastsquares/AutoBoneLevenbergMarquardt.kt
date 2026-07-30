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
 */
object AutoBoneLevenbergMarquardt {

	/**
	 * Eigenvalues of `JᵀJ` below this fraction of the largest are treated as
	 * structurally zero rather than inverted into an enormous variance.
	 */
	private const val SINGULARITY_THRESHOLD = 1e-12

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

		val model = MultivariateJacobianFunction { point: RealVector ->
			val parameters = point.toArray()
			val value = objective.residuals(parameters)
			MathPair.create<RealVector, RealMatrix>(
				ArrayRealVector(value, false),
				Array2DRowRealMatrix(jacobian(objective, parameters, jacobianStep), false),
			)
		}

		val builder = LeastSquaresBuilder()
			.start(initialParameters)
			.model(model)
			// The residuals are already the quantity to drive to zero, so the
			// target is the zero vector rather than a set of observations.
			.target(DoubleArray(m))
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
		val residuals = optimum?.residuals?.toArray() ?: objective.residuals(parameters)
		val jacobian = optimum?.jacobian ?: Array2DRowRealMatrix(
			jacobian(objective, parameters, jacobianStep),
			false,
		)

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

		// Pseudo-inverse: directions that carry no information are left out
		// rather than inverted into a variance of 1e30, which would swamp every
		// other entry once the eigenvectors are recombined.
		val floor = largest * SINGULARITY_THRESHOLD
		val covariance = Array(n) { DoubleArray(n) }
		var dropped = 0
		for (k in 0 until n) {
			val lambda = eigenvalues[k]
			if (lambda <= floor) {
				dropped++
				continue
			}
			val v = eigen.getEigenvector(k)
			val scale = residualVariance / lambda
			for (i in 0 until n) {
				val vi = v.getEntry(i)
				for (j in 0 until n) {
					covariance[i][j] += scale * vi * v.getEntry(j)
				}
			}
		}
		if (dropped > 0) {
			LogManager.warning(
				"[AutoBone/LM] $dropped of $n parameter directions are unconstrained by this " +
					"recording; their uncertainty is reported as zero because it is unbounded, not small",
			)
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
