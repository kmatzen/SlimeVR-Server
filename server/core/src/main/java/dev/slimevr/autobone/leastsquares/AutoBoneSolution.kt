package dev.slimevr.autobone.leastsquares

import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import io.eiren.util.StringUtils
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * The result of a least-squares AutoBone solve, including how well each bone
 * was actually determined by the recording.
 *
 * ## Why the uncertainty is the point
 *
 * AutoBone currently returns a number per bone and nothing else, so a recording
 * in which the user never bent their knees produces a thigh/shin split that
 * looks exactly as confident as one from a recording full of squats. Issue #7
 * calls this out as the genuinely valuable half of moving to a second-order
 * solver, and it is: the covariance is available for free once a Jacobian
 * exists at the optimum.
 *
 * ## How the numbers are derived, and one easy way to get them wrong
 *
 * Commons-math's `Evaluation.getCovariances(threshold)` returns `(JᵀJ)⁻¹` and
 * its `getSigma` returns `sqrt(diag((JᵀJ)⁻¹))` -- neither is scaled by the
 * residual variance. Reporting `getSigma` directly is therefore wrong by a
 * factor of the noise level, and wrong in the flattering direction whenever the
 * fit is poor. The estimate used here is the standard one:
 *
 * ```
 * s² = Σr² / (m − n)          residual variance
 * Cov(θ) = s² · (JᵀJ)⁻¹       parameter covariance, in log-length space
 * ```
 *
 * ## Log space to metres
 *
 * Parameters are `θ = ln(L)`, so the delta method gives `Cov(L) = D Cov(θ) D`
 * with `D = diag(L)`, i.e. `σ_L = L · σ_θ`. A log-space sigma is already a
 * relative uncertainty, which is usually the more useful number to show a user:
 * `σ_θ = 0.05` means "this bone is determined to about ±5%".
 *
 * All lengths here are in the same normalised units the solve ran in unless
 * [scaled] has been applied.
 */
class AutoBoneSolution(
	val lengths: LinkedHashMap<SkeletonConfigOffsets, Float>,
	/** 1σ uncertainty per bone, in the same units as [lengths]. */
	val sigma: LinkedHashMap<SkeletonConfigOffsets, Float>,
	/** 1σ uncertainty per bone as a fraction of its own length. */
	val relativeSigma: LinkedHashMap<SkeletonConfigOffsets, Float>,
	/** Correlation matrix in parameter order, or null if `JᵀJ` was singular. */
	val correlation: Array<DoubleArray>?,
	/** Worst-determined parameter combination, or null if unavailable. */
	val worstDirection: Direction?,
	/** Solved user height, or null when it was held fixed. */
	val height: Float?,
	/** 1σ on [height] in metres, or null when it was held fixed. */
	val heightSigma: Float?,
	/** Ratio of the largest to smallest eigenvalue of `JᵀJ`. */
	val conditionNumber: Double,
	val residualVariance: Double,
	val rms: Double,
	val iterations: Int,
	val evaluations: Int,
	val converged: Boolean,
	val message: String,
) {
	/**
	 * A direction in parameter space and how well the recording determines it.
	 *
	 * Individually well-determined parameters can still leave a *combination*
	 * badly determined -- a recording with no knee flexion pins thigh + shin
	 * while saying almost nothing about thigh − shin. That is invisible in the
	 * diagonal of the covariance and obvious in its eigenvectors, and it is the
	 * shape of the "AutoBone gave me weird proportions" report where total limb
	 * length is right and the split is not.
	 */
	class Direction(
		/**
		 * Keyed by parameter name rather than by [SkeletonConfigOffsets],
		 * because height is a parameter too and is not a bone. Bones use their
		 * `configKey`; height uses [HEIGHT].
		 */
		val components: LinkedHashMap<String, Double>,
		/** 1σ along this direction, in log units, i.e. relative. */
		val sigma: Double,
	) {
		/** The parameters carrying this direction, largest contribution first. */
		fun describe(limit: Int = 3): String = components.entries
			.sortedByDescending { abs(it.value) }
			.take(limit)
			.joinToString(", ") { (name, weight) ->
				"${if (weight < 0) "−" else "+"}${StringUtils.prettyNumber(abs(weight).toFloat(), 2)}·$name"
			}
	}

	/** The same solution with every length and sigma multiplied by [scale]. */
	fun scaled(scale: Float): AutoBoneSolution = AutoBoneSolution(
		lengths = LinkedHashMap(lengths.mapValues { it.value * scale }),
		sigma = LinkedHashMap(sigma.mapValues { it.value * scale }),
		// Relative uncertainty is scale-free, which is the point of reporting it.
		relativeSigma = relativeSigma,
		correlation = correlation,
		worstDirection = worstDirection,
		// Height is deliberately not scaled: it is already in metres, and it is
		// the very quantity the normalised lengths are being scaled *by*.
		height = height,
		heightSigma = heightSigma,
		conditionNumber = conditionNumber,
		residualVariance = residualVariance,
		rms = rms,
		iterations = iterations,
		evaluations = evaluations,
		converged = converged,
		message = message,
	)

	/**
	 * Bones whose relative uncertainty exceeds [threshold].
	 *
	 * This is the query the UI wants: "which of these numbers should I not
	 * present as fact, and what motion would fix it".
	 */
	fun poorlyDetermined(threshold: Float = 0.1f): List<SkeletonConfigOffsets> = relativeSigma.entries
		// `!(x <= threshold)` rather than `x > threshold`, so an unbounded
		// parameter -- infinite or NaN -- is always counted. Those are the
		// worst determined of all, and a plain `>` would silently drop NaN.
		.filter { !(it.value <= threshold) }
		.sortedByDescending { it.value }
		.map { it.key }

	fun report(): String = buildString {
		append("[AutoBone/LM] ")
		append(if (converged) "converged" else "STOPPED")
		append(" in $iterations iterations, $evaluations objective evaluations")
		append(", rms ${StringUtils.prettyNumber(rms.toFloat(), 6)}")
		append(", condition ${StringUtils.prettyNumber(conditionNumber.toFloat(), 1)}")
		append(" ($message)\n")
		for ((offset, length) in lengths) {
			val sd = sigma[offset] ?: 0f
			val rel = relativeSigma[offset] ?: 0f
			if (!sd.isFinite()) {
				append(
					"  %-22s %s cm, NOT DETERMINED by this recording\n".format(
						offset.configKey,
						StringUtils.prettyNumber(length * 100f, 2),
					),
				)
				continue
			}
			append(
				"  %-22s %s ± %s cm (± %s%%)\n".format(
					offset.configKey,
					StringUtils.prettyNumber(length * 100f, 2),
					StringUtils.prettyNumber(sd * 100f, 2),
					StringUtils.prettyNumber(rel * 100f, 1),
				),
			)
		}
		if (height != null) {
			append(
				"  %-22s %s ± %s cm\n".format(
					HEIGHT,
					StringUtils.prettyNumber(height * 100f, 2),
					StringUtils.prettyNumber((heightSigma ?: 0f) * 100f, 2),
				),
			)
		}
		worstDirection?.let {
			append(
				"  worst-determined combination: ± ${StringUtils.prettyNumber(it.sigma.toFloat() * 100f, 1)}% " +
					"along ${it.describe()}\n",
			)
		}
	}

	companion object {
		/** Parameter name used for the solved height, which is not a bone. */
		const val HEIGHT = "height"

		/**
		 * Builds a solution from the log-space covariance.
		 *
		 * [covariance] is expected to already carry the residual-variance
		 * scaling; see the class doc for why that is not commons-math's
		 * default.
		 */
		fun from(
			offsets: List<SkeletonConfigOffsets>,
			parameters: DoubleArray,
			/** Index of the height parameter, or -1 when it was held fixed. */
			heightIndex: Int,
			covariance: Array<DoubleArray>?,
			worstDirection: Direction?,
			conditionNumber: Double,
			residualVariance: Double,
			rms: Double,
			iterations: Int,
			evaluations: Int,
			converged: Boolean,
			message: String,
		): AutoBoneSolution {
			val lengths = LinkedHashMap<SkeletonConfigOffsets, Float>(offsets.size)
			val sigma = LinkedHashMap<SkeletonConfigOffsets, Float>(offsets.size)
			val relative = LinkedHashMap<SkeletonConfigOffsets, Float>(offsets.size)

			/** log-space σ for parameter [i], or NaN when unavailable. */
			fun logSigmaAt(i: Int): Double = covariance?.get(i)?.get(i)?.let { if (it > 0.0) sqrt(it) else 0.0 } ?: Double.NaN

			for (i in offsets.indices) {
				val length = kotlin.math.exp(parameters[i])
				lengths[offsets[i]] = length.toFloat()

				// Variance is in log space, so its square root is already a
				// relative uncertainty; the delta method turns it into metres.
				val logSigma = logSigmaAt(i)
				relative[offsets[i]] = logSigma.toFloat()
				sigma[offsets[i]] = (length * logSigma).toFloat()
			}

			// Height, if it was solved for, goes through exactly the same delta
			// method -- it is one more log-parameter, not a special case.
			val height = if (heightIndex >= 0) kotlin.math.exp(parameters[heightIndex]) else null
			val heightSigma = height?.let { (it * logSigmaAt(heightIndex)).toFloat() }

			val correlation = covariance?.let { cov ->
				Array(cov.size) { i ->
					DoubleArray(cov.size) { j ->
						val denom = sqrt(cov[i][i] * cov[j][j])
						if (denom > 0.0) cov[i][j] / denom else 0.0
					}
				}
			}

			return AutoBoneSolution(
				lengths = lengths,
				sigma = sigma,
				relativeSigma = relative,
				correlation = correlation,
				worstDirection = worstDirection,
				height = height?.toFloat(),
				heightSigma = heightSigma,
				conditionNumber = conditionNumber,
				residualVariance = residualVariance,
				rms = rms,
				iterations = iterations,
				evaluations = evaluations,
				converged = converged,
				message = message,
			)
		}
	}
}
