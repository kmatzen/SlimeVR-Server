package dev.slimevr.autobone.leastsquares

import dev.slimevr.autobone.AutoBoneStep
import dev.slimevr.autobone.PoseFrameStep
import dev.slimevr.autobone.errors.IAutoBoneError
import dev.slimevr.config.AutoBoneConfig
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigManager
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import kotlin.math.exp
import kotlin.math.ln
import kotlin.math.sqrt

/**
 * AutoBone's objective written as a residual *vector*, which is what a
 * least-squares solver needs.
 *
 * ## What this is not
 *
 * Issue #7 describes the existing optimiser as gradient descent over an
 * objective that is "already a sum of squared residuals". Neither half of that
 * is quite right, and the difference matters for what can be claimed
 * afterwards.
 *
 * `AutoBone.step()` is not gradient descent. For each bone in turn it proposes
 * a length change whose direction comes from [dev.slimevr.autobone.BoneContribution]
 * -- a geometric heuristic for how much that bone can move the foot along the
 * observed slide direction -- and keeps the change only if the error drops. It
 * is a greedy coordinate search with a heuristic direction and an accept/reject
 * test. No gradient is formed.
 *
 * The objective is not a sum of squares either. Per frame pair it computes
 * `errorDeriv = sum_k w_k * e_k`, a weighted sum of non-negative magnitudes,
 * and `0.5 * errorDeriv^2` is used only to size the step. The square of a sum
 * is not a sum of squares, and the accept/reject test is applied per frame
 * pair, so the procedure never descends any single global objective.
 *
 * The consequence for issue #7's proposed correctness test -- "LM and gradient
 * descent should converge to the same optimum" -- is that there is no shared
 * optimum to converge to until one is defined. This class defines it.
 *
 * ## The objective
 *
 * For enabled error terms *k* with configured weights *w_k*, and sampled frame
 * pairs *p*:
 *
 * ```
 * F(θ) = ½ Σ_p Σ_k (w_k / P) · e_k(p; θ)²  +  ½ · w_h · h(θ)²
 * ```
 *
 * so the residual vector is `r = [ sqrt(w_k/P)·e_k(p) ... , sqrt(w_h)·h ]`.
 * Dividing by the pair count *P* makes `F` a mean over the recording rather
 * than a sum, so the weights keep the meaning they have in the existing config
 * and the scale does not move when a recording gets longer.
 *
 * Every `e_k` is already non-negative and vanishes at a perfect fit, so
 * squaring them is well posed. What changes relative to the current scalar is
 * that cross terms disappear: `Σ (w e)²` rather than `(Σ w e)²`. That is the
 * standard least-squares reading of the same error terms, and it is the form
 * that makes a Jacobian, a covariance and a convergence criterion meaningful.
 *
 * ## Parameterisation: log lengths
 *
 * The parameter vector is `θ_i = ln(L_i)`, not `L_i`.
 *
 * Bone lengths are positive, and an unconstrained solver does not know that.
 * The existing code copes with a guard -- "No small or negative numbers!!! Bad
 * algorithm!" in `AutoBone.step()`, which silently skips any proposal below
 * 0.01 -- and a guard that silently skips proposals is a guard that can stall
 * the search without saying so. In log space a negative length is not
 * representable, so the constraint is structural and there is nothing to skip.
 *
 * It also makes the step scale-free: a 1% change in a 0.05 m neck and a 1%
 * change in a 0.45 m thigh are the same distance in θ, which is the right
 * behaviour for a solver that has one damping parameter shared across
 * parameters of very different magnitudes.
 *
 * Covariance comes back in log space and is converted to length units by the
 * delta method -- see [AutoBoneSolution].
 *
 * ## The height residual
 *
 * `h(θ) = Σ_{adjustable height bones} L_i − normalizedHeight`.
 *
 * The existing algorithm renormalises the height offsets after every step so
 * they always sum to the same total. Doing that *inside* a residual function
 * would make the objective exactly invariant to a uniform scaling of the height
 * bones, so `JᵀJ` would be exactly singular in that direction and the
 * covariance -- the thing this whole exercise is for -- would not exist.
 *
 * The constraint is therefore soft: a residual with a large weight, which pins
 * the same direction while leaving the Hessian non-singular and its
 * conditioning readable. [dev.slimevr.autobone.errors.BodyProportionError]
 * assumes the normalised skeleton has height 1, which stays true to the extent
 * this residual is satisfied.
 */
class AutoBoneObjective(
	private val step: PoseFrameStep<AutoBoneStep>,
	/** The bones being solved for, in parameter-vector order. */
	val adjustOffsets: List<SkeletonConfigOffsets>,
	/** Target for the sum of adjustable height bones, in normalised units. */
	private val normalizedHeight: Float,
	/** Frame pairs sampled from the recording, as `(cursor1, cursor2)`. */
	private val framePairs: List<Pair<Int, Int>>,
	private val terms: List<WeightedTerm>,
	private val heightConstraintWeight: Float,
) {
	/** One error term and the weight it carries in the objective. */
	class WeightedTerm(val name: String, val error: IAutoBoneError, val weight: Float)

	/** Which adjustable bones contribute to the height sum. */
	private val heightIndices: IntArray = adjustOffsets
		.withIndex()
		.filter { (_, offset) -> SkeletonConfigManager.HEIGHT_OFFSETS.contains(offset) }
		.map { (index, _) -> index }
		.toIntArray()

	private val pairScale: Double = 1.0 / framePairs.size

	val parameterCount: Int get() = adjustOffsets.size

	val residualCount: Int get() = framePairs.size * terms.size + 1

	val pairCount: Int get() = framePairs.size

	val termNames: List<String> get() = terms.map { it.name }

	/** Counts residual-vector evaluations, so cost can be reported honestly. */
	var evaluations: Int = 0
		private set

	fun toParameters(lengths: Map<SkeletonConfigOffsets, Float>): DoubleArray = DoubleArray(adjustOffsets.size) { i ->
		val length = lengths[adjustOffsets[i]]
			?: error("no initial length for ${adjustOffsets[i]}")
		require(length > 0f) { "${adjustOffsets[i]} has non-positive length $length" }
		ln(length.toDouble())
	}

	fun toLengths(parameters: DoubleArray): LinkedHashMap<SkeletonConfigOffsets, Float> {
		val lengths = LinkedHashMap<SkeletonConfigOffsets, Float>(adjustOffsets.size)
		for (i in adjustOffsets.indices) {
			lengths[adjustOffsets[i]] = exp(parameters[i]).toFloat()
		}
		return lengths
	}

	/**
	 * The residual vector at [parameters].
	 *
	 * Both skeletons are reconfigured and re-solved for every frame pair, which
	 * is the dominant cost and the reason [evaluations] is tracked: the fair
	 * comparison against the existing optimiser is in objective evaluations,
	 * not in iterations.
	 */
	fun residuals(parameters: DoubleArray): DoubleArray {
		evaluations++

		val lengths = DoubleArray(parameters.size) { exp(parameters[it]) }
		applyLengths(step.skeleton1, lengths)
		applyLengths(step.skeleton2, lengths)

		val out = DoubleArray(residualCount)
		var at = 0

		for ((cursor1, cursor2) in framePairs) {
			// setCursors(updatePlayerCursors = true) re-solves both skeletons,
			// so this has to happen after the offsets are applied.
			step.setCursors(cursor1, cursor2, updatePlayerCursors = true)
			for (term in terms) {
				out[at++] = sqrt(term.weight * pairScale) * term.error.getStepError(step)
			}
		}

		var heightSum = 0.0
		for (i in heightIndices) heightSum += lengths[i]
		out[at] = sqrt(heightConstraintWeight.toDouble()) * (heightSum - normalizedHeight)

		return out
	}

	/**
	 * The scalar the *existing* optimiser reduces, averaged over the same frame
	 * pairs: `mean_p( Σ_k w_k e_k(p) )`.
	 *
	 * Kept separate from [residuals] and deliberately not the quantity being
	 * minimised. It is the only well-defined common ground on which the two
	 * optimisers can be compared, because it is defined without reference to
	 * either one's update rule.
	 */
	fun meanStepError(parameters: DoubleArray): Double {
		val lengths = DoubleArray(parameters.size) { exp(parameters[it]) }
		applyLengths(step.skeleton1, lengths)
		applyLengths(step.skeleton2, lengths)

		var sum = 0.0
		for ((cursor1, cursor2) in framePairs) {
			step.setCursors(cursor1, cursor2, updatePlayerCursors = true)
			for (term in terms) {
				sum += term.weight * term.error.getStepError(step)
			}
		}
		return sum / framePairs.size
	}

	private fun applyLengths(manager: HumanPoseManager, lengths: DoubleArray) {
		for (i in adjustOffsets.indices) {
			manager.setOffset(adjustOffsets[i], lengths[i].toFloat())
		}
	}

	companion object {
		/**
		 * The error terms enabled by [config], in a fixed order.
		 *
		 * Mirrors `AutoBone.getErrorDeriv`: a factor of zero means the term is
		 * off, not that it contributes nothing at zero weight, so it is dropped
		 * from the residual vector entirely rather than contributing rows of
		 * zeros that would dilute the RMS.
		 */
		fun enabledTerms(config: AutoBoneConfig, errors: AutoBoneErrorSet): List<WeightedTerm> = buildList {
			if (config.slideErrorFactor > 0f) {
				add(WeightedTerm("slide", errors.slideError, config.slideErrorFactor))
			}
			if (config.offsetSlideErrorFactor > 0f) {
				add(WeightedTerm("offsetSlide", errors.offsetSlideError, config.offsetSlideErrorFactor))
			}
			if (config.footHeightOffsetErrorFactor > 0f) {
				add(WeightedTerm("footHeightOffset", errors.footHeightOffsetError, config.footHeightOffsetErrorFactor))
			}
			if (config.bodyProportionErrorFactor > 0f) {
				add(WeightedTerm("bodyProportion", errors.bodyProportionError, config.bodyProportionErrorFactor))
			}
			if (config.heightErrorFactor > 0f) {
				add(WeightedTerm("height", errors.heightError, config.heightErrorFactor))
			}
			if (config.positionErrorFactor > 0f) {
				add(WeightedTerm("position", errors.positionError, config.positionErrorFactor))
			}
			if (config.positionOffsetErrorFactor > 0f) {
				add(WeightedTerm("positionOffset", errors.positionOffsetError, config.positionOffsetErrorFactor))
			}
		}

		/**
		 * Frame pairs to evaluate, following the same cursor scheme the epoch
		 * loop in [dev.slimevr.autobone.PoseFrameIterator] uses.
		 *
		 * Capped at [maxPairs] by an even stride rather than at random or by
		 * truncation: truncating would use only the start of the recording, and
		 * a random subset would make the objective depend on a seed, which
		 * would put noise into a Jacobian computed by differencing it. A fixed
		 * stride keeps the objective a deterministic function of θ, which
		 * finite differences require.
		 */
		fun sampleFramePairs(
			frameCount: Int,
			config: AutoBoneConfig,
			maxPairs: Int,
		): List<Pair<Int, Int>> {
			val all = buildList {
				var cursorOffset = config.minDataDistance
				while (cursorOffset <= config.maxDataDistance && cursorOffset < frameCount) {
					var cursor = 0
					while (cursor < frameCount - cursorOffset) {
						add(cursor to cursor + cursorOffset)
						cursor += config.cursorIncrement
					}
					cursorOffset++
				}
			}
			if (maxPairs <= 0 || all.size <= maxPairs) return all

			val stride = all.size.toDouble() / maxPairs
			return (0 until maxPairs).map { all[(it * stride).toInt().coerceAtMost(all.size - 1)] }
		}
	}
}
