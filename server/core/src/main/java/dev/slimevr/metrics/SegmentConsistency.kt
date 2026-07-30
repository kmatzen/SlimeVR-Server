package dev.slimevr.metrics

import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs

/**
 * How far the pipeline's output has drifted from being a rigid skeleton.
 *
 * ## What this measures and why it is not covered by [PoseMetrics]
 *
 * Every existing pose metric asks whether the output is in the right *place* --
 * feet that slide, feet that sink through the floor. This asks something prior
 * to that: whether the output is a body at all.
 *
 * A skeleton has fixed segment lengths. Forward kinematics cannot violate them,
 * because it composes rotations down a tree of fixed-length bones. But
 * `LegTweaks` does not work that way. `correctClipping()` translates the ankle
 * and the knee by one displacement and the hip by a different one;
 * `correctSkating()` sets a locked ankle's horizontal position directly. Those
 * are independent translations of joint positions, and independent translations
 * of the endpoints of a segment change its length.
 *
 * So the corrected pose is not the forward-kinematic pose plus an error -- it is
 * not reachable by any configuration of the skeleton at all. The distance from
 * knee to ankle in the corrected output is no longer the length of the shin.
 *
 * ## Why this is the metric issue #4 wants
 *
 * Issue #4 argues that the pipeline's problem is architectural: a chain of
 * stages each correcting symptoms produced by the one before it, where a
 * constraint discovered late cannot inform the joint angles that caused the
 * violation. That is an argument about structure, and structural arguments are
 * hard to hold to account.
 *
 * This turns it into a number. `LegTweaks` cannot express "the foot is planted"
 * as a fact about the pose, because by the time it knows, the pose is already
 * built; all it can do is move the foot. The distance it moves the foot *by* is
 * the size of the constraint it could not apply, and that shows up here as a
 * segment that changed length.
 *
 * It also gives a joint estimator something concrete to beat rather than merely
 * replace. An estimator whose parameters are joint rotations cannot score
 * anything but zero here, because segment lengths are structural to its
 * parameterisation rather than something it has to be careful about.
 *
 * ## Unlike foot slide, this is nonzero on synthetic motion
 *
 * Issue #4 lists a third prerequisite beyond the two it opened with: recordings
 * with a nonzero residual (#15), because the corrected baselines are almost all
 * exactly zero once the leg corrections engage and so cannot express
 * degradation.
 *
 * That is a property of the metrics chosen, not of the pipeline. The corrections
 * drive foot slide to zero *by* deforming the skeleton, so the very frames where
 * slide reads zero are the frames where this reads largest. Measuring the cost
 * rather than the symptom needs no recordings.
 *
 * All lengths are metres.
 */
class SegmentConsistencyAccumulator {

	private var frames = 0
	private var violationSumM = 0f
	private var violationMaxM = 0f
	private var referenceSumM = 0f
	private var samples = 0

	/**
	 * Records one frame.
	 *
	 * Both quantities come from the same frame of the same run, which is what
	 * makes this exact rather than a comparison between two configurations that
	 * might have diverged for other reasons. `LegTweaks` writes its corrections
	 * into the computed trackers and leaves the bone tree alone, so the bone tree
	 * still holds the uncorrected forward-kinematic pose and is the reference for
	 * what each segment's length actually is.
	 */
	fun observe(skeleton: HumanSkeleton) {
		frames++

		observeSegment(
			skeleton.hipTrackerBone.getPosition(),
			skeleton.leftKneeTrackerBone.getPosition(),
			skeleton.computedHipTracker?.position,
			skeleton.computedLeftKneeTracker?.position,
		)
		observeSegment(
			skeleton.hipTrackerBone.getPosition(),
			skeleton.rightKneeTrackerBone.getPosition(),
			skeleton.computedHipTracker?.position,
			skeleton.computedRightKneeTracker?.position,
		)
		observeSegment(
			skeleton.leftKneeTrackerBone.getPosition(),
			skeleton.leftFootTrackerBone.getPosition(),
			skeleton.computedLeftKneeTracker?.position,
			skeleton.computedLeftFootTracker?.position,
		)
		observeSegment(
			skeleton.rightKneeTrackerBone.getPosition(),
			skeleton.rightFootTrackerBone.getPosition(),
			skeleton.computedRightKneeTracker?.position,
			skeleton.computedRightFootTracker?.position,
		)
	}

	private fun observeSegment(
		referenceHead: Vector3,
		referenceTail: Vector3,
		observedHead: Vector3?,
		observedTail: Vector3?,
	) {
		if (observedHead == null || observedTail == null) return

		val reference = (referenceTail - referenceHead).len()
		val observed = (observedTail - observedHead).len()
		val violation = abs(observed - reference)

		violationSumM += violation
		if (violation > violationMaxM) violationMaxM = violation
		referenceSumM += reference
		samples++
	}

	fun result(): SegmentConsistency = SegmentConsistency(
		frames = frames,
		meanViolationM = if (samples == 0) 0f else violationSumM / samples,
		maxViolationM = violationMaxM,
		meanViolationFraction = if (referenceSumM == 0f) 0f else violationSumM / referenceSumM,
	)
}

data class SegmentConsistency(
	val frames: Int,
	/** Mean absolute change in segment length, over every segment and frame. */
	val meanViolationM: Float,
	val maxViolationM: Float,
	/** The same, relative to the segment's own length. */
	val meanViolationFraction: Float,
) {
	fun toMap(): Map<String, Float> = linkedMapOf(
		"segment_violation_mean_m" to meanViolationM,
		"segment_violation_max_m" to maxViolationM,
		"segment_violation_fraction" to meanViolationFraction,
	)
}
