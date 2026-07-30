package dev.slimevr.tracking.processor.skeleton

import io.github.axisangles.ktmath.Vector3

/**
 * Labels foot contact in a recorded sequence, offline, using the whole
 * recording.
 *
 * ## What this is for
 *
 * Issue #5 proposes replacing the threshold heuristics in
 * [LegTweaksBuffer.checkState] with a learned classifier, and the hard part of
 * that is not the classifier -- it is the labels. Of the three sources the issue
 * lists, it recommends starting with self-supervised labelling of the project's
 * own `.pfr` recordings, on the grounds that
 *
 * > An offline labeller can be far more accurate than a real-time detector
 * > because it can look ahead -- that asymmetry is the whole trick.
 *
 * This is that labeller. It is not a detector and must never be used as one: it
 * reads frames after the one it is labelling, so it cannot run live. That is the
 * entire point. A runtime detector has to decide whether the foot is planted
 * *now*, knowing only the past; this decides it knowing how the trajectory
 * continued, which is a strictly easier question and a much better-posed one.
 *
 * The asymmetry is not assumed here. `ContactDetectionTest` measures both
 * against ground truth on the same sequences, because if the offline labeller
 * were not actually better then the training data it produces would be no better
 * than the heuristic it is meant to improve on, and the whole plan would be
 * circular.
 *
 * ## The method
 *
 * A foot is in contact when it is not moving. Offline, "not moving" can be
 * asked over a window centred on the frame in question rather than a window
 * ending at it:
 *
 * 1. **Centred stillness.** Over `±[stillnessWindowFrames]`, take the largest
 *    distance between any sampled position and the position at the frame being
 *    labelled. Below [stillnessRadiusM], the foot is planted.
 * 2. **Height.** A foot well above the floor is not in contact regardless of how
 *    still it is -- a foot held motionless in the air is still, and a runtime
 *    detector has the same rule for the same reason.
 * 3. **Minimum durations.** Contact and flight segments shorter than
 *    [minSegmentFrames] are removed. Real feet do not touch down for 20 ms and
 *    leave again; a one-frame flicker is a measurement artefact, and offline
 *    there is no reason to keep it.
 *
 * Step 1 is what a causal detector cannot do. At the instant of liftoff the past
 * looks exactly like continued stance -- the foot has not moved yet -- so a
 * causal detector cannot know liftoff has happened until enough motion has
 * accumulated to cross a threshold, which is necessarily late. A centred window
 * sees the motion that is about to happen and puts the transition where it
 * belongs.
 *
 * ## What this deliberately does not do
 *
 * Estimate the floor. [floorLevelM] is a parameter because a recording of a
 * seated or kneeling user has no single floor, and guessing one from the data is
 * a separate problem that would silently corrupt every label when it guessed
 * wrong. Callers with real recordings should pass the calibrated floor the
 * recording was made with.
 */
class OfflineContactLabeller(
	/** Half-width of the centred stillness window, in frames. */
	val stillnessWindowFrames: Int = 3,

	/**
	 * How far a planted foot is allowed to move across the window, in metres.
	 *
	 * This is a distance rather than a speed on purpose. A speed threshold has
	 * to be compared against a frame interval and so changes meaning with the
	 * capture rate; a planted foot is one that is in the same place a moment
	 * later, at any rate.
	 */
	val stillnessRadiusM: Float = 0.02f,

	/** Height above [floorLevelM] beyond which a foot is not in contact. */
	val floorDistanceCutoffM: Float = 0.065f,

	val floorLevelM: Float = 0f,

	/** Shortest run of frames that may be called contact, or called flight. */
	val minSegmentFrames: Int = 3,
) {

	/**
	 * Label one foot's trajectory.
	 *
	 * [positions] is the whole recording, in order. The result has one entry per
	 * input frame.
	 */
	fun label(positions: List<Vector3>): BooleanArray {
		val raw = BooleanArray(positions.size)

		for (i in positions.indices) {
			raw[i] = isStill(positions, i) && isNearFloor(positions[i])
		}

		return removeShortSegments(raw)
	}

	/**
	 * True when nothing within the centred window is further than
	 * [stillnessRadiusM] from the frame being labelled.
	 *
	 * Deliberately a maximum rather than a mean. A mean over the window is small
	 * whenever the foot is still for most of it, which is exactly the situation
	 * at a transition -- so averaging would smear liftoff later and touchdown
	 * earlier, reintroducing the lag this exists to avoid.
	 */
	private fun isStill(positions: List<Vector3>, index: Int): Boolean {
		val here = positions[index]
		val from = maxOf(0, index - stillnessWindowFrames)
		val to = minOf(positions.size - 1, index + stillnessWindowFrames)

		for (j in from..to) {
			if ((positions[j] - here).len() > stillnessRadiusM) return false
		}
		return true
	}

	private fun isNearFloor(position: Vector3): Boolean = position.y <= floorLevelM + floorDistanceCutoffM

	/**
	 * Drop runs shorter than [minSegmentFrames], shortest first.
	 *
	 * Shortest first, and re-scanned after each pass, because removing a short
	 * run merges its neighbours -- and two runs that were each long enough on
	 * their own may leave a merged run that swallows a third. Sweeping once in
	 * index order would leave that dependent on which end the sweep started
	 * from.
	 */
	private fun removeShortSegments(raw: BooleanArray): BooleanArray {
		if (raw.isEmpty() || minSegmentFrames <= 1) return raw

		val out = raw.copyOf()
		while (true) {
			val segments = segmentsOf(out)

			// A recording that is one segment has nothing to merge into.
			if (segments.size <= 1) break

			val shortest = segments.filter { it.length < minSegmentFrames }.minByOrNull { it.length } ?: break

			for (i in shortest.start..shortest.end) out[i] = !out[i]
		}
		return out
	}

	private data class Segment(val start: Int, val end: Int) {
		val length: Int get() = end - start + 1
	}

	private fun segmentsOf(values: BooleanArray): List<Segment> {
		val segments = mutableListOf<Segment>()
		var start = 0
		for (i in 1..values.size) {
			if (i == values.size || values[i] != values[start]) {
				segments.add(Segment(start, i - 1))
				start = i
			}
		}
		return segments
	}
}
