package dev.slimevr.replay

import dev.slimevr.metrics.PoseMetrics
import dev.slimevr.metrics.PoseMetricsAccumulator
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.poseframeformat.player.TrackerFramesPlayer
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.trackers.TrackerRole

/**
 * Drives a `.pfr` recording through the real pipeline and reduces it to the
 * same [PoseMetrics] the synthetic suite uses.
 *
 * This is deliberately the *only* thing that differs between the synthetic and
 * corpus paths: the motion source. [PoseMetricsAccumulator], [ReplayBaseline],
 * [FixedStepClock] and the configuration matrix are shared, so a corpus metric
 * and a synthetic metric of the same name mean the same thing and a change to
 * the metrics moves both together.
 *
 * @see CorpusRecording for why the sample rate is a caller argument rather than
 * something read out of the file.
 */
object CorpusReplay {

	/**
	 * The configurations each recording is replayed under, matching
	 * [SkeletonReplayTest]. The suffix becomes part of the baseline key.
	 */
	val configurations = listOf(
		"" to false,
		"+legtweaks" to true,
	)

	/** Baseline key for a corpus metric. Namespaced so a recording can never collide with a synthetic sequence name. */
	fun key(recording: String, suffix: String, metric: String): String = "corpus:$recording$suffix/$metric"

	fun replay(
		frames: PoseFrames,
		rateHz: Float,
		enableSkatingCorrection: Boolean = false,
		offsets: Map<SkeletonConfigOffsets, Float> = emptyMap(),
		clock: FixedStepClock = FixedStepClock(1f / rateHz),
	): PoseMetrics {
		require(rateHz > 0f) { "rateHz must be positive" }

		val player = TrackerFramesPlayer(frames)
		val hpm = HumanPoseManager(player.trackers.toList())

		// Capture-time proportions, where the sidecar records them. Applied
		// before the height is read, since height is derived from the offsets.
		for ((offset, value) in offsets) {
			hpm.setOffset(offset, value)
		}
		val height = hpm.userHeightFromConfig

		// Unlike the synthetic tests, `hasKneeTrackers` is not forced here:
		// HumanSkeleton derives it from the tracker list, and for a recording
		// that list is whatever was actually worn. Forcing it true would make a
		// five-point capture claim knee data it does not have, and LegTweaks
		// gates its floor clip on exactly that flag.

		hpm.setLegTweaksEnabled(enableSkatingCorrection)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, enableSkatingCorrection)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, enableSkatingCorrection)

		// Install the clocks last: every toggle setter above resets the frame
		// buffer, and so does assigning the clock. See SkeletonReplayTest.
		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val accumulator = PoseMetricsAccumulator()
		val dt = 1f / rateHz

		for (i in 0 until player.maxFrameCount) {
			// Advance before the update, so the frame this update produces is
			// exactly one timestep after the previous one.
			clock.advance()
			player.setCursors(i)
			hpm.update()

			// The computed trackers, not the skeleton bones -- the leg
			// corrections are invisible from the bones. See the replay README.
			accumulator.observeAnkles(
				hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position,
				hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position,
				dt,
			)
		}

		return accumulator.result(height)
	}
}
