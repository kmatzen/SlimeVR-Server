package dev.slimevr.tracking.processor.stayaligned

import dev.slimevr.VRServer
import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.math.Angle
import dev.slimevr.tracking.processor.stayaligned.StayAlignedDefaults.IMU_TO_YAW_CORRECTION
import dev.slimevr.tracking.processor.stayaligned.StayAlignedDefaults.YAW_CORRECTION_DEFAULT
import dev.slimevr.tracking.processor.stayaligned.adjust.AdjustTrackerYaw
import dev.slimevr.tracking.processor.stayaligned.trackers.TrackerSkeleton

/**
 * Manager to keep the trackers aligned.
 */
object StayAligned {

	private var nextTrackerIndex = 0

	/**
	 * Where the frame interval comes from, in seconds.
	 *
	 * The correction applied per tick is proportional to this, so it decides how
	 * fast Stay Aligned converges and is not an incidental detail.
	 *
	 * It was read directly from `VRServer.instance`, a `lateinit` global that
	 * throws when unset. That made Stay Aligned impossible to run in the replay
	 * suite at all -- not merely non-deterministic, but a crash on the first
	 * corrected frame -- which is why the whole `stayaligned` package has no test
	 * coverage while the estimator proposed to replace it has four test classes.
	 *
	 * That matters for issue #3 beyond tidiness. The comparison that issue is
	 * waiting on is Stay Aligned against the kinematic solve on the same session,
	 * and it was blocked on a recording corpus (#15). A corpus would not have
	 * unblocked it: replaying one drives the same code path and hits the same
	 * global.
	 *
	 * Same shape as the fix in #16, which took `LegTweaks` frame times from an
	 * injected clock for the same reason. Defaults to the server's timer, so
	 * production behaviour is unchanged.
	 */
	var frameIntervalSec: () -> Float = { VRServer.instance.fpsTimer.timePerFrame }

	/**
	 * Clears the round-robin cursor.
	 *
	 * [nextTrackerIndex] persists across skeletons because this is an object, so
	 * which tracker gets adjusted first depends on how many ticks any previous
	 * skeleton ran. In production there is one skeleton and it does not matter;
	 * under replay it means two runs of identical input start the rotation at
	 * different trackers and produce different output, which is the property a
	 * regression baseline cannot be built on.
	 */
	fun reset() {
		nextTrackerIndex = 0
	}

	/**
	 * Adjusts the yaw of the next tracker.
	 *
	 * We only adjust one tracker per tick to minimize CPU usage. When the server is
	 * running at 1000 Hz and there are 20 trackers, each tracker is still updated 50
	 * times a second.
	 */
	fun adjustNextTracker(trackers: TrackerSkeleton, config: StayAlignedConfig) {
		if (!config.enabled) {
			return
		}

		val numTrackers = trackers.allTrackers.size
		if (numTrackers == 0) {
			return
		}

		val trackerToAdjust = trackers.allTrackers[nextTrackerIndex % numTrackers]
		++nextTrackerIndex

		// Update hide correction since the config could have changed
		trackerToAdjust.stayAligned.hideCorrection = config.hideYawCorrection

		val yawCorrectionPerSec =
			IMU_TO_YAW_CORRECTION.getOrDefault(trackerToAdjust.imuType, YAW_CORRECTION_DEFAULT)
		if (yawCorrectionPerSec == Angle.ZERO) {
			return
		}

		// Scale yaw correction since we're only updating one tracker per tick
		val yawCorrection =
			yawCorrectionPerSec *
				frameIntervalSec() *
				numTrackers.toFloat()

		AdjustTrackerYaw.adjust(
			trackerToAdjust,
			trackers,
			yawCorrection,
			config,
		)
	}
}
