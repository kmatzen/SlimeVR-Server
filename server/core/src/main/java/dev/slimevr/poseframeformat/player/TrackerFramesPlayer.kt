package dev.slimevr.poseframeformat.player

import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.poseframeformat.trackerdata.TrackerFrame
import dev.slimevr.poseframeformat.trackerdata.TrackerFrames
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.udp.IMUType

/**
 * @param imuType which IMU the recording was captured with, when known.
 * See [TrackerFrames.toTracker] -- without it Stay Aligned does not run on the
 * replayed trackers at all.
 */
class TrackerFramesPlayer(
	vararg val frameHolders: TrackerFrames,
	val imuType: IMUType? = null,
) {

	val playerTrackers: Array<PlayerTracker> = frameHolders.map { trackerFrames ->
		PlayerTracker(
			trackerFrames,
			trackerFrames.toTracker(imuType),
		)
	}.toTypedArray()

	val trackers: Array<Tracker> =
		playerTrackers.map { playerTracker -> playerTracker.tracker }.toTypedArray()

	/**
	 * @return The maximum number of [TrackerFrame]s contained within each
	 * [TrackerFrames] in the internal [TrackerFrames] array.
	 * @see [TrackerFrames.frames]
	 * @see [List.size]
	 */
	val maxFrameCount: Int
		get() {
			return frameHolders.maxOfOrNull { tracker -> tracker.frames.size } ?: 0
		}

	constructor(poseFrames: PoseFrames, imuType: IMUType? = null) :
		this(frameHolders = poseFrames.frameHolders.toTypedArray(), imuType = imuType)

	fun setCursors(index: Int) {
		for (playerTracker in playerTrackers) {
			playerTracker.cursor = index
		}
	}

	fun setScales(scale: Float) {
		for (playerTracker in playerTrackers) {
			playerTracker.scale = scale
		}
	}
}
