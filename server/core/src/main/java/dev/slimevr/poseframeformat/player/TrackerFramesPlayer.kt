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

	/**
	 * Earliest sample timestamp anywhere in the recording, or null when nothing
	 * in it is timestamped.
	 *
	 * Every replayed timestamp is expressed relative to this. The recorded
	 * values are the capture machine's clock, and only their *differences* carry
	 * meaning -- the spread between trackers at one instant, and the interval
	 * between one tracker's consecutive samples. Rebasing keeps both exactly
	 * while making a replay independent of when and where it was captured.
	 *
	 * Taken across all trackers rather than per tracker, because a per-tracker
	 * origin would zero the very between-tracker skew this exists to preserve.
	 */
	val timestampOrigin: Long? = frameHolders
		.flatMap { holder -> holder.frames.asSequence().mapNotNull { it?.sampleServerMicros }.asIterable() }
		.minOrNull()

	val playerTrackers: Array<PlayerTracker> = frameHolders.map { trackerFrames ->
		PlayerTracker(
			trackerFrames,
			trackerFrames.toTracker(imuType),
			timestampOrigin = timestampOrigin,
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
