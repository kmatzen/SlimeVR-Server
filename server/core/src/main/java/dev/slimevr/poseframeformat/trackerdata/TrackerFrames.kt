package dev.slimevr.poseframeformat.trackerdata

import dev.slimevr.VRServer
import dev.slimevr.poseframeformat.trackerdata.TrackerFrame.Companion.fromTracker
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import io.eiren.util.collections.FastList

data class TrackerFrames(var name: String = "", val frames: FastList<TrackerFrame?>) {

	constructor(name: String = "", initialCapacity: Int = 5) : this(name, FastList<TrackerFrame?>(initialCapacity))
	constructor(baseTracker: Tracker, frames: FastList<TrackerFrame?>) : this(baseTracker.name, frames)
	constructor(baseTracker: Tracker, initialCapacity: Int = 5) : this(baseTracker, FastList<TrackerFrame?>(initialCapacity))

	fun addFrameFromTracker(index: Int, tracker: Tracker): TrackerFrame? {
		val trackerFrame = fromTracker(tracker)
		frames.add(index, trackerFrame)
		return trackerFrame
	}

	fun addFrameFromTracker(tracker: Tracker): TrackerFrame? {
		val trackerFrame = fromTracker(tracker)
		frames.add(trackerFrame)
		return trackerFrame
	}

	fun tryGetFrame(index: Int): TrackerFrame? = if (index < 0 || index >= frames.size) null else frames[index]

	fun tryGetFirstNotNullFrame(): TrackerFrame? = frames.firstOrNull { frame -> frame != null }

	/**
	 * Rebuilds a [Tracker] that replays this recording.
	 *
	 * @param imuType which IMU the recording was captured with, when known.
	 *
	 * The `.pfr` container does not store it -- the format is a tracker count,
	 * then per tracker a name, a frame count, and the frames -- so it can only
	 * come from the caller. Left null, the reconstructed tracker's
	 * `Tracker.isImu()` is false, and `AdjustTrackerYaw.adjust` returns on that
	 * before doing anything. **Stay Aligned is then inert for the whole replay,
	 * silently**, which matters because comparing Stay Aligned against the
	 * kinematic heading solve on real recordings is the open question on issue
	 * #3 and the corpus is what it is waiting for.
	 *
	 * Defaults to null so existing callers are unchanged: the AutoBone flow and
	 * the pose streamer replay recordings for their geometry, and neither runs
	 * yaw correction.
	 */
	@JvmOverloads
	fun toTracker(imuType: IMUType? = null): Tracker {
		val firstFrame = tryGetFirstNotNullFrame() ?: TrackerFrame.empty
		val tracker = Tracker(
			device = null,
			id = VRServer.getNextLocalTrackerId(),
			name = name,
			trackerPosition = firstFrame.tryGetTrackerPosition(),
			imuType = imuType,
			hasPosition = firstFrame.hasPosition(),
			hasRotation = firstFrame.hasRotation(),
			hasAcceleration = firstFrame.hasAcceleration(),
			// Make sure this is false!! Otherwise HumanSkeleton ignores it
			isInternal = false,
			isComputed = true,
			trackRotDirection = false,
		)

		tracker.status = TrackerStatus.OK

		return tracker
	}
}
