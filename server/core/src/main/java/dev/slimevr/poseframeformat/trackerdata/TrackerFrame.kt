package dev.slimevr.poseframeformat.trackerdata

import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3

data class TrackerFrame(
	val trackerPosition: TrackerPosition? = null,
	val rotation: Quaternion? = null,
	val position: Vector3? = null,
	val acceleration: Vector3? = null,
	val rawRotation: Quaternion? = null,
	/**
	 * Instant the sample behind this frame was taken, in the server's timebase,
	 * or null when the tracker's firmware does not report sample timestamps.
	 *
	 * Recorded per frame rather than per recording because the quantity that
	 * matters is the *difference* between trackers at one instant. That spread
	 * is what [dev.slimevr.tracking.trackers.TimeAlignment] exists to remove,
	 * and it is not recoverable from a uniform frame rate: the recorder ticks
	 * evenly, the trackers do not.
	 *
	 * Absolute values are the capture machine's clock and mean nothing on
	 * replay; [dev.slimevr.poseframeformat.player.TrackerFramesPlayer] rebases
	 * them. Stored raw so the file stays a faithful record.
	 */
	val sampleServerMicros: Long? = null,
) {
	val dataFlags: Int

	val name: String
		get() = "TrackerFrame:/${trackerPosition?.designation ?: "null"}"

	init {
		var initDataFlags = 0

		if (trackerPosition != null) {
			initDataFlags = TrackerFrameData.TRACKER_POSITION_ENUM.add(initDataFlags)
		}
		if (rotation != null) {
			initDataFlags = TrackerFrameData.ROTATION.add(initDataFlags)
		}
		if (position != null) {
			initDataFlags = TrackerFrameData.POSITION.add(initDataFlags)
		}
		if (acceleration != null) {
			initDataFlags = TrackerFrameData.ACCELERATION.add(initDataFlags)
		}
		if (rawRotation != null) {
			initDataFlags = TrackerFrameData.RAW_ROTATION.add(initDataFlags)
		}
		if (sampleServerMicros != null) {
			initDataFlags = TrackerFrameData.SAMPLE_TIMESTAMP.add(initDataFlags)
		}

		dataFlags = initDataFlags
	}

	fun hasData(flag: TrackerFrameData): Boolean = flag.check(dataFlags)

	// region Tracker Try Getters
	fun tryGetTrackerPosition(): TrackerPosition? = if (hasData(TrackerFrameData.TRACKER_POSITION_ENUM) || hasData(TrackerFrameData.DESIGNATION_STRING)) {
		trackerPosition
	} else {
		null
	}

	fun tryGetRotation(): Quaternion? = if (hasData(TrackerFrameData.ROTATION)) {
		rotation
	} else {
		null
	}

	fun tryGetRawRotation(): Quaternion? = if (hasData(TrackerFrameData.RAW_ROTATION)) {
		rawRotation
	} else {
		null
	}

	fun tryGetPosition(): Vector3? = if (hasData(TrackerFrameData.POSITION)) {
		position
	} else {
		null
	}

	fun tryGetAcceleration(): Vector3? = if (hasData(TrackerFrameData.ACCELERATION)) {
		acceleration
	} else {
		null
	}

	/**
	 * When the sample behind this frame was taken, or null if the recording
	 * carries no timestamps.
	 */
	fun tryGetSampleServerMicros(): Long? = if (hasData(TrackerFrameData.SAMPLE_TIMESTAMP)) {
		sampleServerMicros
	} else {
		null
	}

	fun hasRotation(): Boolean = hasData(TrackerFrameData.ROTATION)

	fun hasPosition(): Boolean = hasData(TrackerFrameData.POSITION)

	fun hasAcceleration(): Boolean = hasData(TrackerFrameData.ACCELERATION)
	// endregion

	companion object {
		val empty = TrackerFrame()

		fun fromTracker(tracker: Tracker): TrackerFrame? {
			// If the tracker is not ready
			if (tracker.status != TrackerStatus.OK && tracker.status != TrackerStatus.BUSY && tracker.status != TrackerStatus.OCCLUDED) {
				return null
			}

			val trackerPosition = tracker.trackerPosition

			// If tracker has no data at all, there's no point in writing a frame
			// Note: This includes rawRotation because of `!tracker.hasRotation`
			if (trackerPosition == null && !tracker.hasRotation && !tracker.hasPosition && !tracker.hasAcceleration) {
				return null
			}

			val rotation: Quaternion? = if (tracker.hasRotation) tracker.getRotation() else null
			val position: Vector3? = if (tracker.hasPosition) tracker.position else null
			val acceleration: Vector3? = if (tracker.hasAcceleration) tracker.getAcceleration() else null

			var rawRotation: Quaternion? = if (tracker.hasAdjustedRotation) tracker.getRawRotation() else null
			// If the rawRotation is the same as rotation, there's no point in saving it, set it back to null
			if (rawRotation == rotation) rawRotation = null

			// Only when the firmware actually reports it. Zero means "never
			// timestamped", and writing that would make a recording claim every
			// tracker was sampled at the same instant -- the exact fiction the
			// field exists to avoid.
			val sampleServerMicros = tracker.lastSampleServerMicros.takeIf { it > 0L }

			return TrackerFrame(
				trackerPosition,
				rotation,
				position,
				acceleration,
				rawRotation,
				sampleServerMicros,
			)
		}
	}
}
