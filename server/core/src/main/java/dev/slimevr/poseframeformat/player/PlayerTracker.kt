package dev.slimevr.poseframeformat.player

import dev.slimevr.poseframeformat.trackerdata.TrackerFrames
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Quaternion

/**
 * @param timestampOrigin subtracted from every recorded sample timestamp, so a
 * replay does not depend on the capture machine's clock. Null replays the
 * recording untimestamped, which is what a `.pfr` written before the field
 * existed -- or by firmware that reports no sample times -- gets.
 */
class PlayerTracker(
	val trackerFrames: TrackerFrames,
	val tracker: Tracker,
	private var internalCursor: Int = 0,
	private var internalScale: Float = 1f,
	private val timestampOrigin: Long? = null,
) {

	var cursor: Int
		get() = internalCursor
		set(value) {
			val limitedCursor = limitCursor(value)
			internalCursor = limitedCursor
			setTrackerStateFromIndex(limitedCursor)
		}

	var scale: Float
		get() = internalScale
		set(value) {
			internalScale = value
			setTrackerStateFromIndex()
		}

	init {
		setTrackerStateFromIndex(limitCursor())
	}

	fun limitCursor(cursor: Int): Int {
		return if (cursor < 0 || trackerFrames.frames.isEmpty()) {
			return 0
		} else if (cursor >= trackerFrames.frames.size) {
			return trackerFrames.frames.size - 1
		} else {
			cursor
		}
	}

	fun limitCursor(): Int {
		val limitedCursor = limitCursor(internalCursor)
		internalCursor = limitedCursor
		return limitedCursor
	}

	/**
	 * Rebased time of the sample last pushed into the tracker's history.
	 *
	 * Needed because a cursor is not a clock: it can sit still, or be dragged
	 * backwards by the GUI scrubber, and each of those means something different
	 * for a history that must stay strictly increasing in time.
	 */
	private var lastAppliedSampleMicros: Long? = null

	/**
	 * Feeds one frame's rotation to the tracker, timestamped when the recording
	 * says so.
	 *
	 * [Tracker.setTimestampedRotation] is what builds the sample history
	 * [dev.slimevr.tracking.trackers.TimeAlignment] interpolates from. Without
	 * it every replayed tracker has an empty history, `isEligible` is false for
	 * all of them, and the alignment pass returns having touched nothing -- the
	 * replay completes, every metric is produced, and the correction never ran.
	 */
	private fun applyRotation(rotation: Quaternion, sampleMicros: Long?) {
		if (sampleMicros == null || timestampOrigin == null) {
			tracker.setRotation(rotation)
			return
		}

		val rebased = sampleMicros - timestampOrigin + REPLAY_EPOCH_MICROS
		val previous = lastAppliedSampleMicros

		when {
			// The ordinary case: time moved on, so this is a new sample.
			previous == null || rebased > previous -> {
				tracker.setTimestampedRotation(rotation, rebased)
				lastAppliedSampleMicros = rebased
			}

			// A seek backwards. The history describes samples that, from here,
			// have not happened yet; `record` would read the jump as a clock
			// discontinuity and drop the buffer anyway, so do it deliberately.
			rebased < previous -> {
				tracker.sampleHistory.clear()
				tracker.setTimestampedRotation(rotation, rebased)
				lastAppliedSampleMicros = rebased
			}

			// The same underlying sample, seen twice because the recorder ticked
			// faster than this tracker reported. Pushing it again would be
			// counted as an out-of-order sample, and that counter means "the
			// clock estimate is broken" -- which this is not. The rotation is
			// still applied; only the history is left alone, exactly as it would
			// be live when no new sample has arrived.
			else -> tracker.setRotation(rotation)
		}
	}

	private fun setTrackerStateFromIndex(index: Int = internalCursor) {
		val frame = trackerFrames.tryGetFrame(index) ?: return

		/*
		 * TODO: No way to set adjusted rotation manually? That might be nice to have...
		 * for now we'll stick with just setting the final rotation as raw and not
		 * enabling any adjustments
		 */

		val trackerPosition = frame.tryGetTrackerPosition()
		if (trackerPosition != null) {
			tracker.trackerPosition = trackerPosition
		}

		val rotation = frame.tryGetRotation()
		if (rotation != null) {
			applyRotation(rotation, frame.tryGetSampleServerMicros())
		}

		val position = frame.tryGetPosition()
		if (position != null) {
			tracker.position = position * internalScale
		}

		val acceleration = frame.tryGetAcceleration()
		if (acceleration != null) {
			tracker.setAcceleration(acceleration * internalScale)
		}
	}

	companion object {
		/**
		 * Where a rebased replay timeline starts.
		 *
		 * Not zero: `TimeAlignment.isEligible` reads
		 * `sampleHistory.newestMicros > 0`, using zero as the sentinel for "this
		 * tracker has never reported a timestamp". Rebasing the earliest sample
		 * to zero would make the first frame of every recording indistinguishable
		 * from an untimestamped one.
		 */
		const val REPLAY_EPOCH_MICROS = 1L
	}
}
