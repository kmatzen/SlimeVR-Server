package dev.slimevr.unit

import dev.slimevr.tracking.trackers.TimeAlignment
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Unit tests for the choice of reference instant.
 *
 * The interpolation itself is covered by [TrackerSampleHistoryTests]. What is
 * tested here is the decision of *which* instant to interpolate to, which is
 * where the failure modes are: a reference too far in the past adds latency to
 * every tracker to accommodate one, and a reference too far forward turns
 * interpolation into extrapolation.
 */
class TimeAlignmentTests {

	private fun yaw(deg: Float): Quaternion {
		val h = (deg * PI / 180.0 / 2.0).toFloat()
		return Quaternion(cos(h), 0f, sin(h), 0f)
	}

	private fun angleDeg(a: Quaternion, b: Quaternion): Float = (a.angleToR(b) * 180.0 / PI).toFloat()

	private fun mkTracker(id: Int, position: TrackerPosition): Tracker {
		val tracker = Tracker(
			device = null,
			id = id,
			name = position.name,
			trackerPosition = position,
			trackerNum = 0,
			hasPosition = false,
			hasRotation = true,
			isComputed = false,
			imuType = null,
			allowReset = false,
			allowMounting = false,
			isHmd = false,
			trackRotDirection = false,
		)
		tracker.status = TrackerStatus.OK
		return tracker
	}

	/**
	 * Fills a tracker with samples at [rateHz], the newest landing exactly on
	 * [newestMicros], each rotated by [degPerSecond] so that its rotation at a
	 * given instant is a known function of time.
	 */
	private fun fill(
		tracker: Tracker,
		newestMicros: Long,
		samples: Int = 16,
		rateHz: Float = 100f,
		degPerSecond: Float = 90f,
	) {
		val stepMicros = (1_000_000f / rateHz).toLong()
		for (i in samples - 1 downTo 0) {
			val t = newestMicros - i * stepMicros
			tracker.setTimestampedRotation(yaw(degPerSecond * t / 1_000_000f), t)
		}
	}

	@Test
	fun oneParticipantIsNotAlignedToAnything() {
		val alignment = TimeAlignment()
		val solo = mkTracker(0, TrackerPosition.CHEST)
		fill(solo, 10_000_000)
		val before = solo.getRawRotation()

		alignment.align(listOf(solo, mkTracker(1, TrackerPosition.HIP)))

		assertEquals(1, alignment.participants)
		assertEquals(0L, alignment.referenceMicros, "a lone tracker was given a reference")
		assertTrue(
			angleDeg(solo.getRawRotation(), before) < 1e-4f,
			"a lone tracker's rotation was changed; with nothing to agree with, " +
				"there is nothing to correct",
		)
	}

	/**
	 * Trackers without timestamps are invisible to this. That is what makes the
	 * whole mechanism inert on firmware that cannot report them, and is why it
	 * needs no setting to turn off.
	 */
	@Test
	fun untimestampedTrackersAreLeftAlone() {
		val alignment = TimeAlignment()
		val old = mkTracker(0, TrackerPosition.CHEST)
		old.setRotation(yaw(42f))
		val other = mkTracker(1, TrackerPosition.HIP)
		other.setRotation(yaw(7f))

		alignment.align(listOf(old, other))

		assertEquals(0, alignment.participants)
		assertTrue(angleDeg(old.getRawRotation(), yaw(42f)) < 1e-4f)
		assertTrue(angleDeg(other.getRawRotation(), yaw(7f)) < 1e-4f)
	}

	/**
	 * The core behaviour: the reference is the oldest of the newest samples, so
	 * every participant interpolates and none extrapolates. After the pass all
	 * of them describe that one instant.
	 */
	@Test
	fun everyoneResolvesToTheOldestNewestSample() {
		val alignment = TimeAlignment()
		val chest = mkTracker(0, TrackerPosition.CHEST)
		val hip = mkTracker(1, TrackerPosition.HIP)
		val thigh = mkTracker(2, TrackerPosition.LEFT_UPPER_LEG)

		// Same motion, sampled at instants 4ms and 11ms apart.
		fill(chest, 10_000_000)
		fill(hip, 9_996_000)
		fill(thigh, 9_989_000)

		alignment.align(listOf(chest, hip, thigh))

		assertEquals(3, alignment.participants)
		assertEquals(3, alignment.interpolated)
		assertEquals(9_989_000L, alignment.referenceMicros)
		assertEquals(11_000L, alignment.spreadMicros)

		// 90 deg/s at t = 9.989 s.
		val expected = yaw(90f * 9.989f)
		for (tracker in listOf(chest, hip, thigh)) {
			assertTrue(
				angleDeg(tracker.getRawRotation(), expected) < 0.05f,
				"${tracker.name} did not resolve to the reference instant: " +
					"off by ${angleDeg(tracker.getRawRotation(), expected)}deg",
			)
		}
	}

	/**
	 * One tracker that has stopped reporting must not hold the whole pose in
	 * the past. Beyond the skew bound the reference stops following it, and the
	 * straggler is clamped to its own last sample -- which is exactly what the
	 * server did with it before alignment existed.
	 */
	@Test
	fun aStragglerDoesNotDragTheReferenceBackWithIt() {
		val maxSkew = 50_000L
		val alignment = TimeAlignment(maxSkewMicros = maxSkew)
		val chest = mkTracker(0, TrackerPosition.CHEST)
		val hip = mkTracker(1, TrackerPosition.HIP)
		val stalled = mkTracker(2, TrackerPosition.LEFT_UPPER_LEG)

		fill(chest, 10_000_000)
		fill(hip, 9_998_000)
		fill(stalled, 9_000_000)

		alignment.align(listOf(chest, hip, stalled))

		assertEquals(
			10_000_000L - maxSkew,
			alignment.referenceMicros,
			"the reference followed a tracker one second behind the others",
		)
		assertEquals(1_000_000L, alignment.spreadMicros)
		assertEquals(2, alignment.interpolated, "the straggler was counted as interpolated")
		assertEquals(1L, alignment.stragglerPasses)

		// The straggler is held at its own newest sample, not extrapolated
		// forward to a reference it has no information about.
		assertTrue(
			angleDeg(stalled.getRawRotation(), yaw(90f * 9.0f)) < 0.05f,
			"the straggler was extrapolated forward instead of held",
		)
	}

	/**
	 * A timed-out tracker keeps its last sample time forever. If it still voted
	 * on the reference it would pin the pose to the moment it disappeared until
	 * the skew bound caught up -- and the bound is a backstop, not a policy.
	 */
	@Test
	fun trackersThatAreNotSendingDataDoNotVote() {
		val alignment = TimeAlignment()
		val chest = mkTracker(0, TrackerPosition.CHEST)
		val hip = mkTracker(1, TrackerPosition.HIP)
		val gone = mkTracker(2, TrackerPosition.LEFT_UPPER_LEG)

		fill(chest, 10_000_000)
		fill(hip, 9_997_000)
		fill(gone, 9_500_000)
		gone.status = TrackerStatus.TIMED_OUT

		alignment.align(listOf(chest, hip, gone))

		assertEquals(2, alignment.participants)
		assertEquals(9_997_000L, alignment.referenceMicros)
		assertEquals(0L, alignment.stragglerPasses)
	}

	/**
	 * Aligning trackers that already agree must not move them. Alignment runs
	 * on every solve, so a no-op case that is not actually a no-op would inject
	 * a correction at frame rate.
	 */
	@Test
	fun alreadySynchronisedTrackersAreUnchanged() {
		val alignment = TimeAlignment()
		val chest = mkTracker(0, TrackerPosition.CHEST)
		val hip = mkTracker(1, TrackerPosition.HIP)
		fill(chest, 10_000_000)
		fill(hip, 10_000_000)

		val chestBefore = chest.getRawRotation()
		val hipBefore = hip.getRawRotation()

		alignment.align(listOf(chest, hip))

		assertEquals(0L, alignment.spreadMicros)
		assertTrue(angleDeg(chest.getRawRotation(), chestBefore) < 1e-4f)
		assertTrue(angleDeg(hip.getRawRotation(), hipBefore) < 1e-4f)
	}

	/**
	 * Repeated passes with no new samples must be idempotent. `applyTimeAlignment`
	 * overwrites the raw rotation, so if the history were not the source of
	 * truth each pass would interpolate from the previous pass's output and
	 * walk the rotation steadily backwards in time.
	 */
	@Test
	fun repeatedPassesDoNotDrift() {
		val alignment = TimeAlignment()
		val chest = mkTracker(0, TrackerPosition.CHEST)
		val hip = mkTracker(1, TrackerPosition.HIP)
		fill(chest, 10_000_000)
		fill(hip, 9_993_000)

		alignment.align(listOf(chest, hip))
		val afterFirst = chest.getRawRotation()

		repeat(50) { alignment.align(listOf(chest, hip)) }

		val drift = angleDeg(chest.getRawRotation(), afterFirst)
		assertTrue(
			abs(drift) < 1e-4f,
			"50 alignment passes with no new samples moved the rotation by ${drift}deg; " +
				"the history is not the source of truth",
		)
	}
}
