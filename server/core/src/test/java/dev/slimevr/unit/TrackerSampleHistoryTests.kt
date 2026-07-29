package dev.slimevr.unit

import dev.slimevr.tracking.trackers.TrackerSampleHistory
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Unit tests for the ring buffer behind time alignment.
 *
 * The interesting cases are all about what happens at the edges of the retained
 * window, because that is where a lookup can quietly return something wrong
 * instead of returning nothing.
 */
class TrackerSampleHistoryTests {

	/** Rotation of [deg] degrees about Y, the axis heading errors live on. */
	private fun yaw(deg: Float): Quaternion {
		val h = (deg * PI / 180.0 / 2.0).toFloat()
		return Quaternion(cos(h), 0f, sin(h), 0f)
	}

	/** Angle between two rotations, in degrees. */
	private fun angleDeg(a: Quaternion, b: Quaternion): Float = (a.angleToR(b) * 180.0 / PI).toFloat()

	@Test
	fun anEmptyHistoryResolvesToNothing() {
		val history = TrackerSampleHistory()
		assertNull(
			history.rotationAt(1000),
			"an empty history reported a rotation it cannot have",
		)
		assertEquals(0, history.size)
		assertEquals(0L, history.newestMicros)
	}

	@Test
	fun aSingleSampleResolvesToItself() {
		val history = TrackerSampleHistory()
		history.record(1000, yaw(30f))

		for (t in listOf(0L, 1000L, 5000L)) {
			assertTrue(
				angleDeg(assertNotNull(history.rotationAt(t)), yaw(30f)) < 1e-3f,
				"one sample is the only answer available at t=$t",
			)
		}
	}

	@Test
	fun interpolationIsLinearInAngleBetweenSamples() {
		val history = TrackerSampleHistory()
		history.record(1_000_000, yaw(0f))
		history.record(1_010_000, yaw(90f))

		// Slerp between two rotations about a common axis is linear in angle,
		// so these are exact expectations rather than approximations.
		for ((t, expected) in listOf(
			1_000_000L to 0f,
			1_002_500L to 22.5f,
			1_005_000L to 45f,
			1_007_500L to 67.5f,
			1_010_000L to 90f,
		)) {
			val actual = assertNotNull(history.rotationAt(t))
			assertTrue(
				angleDeg(actual, yaw(expected)) < 0.01f,
				"at t=$t expected ${expected}deg, got ${angleDeg(actual, yaw(0f))}deg",
			)
		}
	}

	/**
	 * The property that makes alignment safe: asking for a time past the last
	 * sample returns that sample, not a rotation extrapolated beyond it.
	 * Extrapolating would invent motion the tracker never reported, which is
	 * the failure this mechanism exists to remove rather than to introduce.
	 */
	@Test
	fun aReferencePastTheNewestSampleIsClampedNotExtrapolated() {
		val history = TrackerSampleHistory()
		history.record(1_000_000, yaw(0f))
		history.record(1_010_000, yaw(10f))

		val far = assertNotNull(history.rotationAt(1_100_000))
		assertTrue(
			angleDeg(far, yaw(10f)) < 1e-3f,
			"a reference 90ms past the newest sample was extrapolated to " +
				"${angleDeg(far, yaw(0f))}deg instead of held at 10deg",
		)
	}

	@Test
	fun aReferenceBeforeTheOldestSampleIsClamped() {
		val history = TrackerSampleHistory()
		history.record(1_000_000, yaw(0f))
		history.record(1_010_000, yaw(10f))

		val old = assertNotNull(history.rotationAt(900_000))
		assertTrue(angleDeg(old, yaw(0f)) < 1e-3f)
	}

	@Test
	fun theBufferWrapsAndKeepsTheNewestSamples() {
		val capacity = 8
		val history = TrackerSampleHistory(capacity)
		for (i in 0 until 100) {
			history.record(1_000_000L + i * 10_000L, yaw(i.toFloat()))
		}

		assertEquals(capacity, history.size)
		assertEquals(1_000_000L + 99 * 10_000L, history.newestMicros)
		assertEquals(1_000_000L + 92 * 10_000L, history.oldestMicros)

		// A lookup inside the surviving window must still interpolate
		// correctly after the ring has wrapped a dozen times.
		val mid = assertNotNull(history.rotationAt(1_000_000L + 95 * 10_000L + 5_000L))
		assertTrue(
			angleDeg(mid, yaw(95.5f)) < 0.01f,
			"interpolation broke after wrapping: got ${angleDeg(mid, yaw(0f))}deg",
		)
	}

	@Test
	fun samplesThatDoNotAdvanceTimeAreRejected() {
		val history = TrackerSampleHistory()
		history.record(1_000_000, yaw(0f))
		history.record(1_010_000, yaw(10f))
		history.record(1_005_000, yaw(99f))
		history.record(1_010_000, yaw(99f))

		assertEquals(2, history.size, "an out-of-order sample entered the buffer")
		assertEquals(2L, history.outOfOrderSamples)
		assertTrue(
			angleDeg(assertNotNull(history.rotationAt(1_010_000)), yaw(10f)) < 1e-3f,
			"the rejected sample changed the answer anyway",
		)
	}

	/**
	 * A reconnect or a rebuilt clock estimate moves every future timestamp into
	 * what the buffer considers the past. Without treating that as an epoch
	 * change the tracker would reject its own samples forever and drop out of
	 * alignment while still looking healthy.
	 */
	@Test
	fun aClockDiscontinuityRestartsTheHistory() {
		val history = TrackerSampleHistory()
		history.record(10_000_000, yaw(0f))
		history.record(10_010_000, yaw(10f))

		history.record(2_000_000, yaw(50f))
		assertEquals(1, history.discontinuities)
		assertEquals(1, history.size)

		history.record(2_010_000, yaw(60f))
		assertEquals(2, history.size)
		assertEquals(2_010_000L, history.newestMicros)
		assertTrue(
			angleDeg(assertNotNull(history.rotationAt(2_005_000)), yaw(55f)) < 0.01f,
			"the history did not resume interpolating after the discontinuity",
		)
	}

	@Test
	fun clearingLeavesNothingBehind() {
		val history = TrackerSampleHistory()
		history.record(1_000_000, yaw(0f))
		history.record(1_010_000, yaw(10f))
		history.clear()

		assertEquals(0, history.size)
		assertEquals(0L, history.newestMicros)
		assertNull(history.rotationAt(1_005_000))
	}

	/**
	 * Interpolation must take the shortest path. Two samples 350 degrees apart
	 * describe a 10 degree rotation the short way; going the long way round
	 * would spin the limb almost fully backwards between two adjacent frames.
	 */
	@Test
	fun interpolationTakesTheShortWayRound() {
		val history = TrackerSampleHistory()
		history.record(1_000_000, yaw(175f))
		history.record(1_010_000, yaw(-175f))

		val mid = assertNotNull(history.rotationAt(1_005_000))
		val toStart = angleDeg(mid, yaw(175f))
		assertTrue(
			abs(toStart - 5f) < 0.01f,
			"midpoint sits ${toStart}deg from the first sample; expected 5deg, " +
				"so the interpolation went the long way round",
		)
	}
}
