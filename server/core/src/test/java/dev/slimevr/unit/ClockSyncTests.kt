package dev.slimevr.unit

import dev.slimevr.tracking.trackers.udp.ClockSync
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * Tests for the tracker clock offset/rate estimator.
 *
 * These build synthetic exchanges from a known ground-truth offset and rate, so
 * the expected answer is exact and any error is the estimator's. That matters
 * more than usual here: clock skew is invisible in normal operation and only
 * shows up as limbs disagreeing about what time it is during fast motion, which
 * is very easy to misattribute to the IK solver.
 */
class ClockSyncTests {

	/**
	 * Simulates one exchange given a true offset and rate error.
	 *
	 * @param upMicros one-way delay server -> tracker
	 * @param downMicros one-way delay tracker -> server
	 */
	private fun exchange(
		sync: ClockSync,
		serverTx: Long,
		trueOffset: Long,
		skewPpm: Double = 0.0,
		upMicros: Long = 1000,
		downMicros: Long = 1000,
		turnaroundMicros: Long = 200,
	): Boolean {
		val trackerRxServerTime = serverTx + upMicros
		// The tracker's clock reads server time, plus the offset, stretched by
		// its rate error.
		val trackerRx = trackerRxServerTime +
			trueOffset +
			(trackerRxServerTime * skewPpm / 1e6).toLong()
		val trackerTx = trackerRx + turnaroundMicros
		val serverRx = trackerRxServerTime + turnaroundMicros + downMicros
		return sync.addExchange(serverTx, trackerRx, trackerTx, serverRx)
	}

	@Test
	fun recoversOffsetOnSymmetricPath() {
		val sync = ClockSync()
		val trueOffset = 1_234_567L

		var t = 0L
		repeat(20) {
			assertTrue(exchange(sync, t, trueOffset))
			t += 500_000
		}

		assertTrue(sync.hasEstimate)
		// With equal up and down delays the NTP estimator is exact.
		assertEquals(trueOffset, sync.offsetMicros, "offset should be recovered exactly")
		assertEquals(20, sync.acceptedSamples)
		assertEquals(0, sync.rejectedSamples)
	}

	@Test
	fun offsetErrorIsBoundedByPathAsymmetry() {
		// A path that is 4 ms slower one way biases the offset by half that.
		// This is a property of the algorithm, not a defect -- but it bounds how
		// good any single sample can be, which is why the estimator filters on
		// delay rather than averaging everything it sees.
		val sync = ClockSync()
		val trueOffset = 50_000L

		var t = 0L
		repeat(20) {
			exchange(sync, t, trueOffset, upMicros = 5000, downMicros = 1000)
			t += 500_000
		}

		val error = abs(sync.offsetMicros - trueOffset)
		assertTrue(
			error <= 2100,
			"asymmetry of 4 ms should bias the offset by about 2 ms, got $error",
		)
	}

	@Test
	fun rejectsDelayedSamples() {
		val sync = ClockSync()
		val trueOffset = 10_000L

		// Establish a good baseline delay.
		var t = 0L
		repeat(5) {
			assertTrue(exchange(sync, t, trueOffset, upMicros = 500, downMicros = 500))
			t += 500_000
		}
		val baseline = sync.offsetMicros

		// A badly delayed exchange must not be allowed to move the estimate.
		val accepted = exchange(
			sync,
			t,
			trueOffset,
			upMicros = 200_000,
			downMicros = 500,
		)
		assertFalse(accepted, "a grossly delayed sample should be rejected")
		assertEquals(1, sync.rejectedSamples)
		assertEquals(baseline, sync.offsetMicros, "rejected sample must not move the estimate")
	}

	@Test
	fun recoversClockRateError() {
		// 40 ppm is a realistic crystal error. Over a minute that is 2.4 ms of
		// divergence -- small per sample, but it is exactly the term that makes
		// an offset-only estimate go stale.
		val sync = ClockSync()
		val trueSkew = 40.0

		var t = 0L
		repeat(120) {
			exchange(sync, t, trueOffset = 1000, skewPpm = trueSkew)
			t += 500_000
		}

		assertTrue(sync.acceptedSamples > 100, "most samples should be accepted")
		assertTrue(
			abs(sync.skewPpm - trueSkew) < 5.0,
			"expected about $trueSkew ppm, got ${sync.skewPpm}",
		)
	}

	@Test
	fun reportsNoSkewBeforeEnoughSamples() {
		val sync = ClockSync()
		exchange(sync, 0, trueOffset = 1000, skewPpm = 40.0)
		exchange(sync, 500_000, trueOffset = 1000, skewPpm = 40.0)
		// Two points would fit a line perfectly and report nonsense; the
		// estimator must decline until it has a real baseline.
		assertEquals(0.0, sync.skewPpm)
		assertTrue(sync.hasEstimate)
	}

	@Test
	fun unwrapsThe32BitTrackerClock() {
		// The tracker's micros() counter wraps every ~71.6 minutes. Without
		// unwrapping, everything works fine and then breaks about an hour after
		// power-on, which is a uniquely unpleasant bug to hunt.
		val sync = ClockSync()
		val range = ClockSync.WRAP_RANGE

		assertEquals(range - 1000, sync.unwrapTrackerMicros(range - 1000))
		// Step forward across the wrap: raw goes 0xFFFFFC18 -> 0x000003E8.
		assertEquals(range + 1000, sync.unwrapTrackerMicros(1000))
		assertEquals(range + 2000, sync.unwrapTrackerMicros(2000))
		// And again, a second time round.
		assertEquals(2 * range - 1000, sync.unwrapTrackerMicros(range - 1000))
		assertEquals(2 * range + 500, sync.unwrapTrackerMicros(500))
	}

	@Test
	fun smallBackwardStepIsNotTreatedAsWrap() {
		// Packet reordering or jitter can make a timestamp go slightly
		// backwards. That must not be mistaken for a wrap, which would add 71
		// minutes to every subsequent reading.
		val sync = ClockSync()
		assertEquals(1_000_000L, sync.unwrapTrackerMicros(1_000_000))
		assertEquals(999_000L, sync.unwrapTrackerMicros(999_000))
		assertEquals(1_001_000L, sync.unwrapTrackerMicros(1_001_000))
	}

	@Test
	fun convertsTrackerTimeToServerTime() {
		val sync = ClockSync()
		val trueOffset = 7_000_000L
		var t = 0L
		repeat(10) {
			exchange(sync, t, trueOffset)
			t += 500_000
		}

		// A sample the tracker stamped at trackerTime happened at
		// trackerTime - offset in server time.
		val trackerStamp = 12_000_000L
		assertEquals(trackerStamp - trueOffset, sync.toServerMicros(trackerStamp))
	}

	@Test
	fun passesThroughBeforeAnyEstimate() {
		// Callers should not have to special-case startup.
		val sync = ClockSync()
		assertFalse(sync.hasEstimate)
		assertEquals(123_456L, sync.toServerMicros(123_456))
	}

	@Test
	fun rejectsImpossibleExchanges() {
		val sync = ClockSync()
		// Tracker claims to have turned the packet around for longer than the
		// entire round trip took.
		assertFalse(sync.addExchange(0, 1000, 90_000, 10_000))
		assertEquals(1, sync.rejectedSamples)
		assertFalse(sync.hasEstimate)
	}

	@Test
	fun twoTrackersDisagreeWithoutSyncAndAgreeWithIt() {
		// The point of the whole exercise: two trackers with different clock
		// offsets stamp the same instant with very different numbers, and
		// converting through their own estimates brings them back together.
		val a = ClockSync()
		val b = ClockSync()
		val offsetA = 3_000_000L
		val offsetB = -8_500_000L

		var t = 0L
		repeat(20) {
			exchange(a, t, offsetA)
			exchange(b, t, offsetB)
			t += 500_000
		}

		val instantServer = 25_000_000L
		val stampA = instantServer + offsetA
		val stampB = instantServer + offsetB

		val rawSkew = abs(stampA - stampB)
		assertTrue(rawSkew > 11_000_000, "raw stamps should differ hugely, got $rawSkew")

		val correctedSkew = abs(a.toServerMicros(stampA) - b.toServerMicros(stampB))
		assertTrue(
			correctedSkew < 100,
			"after conversion the two should agree, differed by $correctedSkew us",
		)
	}
}
