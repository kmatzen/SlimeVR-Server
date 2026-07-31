package dev.slimevr.replay

import dev.slimevr.rawsamples.ImpactTransientDetector
import dev.slimevr.rawsamples.ImuLogReader
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * The instrument for issue #41, and what can be established about it without a
 * wearer.
 *
 * ## The question, and which half of it this answers
 *
 * Issue #5 needs to know *when* a foot landed, and there will be no lighthouse
 * and no pressure mat. The candidate is the heel-strike impact in the raw
 * accelerometer, which only became readable when `.imu` sidecars started
 * carrying raw samples.
 *
 * That question splits cleanly in two:
 *
 * 1. **Can a detector find a sharp transient in this data, and time it?**
 *    Answerable here, with synthetic impacts of known time and size.
 * 2. **Does a real heel strike, through a real leg, produce one?**
 *    Not answerable here. It needs a tracker on a shin and somebody to walk.
 *
 * Everything below is the first half plus one thing that is better than
 * synthetic: **a real stationary capture as the negative control.** A detector
 * that fires on a still device is useless no matter how well it scores on
 * generated impacts, and a real device's floor -- quantisation, mounting,
 * whatever the desk is doing -- is not something synthetic noise can stand in
 * for.
 */
class ImpactTransientTests {

	private val accScale = 0.0011964112f
	private val stepMicros = 8319L

	/**
	 * A stationary accelerometer with an impact spliced in at known times.
	 *
	 * Gravity on Z with per-sample noise, then a brief one-sample-rise
	 * transient -- which is the pessimistic shape, since a real impact rings
	 * over several samples and gives a detector more to work with.
	 */
	private fun synthetic(
		seconds: Double,
		impactSeconds: List<Double>,
		impactG: Double,
		noiseCounts: Double = 3.0,
	): ImuLogReader.Samples {
		val count = (seconds * 1e6 / stepMicros).toInt()
		val micros = LongArray(count) { it * stepMicros }
		val values = ShortArray(count * 3)
		val random = java.util.Random(41)
		// 1 g in raw counts for this scale.
		val gCounts = 9.80665 / accScale

		for (i in 0 until count) {
			values[i * 3] = (random.nextGaussian() * noiseCounts).toInt().toShort()
			values[i * 3 + 1] = (random.nextGaussian() * noiseCounts).toInt().toShort()
			values[i * 3 + 2] = (gCounts + random.nextGaussian() * noiseCounts).toInt().toShort()
		}
		for (t in impactSeconds) {
			val i = (t * 1e6 / stepMicros).toInt()
			if (i in 0 until count) {
				val spike = (gCounts * impactG).toInt()
				values[i * 3 + 2] = (values[i * 3 + 2] + spike).coerceIn(-32768, 32767).toShort()
			}
		}
		return ImuLogReader.Samples(micros, values)
	}

	private fun stationary(): ImuLogReader = ImuLogReader.read(
		java.io.File(javaClass.getResource("/rawsamples/stationary-lsm6dsv.imu")!!.toURI()),
	)

	class Spliced(val samples: ImuLogReader.Samples, val impactMicros: List<Long>)

	/**
	 * Adds a one-sample impact of [g] gravities near each of [seconds].
	 *
	 * Returns the times of the samples actually modified rather than the times
	 * requested. The real capture has marked gaps, so a requested instant can
	 * fall in one; comparing the detector against a wall-clock time it was never
	 * given would measure the splice, not the detector.
	 */
	private fun splice(
		samples: ImuLogReader.Samples,
		seconds: List<Double>,
		g: Double,
	): Spliced {
		val values = samples.values.copyOf()
		val spike = (9.80665 * g / accScale).toInt()
		val t0 = samples.micros.first()
		val actual = mutableListOf<Long>()
		for (t in seconds) {
			val target = t0 + (t * 1e6).toLong()
			val i = samples.micros.indexOfFirst { it >= target }
			if (i >= 0) {
				values[i * 3 + 2] = (values[i * 3 + 2] + spike).coerceIn(-32768, 32767).toShort()
				actual.add(samples.micros[i])
			}
		}
		return Spliced(ImuLogReader.Samples(samples.micros, values), actual)
	}

	// --- the negative control ------------------------------------------------

	/**
	 * The one that matters most, and the only one here using real data.
	 *
	 * Ten seconds of a real LSM6DSV sitting still. The detector must find
	 * nothing. If it fires here, every impact it reports on a walking capture is
	 * suspect, and no amount of synthetic performance rescues that.
	 */
	@Test
	@DisplayName("finds nothing in ten seconds of a real, stationary device")
	fun `no false positives on real stationary data`() {
		val log = stationary()
		val result = ImpactTransientDetector().detect(log)

		println("stationary: ${result.report()}")

		assertTrue(log.accel.size > 1000, "fixture is too short to mean anything")
		assertEquals(
			0,
			result.impacts.size,
			"the detector fired ${result.impacts.size} times on a device that was not moving; " +
				"peak was ${result.peakToNoise} MAD against a threshold of 20",
		)
	}

	/**
	 * And it must not be quiet by accident.
	 *
	 * A detector that finds nothing because its threshold is enormous would pass
	 * the test above and be useless. The headroom between the loudest thing in a
	 * still capture and the threshold is what says there is room for a real
	 * impact to land in.
	 */
	@Test
	@DisplayName("the still capture leaves headroom rather than scraping the threshold")
	fun `stationary headroom is real`() {
		val log = stationary()
		val detector = ImpactTransientDetector()
		val result = detector.detect(log)

		println("stationary peak: %.2f MAD (threshold %.1f)".format(result.peakToNoise, detector.thresholdMads))

		assertTrue(
			result.peakToNoise < detector.thresholdMads,
			"the loudest sample in a still capture reaches ${result.peakToNoise} MAD, at or above " +
				"the ${detector.thresholdMads} MAD threshold -- there is no room left for an impact",
		)
	}

	// --- can it find and time an impact at all -------------------------------

	@Test
	@DisplayName("finds synthetic impacts and times them to within a sample")
	fun `finds and times impacts`() {
		val times = listOf(1.0, 2.0, 3.0, 4.0, 5.0)
		val result = ImpactTransientDetector().detect(synthetic(6.0, times, impactG = 4.0), accScale)

		println("synthetic 4g: ${result.report()}")

		assertEquals(times.size, result.impacts.size, "wrong number of impacts: ${result.report()}")
		for ((expected, impact) in times.zip(result.impacts)) {
			val errorMillis = abs(impact.micros / 1000.0 - expected * 1000.0)
			// One accelerometer period is 8.3 ms; the transient is one sample
			// wide, so a correct answer is the sample itself or its neighbour.
			assertTrue(
				errorMillis <= 2 * stepMicros / 1000.0,
				"impact at ${impact.micros / 1000.0} ms should be ${expected * 1000} ms",
			)
		}
	}

	/**
	 * The number this whole issue turns on, measured against a real floor.
	 *
	 * Not "is a 4 g impact detectable" -- obviously -- but how small one can get
	 * before it disappears. That is what a real heel strike, attenuated by a
	 * boot and a tibia, has to beat.
	 *
	 * Impacts are spliced into the **real stationary capture** rather than into
	 * generated noise, because generated noise gave the wrong answer once
	 * already: its tails are far lighter than the device's, and a threshold
	 * calibrated against it fired five times on a tracker that was not moving.
	 */
	@Test
	@DisplayName("reports the smallest impact separable from a real device's floor")
	fun `sensitivity against the real floor`() {
		val log = stationary()
		val detector = ImpactTransientDetector()
		val times = listOf(1.0, 2.5, 4.0, 5.5, 7.0)
		var smallest = Double.NaN

		for (g in listOf(1.0, 0.5, 0.25, 0.1, 0.05, 0.02)) {
			val spliced = splice(log.accel, times, g)
			val result = detector.detect(spliced.samples, log.accScale)
			println("  %5.3f g -> %d/%d impacts, peak %.1f MAD".format(g, result.impacts.size, times.size, result.peakToNoise))
			if (result.impacts.size == times.size) smallest = g
		}

		println("smallest impact separable from a real floor: $smallest g")
		assertTrue(
			smallest <= 0.25,
			"the detector needs more than 0.25 g above a real device floor, which is a lot to " +
				"ask of a heel strike arriving through a boot and a tibia -- it managed $smallest g",
		)
	}

	/**
	 * Timing, also against the real floor.
	 *
	 * A detector that finds an impact but dates it wrongly is worse than useless
	 * for issue #5, whose whole metric is *when*.
	 */
	@Test
	@DisplayName("times impacts spliced into real noise to within a sample")
	fun `timing against the real floor`() {
		val log = stationary()
		val times = listOf(1.0, 2.5, 4.0, 5.5, 7.0)
		val spliced = splice(log.accel, times, 0.5)
		val result = ImpactTransientDetector().detect(spliced.samples, log.accScale)

		assertEquals(spliced.impactMicros.size, result.impacts.size, "not all impacts found: ${result.report()}")

		val errorsMillis = spliced.impactMicros.zip(result.impacts).map { (expected, impact) ->
			abs(impact.micros - expected) / 1000.0
		}
		println("timing error vs real floor: %.1f ms max".format(errorsMillis.max()))

		// One accelerometer period is 8.3 ms and the splice is one sample wide,
		// so the sample itself or its neighbour is a correct answer. Issue #5
		// names 50 ms as the damaging threshold, so this has an order of
		// magnitude in hand.
		assertTrue(
			errorsMillis.max() <= 2 * stepMicros / 1000.0,
			"worst timing error ${errorsMillis.max()} ms",
		)
	}

	// --- things that would quietly corrupt a timing measurement --------------

	/**
	 * A marked gap must not be read as an impact.
	 *
	 * Two samples either side of a hole are not adjacent in time, and
	 * differencing them manufactures a large jerk exactly where the recording
	 * admits it has no data. That would put a phantom footfall at every gap --
	 * and real captures have them.
	 */
	@Test
	@DisplayName("a gap in the samples is not an impact")
	fun `gaps do not fire`() {
		val clean = synthetic(6.0, emptyList(), 0.0)
		// Excise a second from the middle, leaving the times either side intact.
		val keep = clean.micros.indices.filter { clean.micros[it] < 2_000_000 || clean.micros[it] > 3_000_000 }
		val micros = LongArray(keep.size) { clean.micros[keep[it]] }
		val values = ShortArray(keep.size * 3)
		for ((newIndex, oldIndex) in keep.withIndex()) {
			values[newIndex * 3] = clean.values[oldIndex * 3]
			values[newIndex * 3 + 1] = clean.values[oldIndex * 3 + 1]
			values[newIndex * 3 + 2] = clean.values[oldIndex * 3 + 2]
		}

		val result = ImpactTransientDetector().detect(ImuLogReader.Samples(micros, values), accScale)
		println("gapped, no impacts: ${result.report()}")

		assertEquals(0, result.impacts.size, "a marked gap was reported as an impact")
	}

	@Test
	@DisplayName("one impact is reported once, not once per sample it rings for")
	fun `refractory collapses a ring`() {
		// A four-sample ring, as a real impact would produce.
		val ring = listOf(1.0, 1.0 + 8.319e-3, 1.0 + 16.6e-3, 1.0 + 25.0e-3)
		val result = ImpactTransientDetector().detect(synthetic(3.0, ring, impactG = 3.0), accScale)

		println("ringing impact: ${result.report()}")
		assertEquals(1, result.impacts.size, "a single ringing impact was reported ${result.impacts.size} times")
	}

	// --- the round trip ------------------------------------------------------

	@Test
	@DisplayName("the reader recovers what the writer wrote")
	fun `reader round trips the writer`() {
		val log = stationary()

		assertEquals("LSM6DSV", log.sensorName)
		assertEquals(0.008319467f, log.accTs)
		assertEquals(0.0011964112f, log.accScale)
		assertTrue(log.gyro.size > log.accel.size, "the gyroscope runs at twice the accelerometer rate")
		// Roughly 1 g at rest, which is the check that scaling is applied at all.
		val magnitude = log.accel.magnitude(0, log.accScale)
		assertTrue(
			magnitude in 9.0..10.6,
			"a stationary tracker reads $magnitude m/s^2; scale factors are not being applied correctly",
		)
	}
}
