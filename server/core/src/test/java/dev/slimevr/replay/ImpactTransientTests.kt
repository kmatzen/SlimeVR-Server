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
 * Both halves are now answered. A tracker went on a shin, and the fixtures
 * below are what came back: a stationary capture, five deliberate stamps the
 * wearer counted, and a normal walk. The synthetic cases are kept because they
 * are the only ones where the true impact time is known to the sample.
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

	// --- the real answer, from a tracker on a shin ---------------------------

	private fun fixture(name: String): ImuLogReader = ImuLogReader.read(
		java.io.File(javaClass.getResource("/rawsamples/$name")!!.toURI()),
	)

	/**
	 * The cheap kill, and it did not kill it.
	 *
	 * Five deliberate heel stamps, counted by the wearer, trimmed from a
	 * ten-stamp capture where the detector found ten. If a hard stamp through a
	 * boot and a tibia had not reached a mid-shin tracker, issue #41 would have
	 * ended here -- and issue #6 would have lost its flight-time ground truth at
	 * the same moment.
	 *
	 * It reached it enormously. Against the walking-motion scale the weakest of
	 * the ten peaked at 826 MAD and the strongest at 2395, on a threshold of 20.
	 */
	@Test
	@DisplayName("finds every deliberate heel stamp, on a known count")
	fun `stamps are found`() {
		val result = ImpactTransientDetector().detect(fixture("shin-stamp-lsm6dsv.imu"))
		println("stamps: ${result.report()}")

		// A stamp is a whole movement -- lift, strike, settle -- so it produces
		// more than one transient. What must be true is that every stamp is
		// represented and none is invented, which a count within one of five is.
		val strong = result.impacts.filter { it.prominence > 100 }
		println("  strong transients: ${strong.size}, prominence ${strong.map { it.prominence.toInt() }}")
		assertTrue(
			strong.size in 4..8,
			"five counted stamps produced ${strong.size} strong transients: ${result.report()}",
		)
	}

	/**
	 * The question issue #41 actually turns on.
	 *
	 * A normal heel strike is far softer than a stamp and arrives buried in the
	 * leg's own swing, which is the real competition: standing still on a shin
	 * gives a jerk MAD of 0.0044 m/s², walking gives 0.216 -- **forty-nine times
	 * larger**. Sensor noise is not the floor here; the wearer is.
	 *
	 * Fourteen seconds of ordinary walking in place. What makes the answer
	 * credible without a camera is not the count on its own but its *regularity*:
	 * detections land about 1.26 s apart with little spread, which is one leg at
	 * a normal cadence, and each is a single transient rather than a burst.
	 */
	@Test
	@DisplayName("finds heel strikes in a normal walk, at a gait-like cadence")
	fun `walking strikes are found at gait cadence`() {
		val log = fixture("shin-walk-lsm6dsv.imu")
		val result = ImpactTransientDetector().detect(log)
		println("walk: ${result.report()}")

		val intervals = result.intervalsMillis()
		val median = intervals.sorted()[intervals.size / 2]
		println("  intervals ms: median=%.0f min=%.0f max=%.0f".format(median, intervals.min(), intervals.max()))

		// One leg strikes once per stride. Anything near half of this would mean
		// two events per stride were being counted -- plausibly strike plus
		// toe-off, which would be a different and also interesting result.
		assertTrue(
			result.impactsPerSecond in 0.5..1.3,
			"detected ${result.impactsPerSecond}/s, which is not one leg at a walking cadence",
		)
		assertTrue(
			median in 900.0..1600.0,
			"median interval $median ms is not a stride",
		)
	}

	/**
	 * And the same detector, unchanged, must still find nothing when the wearer
	 * is standing still.
	 *
	 * This is the pairing that makes the walking result mean something. A
	 * detector tuned until it produced a gait-like number would also fire on a
	 * still leg; one that does neither has separated the impact from the motion
	 * rather than from the noise.
	 */
	@Test
	@DisplayName("the same settings that find gait find nothing at rest")
	fun `gait settings stay silent at rest`() {
		val result = ImpactTransientDetector().detect(stationary())
		assertEquals(0, result.impacts.size, "fired at rest with the settings that detect gait")
	}

	// --- both edges of the contact interval ----------------------------------

	/**
	 * Toe-off is recoverable too, and the gait duty cycle is the proof.
	 *
	 * Issue #41 named this as the risk it could not resolve: touchdown is a
	 * shock and toe-off is not, so the method might date one edge of the contact
	 * interval and never the other.
	 *
	 * It dates both. What makes that believable without a camera is not the
	 * count but the *shape*: stance is about 60% of a human gait cycle and swing
	 * about 40%, and nothing in this detector knows that. If it were firing on
	 * something other than a strike and a release it would have no reason to
	 * reproduce the ratio.
	 */
	@Test
	@DisplayName("pairs each strike with a toe-off, at a human stance fraction")
	fun `contact intervals have a gait duty cycle`() {
		val log = fixture("shin-walk-lsm6dsv.imu")
		val contacts = ImpactTransientDetector().detectContacts(log)
		val paired = contacts.filter { it.toeOffMicros != null }

		println("contacts: ${contacts.size}, paired: ${paired.size}")
		val stances = paired.mapNotNull { it.stanceMicros }.map { it / 1000.0 }
		val strides = contacts.zipWithNext { a, b -> (b.strikeMicros - a.strikeMicros) / 1000.0 }
		val stance = stances.sorted()[stances.size / 2]
		val stride = strides.sorted()[strides.size / 2]
		println("  median stance %.0f ms, median stride %.0f ms, duty %.2f".format(stance, stride, stance / stride))

		// The final strike has no stride after it to bound the search, so it can
		// never pair; anything much worse than that means releases are being
		// missed rather than legitimately absent.
		assertTrue(
			paired.size >= contacts.size - 2,
			"only ${paired.size} of ${contacts.size} strikes found a release",
		)
		// Human walking stance fraction is about 0.6. Anything near 0.5 or 0.75
		// would mean the second event is not toe-off.
		// Textbook overground walking is about 0.60. Walking in place runs a
		// little higher, because the foot is not carried forward -- 0.64 here.
		assertTrue(
			stance / stride in 0.50..0.75,
			"stance fraction ${stance / stride} is not a walking duty cycle",
		)
	}

	/**
	 * And the lower threshold that finds a release must not reach into a
	 * stationary capture.
	 *
	 * This is the reason releases are searched for only *after* a strike. The
	 * loudest sample in a still recording reaches about 11 times its own local
	 * scale, which is well inside the range a toe-off occupies -- so a detector
	 * that simply lowered its threshold would report footfalls for anyone
	 * standing still. Nothing precedes a release at rest, so nothing is looked
	 * for.
	 */
	@Test
	@DisplayName("no contacts at all in a stationary capture")
	fun `no contacts at rest`() {
		val contacts = ImpactTransientDetector().detectContacts(stationary())
		assertEquals(0, contacts.size, "reported ${contacts.size} contacts on a device at rest")
	}
}
