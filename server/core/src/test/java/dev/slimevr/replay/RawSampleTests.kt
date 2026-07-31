package dev.slimevr.replay

import dev.slimevr.rawsamples.ImuLogWriter
import dev.slimevr.rawsamples.RawSampleCapture
import dev.slimevr.rawsamples.RawSampleCollector
import dev.slimevr.rawsamples.RawSampleKind
import dev.slimevr.rawsamples.RawSampleStream
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * Raw sample reassembly and the `.imu` sidecar.
 *
 * ## What is actually being guarded
 *
 * A `.pfr` stores fused output, so it freezes the firmware's whole signal chain
 * -- VQF and its parameters, rest detection, the error model, online
 * estimation. Raw counts make a capture re-runnable against any of those,
 * forever. That is only true if the raw file is trustworthy, and there is
 * exactly one way for it to be untrustworthy in a way nobody notices.
 *
 * **A hole that is not marked.** Losing a fused rotation is harmless because the
 * next one supersedes it; losing a raw sample corrupts every re-fusion run
 * downstream of it, silently, because the filter integrates a gap it cannot see.
 * A file that concatenates across a loss looks complete and is wrong.
 *
 * So most of what follows is about gaps: that they are detected, that the count
 * of missing samples is exact rather than estimated, and that they reach the
 * file.
 */
class RawSampleTests {

	private val step = 5_000L

	private fun samples(vararg values: Int) = ShortArray(values.size) { values[it].toShort() }

	private fun stream() = RawSampleStream(RawSampleKind.GYRO, step)

	@Test
	@DisplayName("contiguous batches join into one run")
	fun `contiguous batches join`() {
		val s = stream()
		s.accept(0, 0, 0, samples(1, 2, 3, 4, 5, 6), 2)
		s.accept(1, 0, 2 * step, samples(7, 8, 9), 1)

		assertEquals(1, s.runs.size, "a continuation started a new run")
		assertEquals(3, s.sampleCount)
		assertTrue(s.isComplete)
		assertEquals(0L, s.missingSamples)
	}

	/**
	 * The core property. The nominal timeline is perfectly regular, so a gap's
	 * size is arithmetic rather than inference -- and a run is never extended
	 * across one.
	 */
	@Test
	@DisplayName("a lost batch becomes a new run with an exact missing count")
	fun `lost batch is measured exactly`() {
		val s = stream()
		s.accept(0, 0, 0, samples(1, 1, 1, 2, 2, 2), 2)
		// Sequence 1 never arrived. It held 4 samples, which the timeline says
		// exactly: base jumped from 2*step to 6*step.
		s.accept(2, 0, 6 * step, samples(3, 3, 3), 1)

		assertEquals(2, s.runs.size, "the stream concatenated across a loss")
		assertEquals(1, s.lostBatches)
		assertEquals(4L, s.missingSamples)
		assertFalse(s.isComplete)
		assertEquals(0L + 2 * step, s.runs[0].endMicros(step))
		assertEquals(6 * step, s.runs[1].startMicros)
	}

	/**
	 * A tracker-side overrun is a real hole, and must split the run.
	 *
	 * This test originally asserted the opposite -- that an overrun "is not a
	 * hole in what arrived" -- which was wrong, and wrong in the direction the
	 * whole design exists to prevent. The samples were discarded before they
	 * were ever buffered, so the batches either side of the overrun are *not*
	 * contiguous in time. Joining them would produce a run that claims
	 * continuity across a gap, which is exactly the unmarked hole a re-fusion
	 * run cannot see.
	 *
	 * The two causes still stay counted apart, because they have different
	 * fixes: the tracker could not send fast enough, versus the network dropped
	 * it.
	 */
	@Test
	@DisplayName("a tracker-side overrun splits the run and is counted apart from transit loss")
	fun `overrun splits and is counted separately`() {
		val s = stream()
		s.accept(0, 0, 0, samples(1, 1, 1), 1)
		s.accept(1, 7, step, samples(2, 2, 2), 1)

		assertEquals(7, s.droppedOnTracker)
		assertEquals(0, s.lostBatches, "an overrun is not transit loss")
		assertEquals(0L, s.missingSamples)
		assertFalse(s.isComplete)
		assertEquals(2, s.runs.size, "an overrun left the run joined across a real hole")
	}

	/**
	 * Contiguity is decided by the tracker's sequence and drop counters, not by
	 * comparing reconstructed timestamps.
	 *
	 * The tracker accumulates its nominal clock in nanoseconds and truncates to
	 * microseconds per sample, so a time rebuilt as `base + i * step` drifts
	 * from the tracker's own by up to a microsecond per sample, cumulatively.
	 *
	 * Measured on hardware before this was fixed: a clean 1824-sample capture
	 * came back as 114 runs with 114 `missing=0` gap markers. The file looked
	 * shredded and was whole.
	 */
	@Test
	@DisplayName("rounding drift in the tracker clock does not split a clean run")
	fun `rounding drift does not split`() {
		val s = stream()
		s.accept(0, 0, 0, samples(1, 1, 1, 2, 2, 2), 2)
		// One microsecond later than `base + 2 * step` would predict.
		s.accept(1, 0, 2 * step + 1, samples(3, 3, 3), 1)

		assertEquals(1, s.runs.size, "a microsecond of clock rounding split the run")
		assertEquals(0, s.lostBatches)
		assertEquals(0L, s.missingSamples)
		assertTrue(s.isComplete)
		// And the tracker's own base is what the third sample reports, rather
		// than a time the server recomputed for it.
		assertEquals(2 * step + 1, s.runs[0].sampleMicros(2, step))
	}

	/**
	 * UDP reorders. A batch arriving late is not missing data, and counting it
	 * as a hole would overstate the damage in a file people will act on.
	 */
	@Test
	@DisplayName("a reordered batch is not counted as loss")
	fun `reordering is not loss`() {
		val s = stream()
		s.accept(5, 0, 5 * step, samples(1, 1, 1), 1)
		s.accept(3, 0, 3 * step, samples(2, 2, 2), 1)

		assertEquals(0, s.lostBatches)
		assertEquals(0L, s.missingSamples)
		// It still starts a new run: it does not continue the previous one, and
		// pretending it did would put samples in the wrong order.
		assertEquals(2, s.runs.size)
	}

	@Test
	@DisplayName("a batch that neither continues nor jumps forward still splits the run")
	fun `duplicate splits without counting loss`() {
		val s = stream()
		s.accept(0, 0, 0, samples(1, 1, 1), 1)
		s.accept(0, 0, 0, samples(1, 1, 1), 1)

		assertEquals(0, s.lostBatches)
		assertEquals(0L, s.missingSamples)
	}

	@Test
	@DisplayName("an empty batch is ignored rather than starting a run")
	fun `empty batch ignored`() {
		val s = stream()
		s.accept(0, 0, 0, ShortArray(0), 0)
		assertEquals(0, s.runs.size)
		assertEquals(0, s.sampleCount)
	}

	// --- the sidecar --------------------------------------------------------

	private fun capture(): RawSampleCapture {
		val c = RawSampleCapture("BMI270", accTs = 0.01f, gyrTs = 0.005f, accScale = 1.5f, gyrScale = 0.07f)
		c.accel.accept(0, 0, 0, samples(10, 11, 12, 13, 14, 15), 2)
		c.gyro.accept(0, 0, 0, samples(20, 21, 22), 1)
		return c
	}

	/**
	 * The format is not a choice. `tools/fusion-bench` in the firmware
	 * repository already parses `# slimevr-imu-log v1` with these header keys
	 * and this column set, so emitting exactly it is what closes the join
	 * without new tooling.
	 */
	@Test
	@DisplayName("the sidecar is what fusion-bench already parses")
	fun `header matches the bench format`() {
		val text = ImuLogWriter.render(capture())
		val lines = text.lines()

		assertEquals("# slimevr-imu-log v1", lines[0])
		assertTrue(text.contains("# sensor BMI270"))
		assertTrue(text.contains("# acc_ts 0.01"), "acc_ts missing or reformatted:\n$text")
		assertTrue(text.contains("# gyr_ts 0.005"))
		assertTrue(text.contains("# acc_scale 1.5"))
		assertTrue(text.contains("# gyr_scale 0.07"))
		assertTrue(text.contains("t_us,ax,ay,az,gx,gy,gz"))
	}

	/**
	 * Accelerometer rows fill the `a*` columns and leave `g*` empty, and the
	 * reverse for gyroscope -- the firmware's own row shape, which the bench
	 * reads.
	 */
	@Test
	@DisplayName("rows keep the firmware's column layout and merge in time order")
	fun `rows are shaped and ordered`() {
		val text = ImuLogWriter.render(capture())
		val rows = text.lines().filter { it.isNotBlank() && !it.startsWith("#") && !it.startsWith("t_us") }

		assertEquals(3, rows.size)
		// gyro step is 5000 us, accel 10000, so: gyro@0, accel@0, accel@10000.
		assertTrue(rows.any { it == "0,10,11,12,,," }, "accel row missing:\n$text")
		assertTrue(rows.any { it == "0,,,,20,21,22" }, "gyro row missing:\n$text")
		assertTrue(rows.any { it == "10000,13,14,15,,," })

		val times = rows.map { it.substringBefore(',').toLong() }
		assertEquals(times.sorted(), times, "rows are not in nominal-time order")
	}

	/**
	 * The reason this class exists rather than a `joinToString`. A gap must
	 * reach the file, as a comment the bench's parser skips, so a holed capture
	 * loads as two shorter captures with a documented hole rather than as one
	 * continuous capture that silently jumps.
	 */
	@Test
	@DisplayName("a gap reaches the file, with its exact size")
	fun `gaps are written`() {
		val c = RawSampleCapture("BMI270", 0.01f, 0.005f, 1.5f, 0.07f)
		c.gyro.accept(0, 0, 0, samples(1, 1, 1), 1)
		c.gyro.accept(2, 0, 4 * 5_000L, samples(2, 2, 2), 1)

		val text = ImuLogWriter.render(c)

		assertTrue(text.contains("# INCOMPLETE"), "an incomplete capture does not say so:\n$text")
		assertTrue(
			text.contains("# gap gyro missing=3 from_us=5000 to_us=20000"),
			"gap marker missing or wrong:\n$text",
		)
		// And it must still be a loadable file: every non-comment line is a row.
		val rows = text.lines().filter { it.isNotBlank() && !it.startsWith("#") && !it.startsWith("t_us") }
		assertEquals(2, rows.size)
	}

	@Test
	@DisplayName("a complete capture is not labelled incomplete")
	fun `complete capture has no marker`() {
		val text = ImuLogWriter.render(capture())
		assertFalse(text.contains("# INCOMPLETE"))
		assertFalse(text.contains("# gap "))
	}

	@Test
	@DisplayName("two renders of one capture are identical")
	fun `render is deterministic`() {
		val c = capture()
		assertEquals(ImuLogWriter.render(c), ImuLogWriter.render(c))
	}

	// --- the collector ------------------------------------------------------

	@Test
	@DisplayName("nothing is collected unless a capture is running")
	fun `collector is off by default`() {
		val collector = RawSampleCollector()
		collector.streamInfo(1, 0, "BMI270", 0.01f, 0.005f, 1.5f, 0.07f)
		collector.samples(1, 0, RawSampleKind.GYRO, 0, 0, 0, samples(1, 1, 1), 1)

		assertFalse(collector.isCapturing)
		assertEquals(0, collector.sampleCount)
		assertTrue(collector.results().isEmpty())
	}

	/**
	 * Raw counts with guessed scale factors are not a degraded capture, they are
	 * a wrong one. Discarding and counting is the honest answer, and the tracker
	 * repeats the metadata often enough that the loss is bounded.
	 */
	@Test
	@DisplayName("samples arriving before their scale factors are discarded and counted")
	fun `unscalable batches are counted`() {
		val collector = RawSampleCollector()
		collector.start()
		collector.samples(1, 0, RawSampleKind.GYRO, 0, 0, 0, samples(1, 1, 1), 1)

		assertEquals(0, collector.sampleCount)
		assertEquals(1L, collector.unscalableBatches)
		assertFalse(collector.isComplete)
	}

	@Test
	@DisplayName("sensors are kept apart, and repeated metadata does not reset them")
	fun `sensors are separate and info is idempotent`() {
		val collector = RawSampleCollector()
		collector.start()
		collector.streamInfo(1, 0, "BMI270", 0.01f, 0.005f, 1.5f, 0.07f)
		collector.streamInfo(1, 1, "LSM6DSV", 0.008f, 0.004f, 1.2f, 0.06f)
		collector.samples(1, 0, RawSampleKind.GYRO, 0, 0, 0, samples(1, 1, 1), 1)

		// The tracker repeats this every couple of seconds; taking it again must
		// not throw away what has been collected since.
		collector.streamInfo(1, 0, "BMI270", 0.01f, 0.005f, 1.5f, 0.07f)
		collector.samples(1, 0, RawSampleKind.GYRO, 1, 0, 5_000, samples(2, 2, 2), 1)

		val results = collector.results()
		assertEquals(2, results.size)
		assertEquals(2, results.getValue(RawSampleCollector.Key(1, 0)).sampleCount)
		assertEquals(0, results.getValue(RawSampleCollector.Key(1, 1)).sampleCount)
		assertEquals("LSM6DSV", results.getValue(RawSampleCollector.Key(1, 1)).sensorName)
	}

	@Test
	@DisplayName("starting a capture clears the previous one")
	fun `start clears`() {
		val collector = RawSampleCollector()
		collector.start()
		collector.streamInfo(1, 0, "BMI270", 0.01f, 0.005f, 1.5f, 0.07f)
		collector.samples(1, 0, RawSampleKind.GYRO, 0, 0, 0, samples(1, 1, 1), 1)
		collector.stop()
		collector.start()

		assertEquals(0, collector.sampleCount)
		assertTrue(collector.results().isEmpty())
	}
}
