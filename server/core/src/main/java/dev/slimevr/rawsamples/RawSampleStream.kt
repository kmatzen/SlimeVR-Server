package dev.slimevr.rawsamples

/**
 * One tracker's raw samples for one sensor axis-set, reassembled from batches.
 *
 * ## What this is for
 *
 * A `.pfr` recording stores *fused* tracker output, so it freezes VQF and its
 * parameters, rest detection, the sensor error model, online estimation and the
 * FIFO configuration. Raw counts make a capture re-runnable against any fusion
 * configuration, forever. This is the server side of
 * kmatzen/SlimeVR-Tracker-ESP#23, which sends them.
 *
 * ## Gaps are the whole problem
 *
 * Losing a fused rotation packet is harmless -- the next one supersedes it.
 * **Losing a raw sample corrupts every re-fusion run downstream of it**,
 * silently, because the filter integrates a hole it cannot see.
 *
 * So this never concatenates across a loss. Samples are held as [Run]s, each of
 * which is contiguous by construction, with the gaps between them recorded
 * explicitly. A capture whose holes are marked is still useful; a capture with
 * unmarked holes is worse than none, because it looks fine.
 *
 * ## Why a missing-sample count is exact rather than an estimate
 *
 * The firmware timestamps samples on a *nominal* timeline -- accumulated from
 * the configured sample period rather than read from a clock -- because the
 * configured period is what the on-device fusion integrates. That makes the
 * timeline perfectly regular, so the number of samples missing between two
 * batches is arithmetic rather than inference:
 *
 *     missing = (nextBase - expectedNext) / stepMicros
 *
 * A lost batch therefore costs a known number of samples at a known place, and
 * the recording can say so. That property is worth protecting: it is the reason
 * nominal timestamps were kept when the stream moved from `Serial` to the
 * network.
 *
 * ## Two independent kinds of loss
 *
 * They have different causes and different fixes, so they are counted apart:
 *
 * - [droppedOnTracker] -- the tracker's buffer overran and it discarded samples
 *   rather than overwriting, which it reports as a cumulative count. Means the
 *   tracker could not send fast enough.
 * - [lostBatches] / [missingSamples] -- batches that never arrived. Means the
 *   network dropped them.
 */
class RawSampleStream(
	val kind: RawSampleKind,
	/** Nominal microseconds between consecutive samples of this stream. */
	val stepMicros: Long,
) {
	/**
	 * A contiguous stretch of samples.
	 *
	 * Contiguous by construction rather than by assertion: a run is only ever
	 * extended by a batch that continues it exactly, and anything else starts a
	 * new one.
	 */
	class Run(val startMicros: Long) {
		private var values = ShortArray(INITIAL_CAPACITY)
		var count: Int = 0
			private set

		/**
		 * Where each batch's samples begin, and the time the tracker gave it.
		 *
		 * Sample times are anchored per batch rather than derived by multiplying
		 * a step by an index across the whole run, because the two disagree.
		 *
		 * The tracker accumulates its nominal clock in nanoseconds and truncates
		 * to microseconds per sample; the server would be multiplying a *rounded*
		 * microsecond step. At the LSM6DSV's calibrated 8319467 ns that is up to
		 * 1 us of divergence per sample, and it accumulates -- measured on
		 * hardware as 7 us after 16 samples and growing.
		 *
		 * The tracker's own base is authoritative, so it is what gets used.
		 */
		private val anchorIndex = mutableListOf<Int>()
		private val anchorMicros = mutableListOf<Long>()

		/** Samples, three components each, in x, y, z order. */
		val samples: ShortArray get() = values

		fun anchor(micros: Long) {
			anchorIndex.add(count)
			anchorMicros.add(micros)
		}

		fun add(x: Short, y: Short, z: Short) {
			if ((count + 1) * 3 > values.size) {
				values = values.copyOf(values.size * 2)
			}
			values[count * 3] = x
			values[count * 3 + 1] = y
			values[count * 3 + 2] = z
			count++
		}

		fun sampleMicros(index: Int, stepMicros: Long): Long {
			var at = anchorIndex.binarySearch(index)
			if (at < 0) at = -at - 2
			if (at < 0) return startMicros + index * stepMicros
			return anchorMicros[at] + (index - anchorIndex[at]) * stepMicros
		}

		/** Nominal time one step past the last sample -- where a continuation would begin. */
		fun endMicros(stepMicros: Long): Long = if (count == 0) {
			startMicros
		} else {
			sampleMicros(count - 1, stepMicros) + stepMicros
		}

		private companion object {
			const val INITIAL_CAPACITY = 3 * 256
		}
	}

	private val internalRuns = mutableListOf<Run>()
	val runs: List<Run> get() = internalRuns

	/** Cumulative samples the tracker discarded to buffer overrun. */
	var droppedOnTracker: Int = 0
		private set

	/** Batches that never arrived, inferred from gaps in the sequence. */
	var lostBatches: Int = 0
		private set

	/** Samples missing because of those batches, computed from the nominal timeline. */
	var missingSamples: Long = 0
		private set

	/** Batches accepted, for the report. */
	var batches: Int = 0
		private set

	val sampleCount: Int get() = internalRuns.sumOf { it.count }

	private var expectedSequence: Long = -1

	/**
	 * Accepts one batch.
	 *
	 * @param sequence the tracker's batch counter for this stream. It counts
	 *   batches *produced*, so a gap in it means data was lost on the wire.
	 * @param droppedTotal the tracker's cumulative overrun count.
	 * @param baseMicros nominal time of the batch's first sample.
	 */
	fun accept(
		sequence: Long,
		droppedTotal: Int,
		baseMicros: Long,
		values: ShortArray,
		count: Int,
	) {
		if (count <= 0) return
		batches++

		val current = internalRuns.lastOrNull()
		// Continuity is a question about *production*, not about arithmetic.
		// The tracker's sequence counts batches it produced, and its drop
		// counter says whether anything was discarded between them -- so those
		// two answer it exactly.
		//
		// Comparing timestamps instead looks equivalent and is not: the
		// tracker's nominal clock truncates nanoseconds to microseconds per
		// sample, so a reconstructed time drifts from the tracker's by up to a
		// microsecond per sample. Measured on hardware, that split a clean
		// 1824-sample capture into 114 runs and emitted 114 `missing=0` gap
		// markers -- a file that looked shredded and was not.
		val continues = current != null &&
			sequence == expectedSequence &&
			droppedTotal == droppedOnTracker

		if (!continues) {
			if (current != null) {
				// Only count loss for a forward jump. A batch that arrives out
				// of order, or repeats, is not missing data -- it is UDP being
				// UDP -- and counting it as a hole would overstate the damage.
				if (expectedSequence in 0 until sequence) {
					lostBatches += (sequence - expectedSequence).toInt()
					val expectedMicros = current.endMicros(stepMicros)
					if (baseMicros > expectedMicros && stepMicros > 0) {
						missingSamples += (baseMicros - expectedMicros) / stepMicros
					}
				}
			}
			internalRuns.add(Run(baseMicros))
		}

		if (droppedTotal > droppedOnTracker) {
			droppedOnTracker = droppedTotal
		}

		val run = internalRuns.last()
		run.anchor(baseMicros)
		for (i in 0 until count) {
			run.add(values[i * 3], values[i * 3 + 1], values[i * 3 + 2])
		}
		expectedSequence = sequence + 1
	}

	/** True when every sample the tracker produced reached us. */
	val isComplete: Boolean get() = droppedOnTracker == 0 && missingSamples == 0L && lostBatches == 0

	fun summary(): String = "%s: %d samples in %d run(s), %d batches, %d dropped on tracker, %d lost in transit (%d batches)".format(
		kind.name.lowercase(),
		sampleCount,
		internalRuns.size,
		batches,
		droppedOnTracker,
		missingSamples,
		lostBatches,
	)
}

/** Which of a sensor's two raw streams a batch belongs to. */
enum class RawSampleKind {
	ACCEL,
	GYRO,
	;

	companion object {
		fun fromId(id: Int): RawSampleKind? = entries.getOrNull(id)
	}
}
