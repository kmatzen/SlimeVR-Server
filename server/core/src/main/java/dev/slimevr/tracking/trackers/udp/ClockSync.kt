package dev.slimevr.tracking.trackers.udp

/**
 * Estimates a tracker's clock offset and rate relative to the server's.
 *
 * Trackers timestamp their own samples, but every tracker's clock starts at an
 * arbitrary moment and runs at a slightly different rate. With one tracker that
 * is harmless -- everything is late by the same amount and it reads as input
 * lag. With six to sixteen on independent links it is a correctness problem,
 * because the skeleton is then solved from observations taken at *different*
 * instants.
 *
 * This is the standard NTP four-timestamp exchange:
 *
 * ```
 *   serverTx ----------> trackerRx
 *                        trackerTx
 *   serverRx <----------
 * ```
 *
 * offset = ((trackerRx - serverTx) + (trackerTx - serverRx)) / 2
 * delay  =  (serverRx - serverTx) - (trackerTx - trackerRx)
 *
 * The offset estimate is only as good as the assumption that the outbound and
 * return paths took the same time, so a sample taken during a network hiccup is
 * badly biased. Filtering on minimum observed delay is what makes this usable
 * over WiFi, and is the main reason to prefer this over simply differencing
 * arrival times.
 *
 * All times are microseconds.
 */
class ClockSync(
	/**
	 * Samples retained for the rate fit. At one exchange every 500 ms this is
	 * about a minute of history -- long enough to see a few tens of ppm of drift
	 * above the noise, short enough to follow a real rate change.
	 */
	private val windowSize: Int = 120,
) {
	/** One accepted exchange. */
	private data class Sample(
		/** Server time at the midpoint of the exchange. */
		val serverMicros: Long,
		/** trackerClock - serverClock at that moment. */
		val offsetMicros: Long,
		/** Round-trip delay, excluding the tracker's own turnaround. */
		val delayMicros: Long,
	)

	private val samples = ArrayDeque<Sample>()

	/**
	 * Tracker's 32-bit microsecond clock wraps roughly every 71.6 minutes, so
	 * raw tracker timestamps have to be unwrapped into a monotonic value before
	 * any arithmetic. Failing to do this produces a working system that breaks
	 * about an hour after it is switched on, which is a uniquely unpleasant bug
	 * to hunt.
	 */
	private var trackerEpochs = 0L
	private var lastRawTrackerMicros = 0L
	private var haveTrackerReference = false

	/** Best current estimate of trackerClock - serverClock, microseconds. */
	var offsetMicros: Long = 0
		private set

	/**
	 * Tracker clock rate error relative to the server, in parts per million.
	 * Positive means the tracker's clock runs fast.
	 */
	var skewPpm: Double = 0.0
		private set

	/** Lowest round-trip delay observed, microseconds. Negative if none yet. */
	var bestDelayMicros: Long = -1
		private set

	/** Exchanges accepted into the estimate. */
	var acceptedSamples: Int = 0
		private set

	/** Exchanges rejected as too delayed to be informative. */
	var rejectedSamples: Int = 0
		private set

	val hasEstimate: Boolean
		get() = samples.isNotEmpty()

	/**
	 * Unwraps a raw 32-bit tracker timestamp into a monotonically increasing
	 * value. Must be called in the order the timestamps were produced.
	 */
	fun unwrapTrackerMicros(raw: Long): Long {
		val masked = raw and 0xFFFFFFFFL
		if (!haveTrackerReference) {
			haveTrackerReference = true
			lastRawTrackerMicros = masked
			return masked
		}
		// A large backwards step is a wrap, not time travel. The threshold is
		// half the range, which is the usual choice: it is correct as long as
		// consecutive readings are less than ~36 minutes apart.
		if (masked < lastRawTrackerMicros &&
			lastRawTrackerMicros - masked > WRAP_HALF
		) {
			trackerEpochs++
		}
		lastRawTrackerMicros = masked
		return masked + trackerEpochs * WRAP_RANGE
	}

	/**
	 * Records one completed exchange. Returns true if it was accepted.
	 *
	 * [trackerRxMicros] and [trackerTxMicros] must already be unwrapped via
	 * [unwrapTrackerMicros].
	 */
	fun addExchange(
		serverTxMicros: Long,
		trackerRxMicros: Long,
		trackerTxMicros: Long,
		serverRxMicros: Long,
	): Boolean {
		val roundTrip = serverRxMicros - serverTxMicros
		val turnaround = trackerTxMicros - trackerRxMicros
		val delay = roundTrip - turnaround

		// A negative delay is impossible and means one of the clocks moved
		// backwards or a packet was reordered; a wildly large one means the
		// exchange was not round-trip symmetric. Neither is worth keeping.
		if (delay < 0 || roundTrip < 0) {
			rejectedSamples++
			return false
		}

		if (bestDelayMicros < 0 || delay < bestDelayMicros) {
			bestDelayMicros = delay
		} else if (delay > bestDelayMicros * DELAY_REJECT_FACTOR + DELAY_REJECT_FLOOR) {
			// Minimum-delay filtering. The offset error introduced by an
			// asymmetric path is bounded by half the excess delay, so a sample
			// several times worse than the best seen carries little information
			// and a lot of bias.
			rejectedSamples++
			return false
		}

		val offset = (
			(trackerRxMicros - serverTxMicros) +
				(trackerTxMicros - serverRxMicros)
			) /
			2
		val serverMid = serverTxMicros + roundTrip / 2

		samples.addLast(Sample(serverMid, offset, delay))
		while (samples.size > windowSize) {
			samples.removeFirst()
		}
		acceptedSamples++

		recompute()
		return true
	}

	/**
	 * Converts a tracker timestamp to server time.
	 *
	 * [trackerMicros] must be unwrapped. Returns the input unchanged until
	 * there is an estimate, so a caller need not special-case startup.
	 */
	fun toServerMicros(trackerMicros: Long): Long {
		if (!hasEstimate) {
			return trackerMicros
		}
		return trackerMicros - offsetMicros
	}

	private fun recompute() {
		// Offset comes from the least-delayed sample in the window rather than
		// an average: averaging mixes in the asymmetry error of the worse
		// samples, whereas the best one is by construction the least biased.
		var best = samples.first()
		for (s in samples) {
			if (s.delayMicros < best.delayMicros) {
				best = s
			}
		}

		if (samples.size < MIN_SAMPLES_FOR_SKEW) {
			offsetMicros = best.offsetMicros
			skewPpm = 0.0
			return
		}

		// Rate is the slope of offset against server time. Least squares over
		// the window; the offsets are noisy but the drift is systematic, so it
		// emerges over a long enough baseline.
		val n = samples.size
		var meanT = 0.0
		var meanO = 0.0
		for (s in samples) {
			meanT += s.serverMicros.toDouble()
			meanO += s.offsetMicros.toDouble()
		}
		meanT /= n
		meanO /= n

		var num = 0.0
		var den = 0.0
		for (s in samples) {
			val dt = s.serverMicros.toDouble() - meanT
			num += dt * (s.offsetMicros.toDouble() - meanO)
			den += dt * dt
		}

		val slope = if (den > 0.0) num / den else 0.0
		skewPpm = slope * 1e6

		// Report the offset at the newest sample's time, projected along the
		// fitted rate from the least-delayed sample. This keeps the low bias of
		// the best sample while staying current.
		val newest = samples.last()
		val projected = best.offsetMicros.toDouble() +
			slope *
			(newest.serverMicros - best.serverMicros).toDouble()
		offsetMicros = projected.toLong()
	}

	companion object {
		const val WRAP_RANGE = 0x1_0000_0000L
		const val WRAP_HALF = WRAP_RANGE / 2

		/** Reject samples this many times worse than the best delay seen... */
		const val DELAY_REJECT_FACTOR = 3

		/** ...plus this floor, so a very fast link does not reject everything. */
		const val DELAY_REJECT_FLOOR = 2000L

		/** Below this, report offset only; a rate fit would be noise. */
		const val MIN_SAMPLES_FOR_SKEW = 8
	}
}
