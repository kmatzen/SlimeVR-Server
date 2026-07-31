package dev.slimevr.rawsamples

import kotlin.math.abs

/**
 * Finds heel-strike impacts in a raw accelerometer stream.
 *
 * ## What this is for
 *
 * Issue #5's metric is contact *timing*, and measuring it absolutely needs
 * something that independently knows when the foot landed. There will be no
 * lighthouse and no pressure mat, so the candidate is the impact itself: a heel
 * strike is a shock, and a shank-mounted accelerometer is where that shock is
 * normally measured.
 *
 * It only became possible when `.imu` sidecars started carrying raw samples.
 * **Fused output smooths a transient away** -- rejecting them is what a fusion
 * filter is for -- so the sharpest edge in the signal is exactly what the `.pfr`
 * discards and the `.imu` keeps.
 *
 * Issue #41 is the experiment. This is the instrument, and it is deliberately
 * separable from the question: whether a real heel strike through a real leg
 * produces a usable edge is answered by [Result.peakToNoise] on a real capture,
 * not by anything decided here.
 *
 * ## Why jerk rather than acceleration
 *
 * Acceleration on a walking shin is dominated by gravity and by the swing
 * itself, both of which are large and slow. An impact is neither: it is small in
 * total energy and very fast. Differencing suppresses everything smooth in
 * proportion to how smooth it is, which turns "sharpest" into "largest" and
 * leaves a threshold that means something.
 *
 * Magnitude is differenced rather than each axis, so the answer does not depend
 * on which way the tracker is mounted.
 *
 * ## Why the threshold is not a constant
 *
 * It is a multiple of the capture's own median absolute deviation. A fixed
 * number in m/s² would be a constant tuned against whatever recording happened
 * to be available -- the practice issue #5 objects to -- and would not survive a
 * different mounting, a different floor, or a different wearer.
 *
 * MAD rather than standard deviation because the impacts themselves are in the
 * data being measured, and they would inflate a standard deviation enough to
 * hide the smaller ones. A median is indifferent to them.
 */
class ImpactTransientDetector(
	/**
	 * Multiple of the noise MAD above which a jerk sample is a candidate.
	 *
	 * Set from a real device's floor rather than from a noise model. Ten seconds
	 * of a stationary LSM6DSV reaches **9.9 MAD** -- its jerk distribution has
	 * tails roughly four times heavier than a Gaussian of the same MAD, so a
	 * threshold chosen against simulated noise fires on real hardware. It did:
	 * at 8 MAD the still capture produced five phantom impacts.
	 *
	 * Twenty leaves about 2x margin over the measured floor while staying far
	 * below anything an impact produces, and
	 * `ImpactTransientTests` pins both ends of that against the committed
	 * stationary fixture.
	 */
	val thresholdMads: Double = 20.0,
	/**
	 * Minimum spacing between reported impacts.
	 *
	 * A foot cannot strike twice in this window, and an impact rings for several
	 * samples. 150 ms is comfortably below the fastest plausible step and
	 * comfortably above the ring.
	 */
	val refractoryMicros: Long = 350_000,
	/**
	 * Width of the window the local scale is estimated over.
	 *
	 * Long enough to contain several strides, so a single impact cannot inflate
	 * the scale that is meant to detect it, and short enough to track the
	 * difference between standing and walking rather than averaging across it.
	 */
	val scaleWindowMicros: Long = 2_000_000,
) {
	data class Impact(
		val micros: Long,
		/** Peak jerk at the impact, in MADs. The signal-to-noise this is all about. */
		val prominence: Double,
	)

	data class Result(
		val impacts: List<Impact>,
		/** Median absolute deviation of the jerk signal, in m/s^2 per sample. */
		val noiseMad: Double,
		/** Largest jerk seen anywhere, in MADs. */
		val peakToNoise: Double,
		val sampleCount: Int,
		val durationSeconds: Double,
	) {
		val impactsPerSecond: Double
			get() = if (durationSeconds <= 0.0) 0.0 else impacts.size / durationSeconds

		/** Intervals between consecutive impacts, in milliseconds. */
		fun intervalsMillis(): List<Double> = impacts
			.zipWithNext { a, b -> (b.micros - a.micros) / 1000.0 }

		fun report(): String = buildString {
			append("%d impacts in %.1f s (%.2f/s)".format(impacts.size, durationSeconds, impactsPerSecond))
			append(", noise MAD %.5f m/s^2".format(noiseMad))
			append(", peak %.1f MAD".format(peakToNoise))
			if (impacts.isNotEmpty()) {
				append(
					", prominence %.1f-%.1f MAD".format(
						impacts.minOf { it.prominence },
						impacts.maxOf { it.prominence },
					),
				)
			}
		}
	}

	/**
	 * A contact interval: when the foot landed, and when it left.
	 *
	 * [toeOffMicros] is null when no release could be found for a strike, which
	 * is the honest answer at the end of a recording and whenever the release is
	 * too soft to see.
	 */
	data class Contact(val strikeMicros: Long, val toeOffMicros: Long?) {
		val stanceMicros: Long? get() = toeOffMicros?.minus(strikeMicros)
	}

	/**
	 * Pairs each heel strike with the toe-off that ends its stance.
	 *
	 * ## Why toe-off needs a different search
	 *
	 * A heel strike is an impact and a toe-off is a release, so the release is
	 * far weaker -- measured on a real walk, about 3.6x smaller. Simply lowering
	 * the threshold until releases appear also drops it below the loudest thing
	 * in a *stationary* capture, which peaks around 11 times its own local scale.
	 * A detector that finds toe-off that way invents footfalls in anyone standing
	 * still.
	 *
	 * So the release is only ever looked for **after a strike has been found**,
	 * inside the window where gait says it must be. Nothing is searched for at
	 * rest because nothing precedes it there, which is what makes a lower
	 * threshold safe.
	 *
	 * ## Why the result is checkable without a camera
	 *
	 * Stance is about 60% of a gait cycle and swing about 40%, so the two
	 * intervals should alternate at roughly 1.5:1. On the committed walking
	 * fixture they come out at 782 ms and 499 ms -- a ratio of 1.57, summing to
	 * 1281 ms against a stride measured independently at 1256 ms.
	 *
	 * That agreement is the evidence. A detector firing on something other than
	 * strike and release would have no reason to reproduce the duty cycle of
	 * human gait.
	 */
	fun detectContacts(log: ImuLogReader): List<Contact> = detectContacts(log.accel, log.accScale)

	fun detectContacts(samples: ImuLogReader.Samples, scale: Float): List<Contact> {
		val strikes = detect(samples, scale).impacts
		if (strikes.isEmpty()) return emptyList()

		val jerk = jerkOf(samples, scale)
		val localMad = localScale(jerk, samples.micros)
		val releaseThreshold = thresholdMads / RELEASE_THRESHOLD_DIVISOR

		return strikes.mapIndexed { index, strike ->
			val nextStrike = strikes.getOrNull(index + 1)?.micros ?: Long.MAX_VALUE
			// Stance cannot begin instantly -- the impact itself rings -- and
			// cannot outlast the stride it belongs to.
			val from = strike.micros + refractoryMicros
			val until = minOf(nextStrike - MIN_SWING_MICROS, strike.micros + MAX_STANCE_MICROS)

			var best = -1
			var bestValue = 0.0
			for (i in jerk.indices) {
				val t = samples.micros[i + 1]
				if (t < from) continue
				if (t > until) break
				val mad = localMad[i]
				if (mad <= 0.0) continue
				val ratio = jerk[i] / mad
				if (ratio >= releaseThreshold && ratio > bestValue) {
					bestValue = ratio
					best = i
				}
			}
			Contact(strike.micros, if (best >= 0) samples.micros[best + 1] else null)
		}
	}

	fun detect(log: ImuLogReader): Result = detect(log.accel, log.accScale)

	/**
	 * |a| per sample, first-differenced.
	 *
	 * Only consecutive samples are differenced: a pair either side of a marked
	 * gap is not adjacent in time, and treating it as such would manufacture a
	 * huge jerk exactly where the recording admits it has no data.
	 */
	private fun jerkOf(samples: ImuLogReader.Samples, scale: Float): DoubleArray {
		val jerk = DoubleArray(samples.size - 1)
		for (i in 1 until samples.size) {
			val adjacent = samples.micros[i] - samples.micros[i - 1] <= MAX_ADJACENT_MICROS
			jerk[i - 1] = if (adjacent) {
				abs(samples.magnitude(i, scale) - samples.magnitude(i - 1, scale))
			} else {
				0.0
			}
		}
		return jerk
	}

	fun detect(samples: ImuLogReader.Samples, scale: Float): Result {
		if (samples.size < 3) {
			return Result(emptyList(), 0.0, 0.0, samples.size, 0.0)
		}

		val jerk = jerkOf(samples, scale)

		val globalMad = medianAbsoluteDeviation(jerk)
		if (globalMad <= 0.0) {
			return Result(emptyList(), 0.0, 0.0, samples.size, samples.durationSeconds)
		}
		val localMad = localScale(jerk, samples.micros)

		val impacts = mutableListOf<Impact>()
		var i = 0
		var peak = 0.0
		while (i < jerk.size) {
			val mad = localMad[i]
			if (mad <= 0.0) {
				i++
				continue
			}
			val threshold = mad * thresholdMads
			if (jerk[i] / mad > peak) peak = jerk[i] / mad
			if (jerk[i] < threshold) {
				i++
				continue
			}

			// Take the largest sample in the burst rather than the first to
			// cross: the leading edge is where the threshold happens to sit,
			// the peak is where the impact is.
			var best = i
			var j = i
			while (j < jerk.size && samples.micros[j] - samples.micros[i] < refractoryMicros) {
				if (jerk[j] > jerk[best]) best = j
				val scale = localMad[j]
				if (scale > 0.0 && jerk[j] / scale > peak) peak = jerk[j] / scale
				j++
			}
			impacts.add(Impact(samples.micros[best + 1], jerk[best] / mad))
			i = j
		}

		return Result(
			impacts = impacts,
			noiseMad = globalMad,
			peakToNoise = peak,
			sampleCount = samples.size,
			durationSeconds = samples.durationSeconds,
		)
	}

	/**
	 * MAD of the jerk in a window around each sample.
	 *
	 * Evaluated on a coarse grid and held constant between grid points: the
	 * scale is a property of what the leg is doing over seconds, so computing it
	 * per sample would cost a great deal to say the same thing.
	 */
	private fun localScale(jerk: DoubleArray, micros: LongArray): DoubleArray {
		val out = DoubleArray(jerk.size)
		val half = scaleWindowMicros / 2
		var gridStart = 0
		while (gridStart < jerk.size) {
			val centre = micros[gridStart]
			var lo = gridStart
			while (lo > 0 && centre - micros[lo] < half) lo--
			var hi = gridStart
			while (hi < jerk.size - 1 && micros[hi] - centre < half) hi++

			val window = jerk.copyOfRange(lo, hi + 1)
			val mad = medianAbsoluteDeviation(window)

			var gridEnd = gridStart
			while (gridEnd < jerk.size && micros[gridEnd] - centre < scaleWindowMicros / 8) {
				out[gridEnd] = mad
				gridEnd++
			}
			gridStart = if (gridEnd > gridStart) gridEnd else gridStart + 1
		}
		return out
	}

	private fun medianAbsoluteDeviation(values: DoubleArray): Double {
		if (values.isEmpty()) return 0.0
		val median = median(values.copyOf())
		val deviations = DoubleArray(values.size) { abs(values[it] - median) }
		return median(deviations)
	}

	private fun median(values: DoubleArray): Double {
		values.sort()
		val mid = values.size / 2
		return if (values.size % 2 == 0) (values[mid - 1] + values[mid]) / 2.0 else values[mid]
	}

	companion object {
		/**
		 * Beyond this, two samples are not consecutive.
		 *
		 * Three nominal periods at the slowest configured accelerometer rate,
		 * so ordinary jitter does not trip it but a marked gap does.
		 */
		private const val MAX_ADJACENT_MICROS = 30_000L

		/**
		 * How much weaker a release may be than the strike threshold.
		 *
		 * Measured at about 3.6x on a real walk; three keeps a little margin
		 * without reaching down into the range a stationary capture occupies.
		 */
		private const val RELEASE_THRESHOLD_DIVISOR = 3.0

		/** A stride's swing phase is never shorter than this. */
		private const val MIN_SWING_MICROS = 200_000L

		/** Nor its stance longer than this, at any plausible walking cadence. */
		private const val MAX_STANCE_MICROS = 1_100_000L
	}
}
