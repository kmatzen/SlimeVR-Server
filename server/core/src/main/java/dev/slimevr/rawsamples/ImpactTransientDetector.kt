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
	val refractoryMicros: Long = 150_000,
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

	fun detect(log: ImuLogReader): Result = detect(log.accel, log.accScale)

	fun detect(samples: ImuLogReader.Samples, scale: Float): Result {
		if (samples.size < 3) {
			return Result(emptyList(), 0.0, 0.0, samples.size, 0.0)
		}

		// |a| per sample, then first difference. Only consecutive samples are
		// differenced: a pair either side of a marked gap is not adjacent in
		// time, and treating it as such would manufacture a huge jerk exactly
		// where the recording admits it has no data.
		val jerk = DoubleArray(samples.size - 1)
		for (i in 1 until samples.size) {
			val adjacent = samples.micros[i] - samples.micros[i - 1] <= MAX_ADJACENT_MICROS
			jerk[i - 1] = if (adjacent) {
				abs(samples.magnitude(i, scale) - samples.magnitude(i - 1, scale))
			} else {
				0.0
			}
		}

		val mad = medianAbsoluteDeviation(jerk)
		if (mad <= 0.0) {
			return Result(emptyList(), 0.0, 0.0, samples.size, samples.durationSeconds)
		}

		val threshold = mad * thresholdMads
		val impacts = mutableListOf<Impact>()
		var i = 0
		var peak = 0.0
		while (i < jerk.size) {
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
				if (jerk[j] / mad > peak) peak = jerk[j] / mad
				j++
			}
			impacts.add(Impact(samples.micros[best + 1], jerk[best] / mad))
			i = j
		}

		return Result(
			impacts = impacts,
			noiseMad = mad,
			peakToNoise = peak,
			sampleCount = samples.size,
			durationSeconds = samples.durationSeconds,
		)
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
	}
}
