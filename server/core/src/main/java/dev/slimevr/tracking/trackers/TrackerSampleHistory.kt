package dev.slimevr.tracking.trackers

import io.github.axisangles.ktmath.Quaternion

/**
 * A short history of one tracker's timestamped raw rotations.
 *
 * Trackers that run firmware advertising `PROTOCOL_SAMPLE_TIMESTAMPS` report
 * the instant each sample was taken, which `ClockSync` converts into the
 * server's timebase. Keeping the last few of those makes it possible to ask
 * what the tracker was reading at an *arbitrary* instant rather than only at
 * whatever moment its most recent packet happened to describe, which is what
 * [TimeAlignment] needs to bring every tracker onto a common tick.
 *
 * The buffer is deliberately small. Its only job is to bracket a reference time
 * that sits a few milliseconds in the past; retaining more would not improve
 * the interpolation and would invite the temptation to align to a stale
 * reference. See [DEFAULT_CAPACITY].
 *
 * Times are microseconds in the server's timebase, so samples from different
 * trackers are directly comparable.
 *
 * Samples are recorded on the UDP receive thread and read on the server thread,
 * so the buffer is synchronised. A torn read here would not merely return a
 * stale rotation -- it could pair a timestamp with the rotation that replaced
 * it, and interpolate confidently between two things that never coexisted.
 * `QuaternionMovingAverage` locks for the same reason.
 */
class TrackerSampleHistory(private val capacity: Int = DEFAULT_CAPACITY) {

	init {
		require(capacity >= 2) { "a history that cannot hold two samples cannot interpolate" }
	}

	private val micros = LongArray(capacity)
	private val rotations = arrayOfNulls<Quaternion>(capacity)

	/** Physical slot the next sample is written to. */
	private var next = 0

	/** Retained samples, up to [capacity]. */
	@Volatile
	var size: Int = 0
		private set

	/** Time of the newest retained sample, or 0 when empty. */
	@Volatile
	var newestMicros: Long = 0
		private set

	/** Time of the oldest retained sample, or 0 when empty. */
	@Volatile
	var oldestMicros: Long = 0
		private set

	/**
	 * Samples rejected for not being newer than the newest retained one.
	 *
	 * UDP reorders, and the clock estimate is itself being refined while
	 * samples arrive, so a converted timestamp can occasionally step backwards
	 * by a few microseconds. Keeping the buffer strictly increasing in time
	 * costs one dropped sample and buys a lookup that cannot silently return
	 * nonsense. Non-zero counts are expected; a count that grows at anything
	 * like the sample rate is a broken clock estimate, not reordering.
	 */
	@Volatile
	var outOfOrderSamples: Long = 0
		private set

	/**
	 * Times the history was discarded because a sample arrived from what looks
	 * like a different epoch. See [record].
	 */
	@Volatile
	var discontinuities: Long = 0
		private set

	@Synchronized
	fun clear() {
		size = 0
		next = 0
		newestMicros = 0
		oldestMicros = 0
	}

	/** Time spanned by the retained samples; 0 with fewer than two. */
	val spanMicros: Long
		get() = if (size < 2) 0 else newestMicros - oldestMicros

	@Synchronized
	fun record(serverMicros: Long, rotation: Quaternion) {
		if (size > 0 && serverMicros <= newestMicros) {
			// A sample far enough in the past is not reordering, it is a new
			// epoch: the tracker reconnected, or its clock estimate was rebuilt
			// and every future timestamp will now land before the ones already
			// held. Without this the buffer would reject every sample from here
			// on and the tracker would silently stop taking part in alignment
			// while still looking healthy.
			if (serverMicros < newestMicros - DISCONTINUITY_MICROS) {
				discontinuities++
				size = 0
				next = 0
			} else {
				outOfOrderSamples++
				return
			}
		}

		micros[next] = serverMicros
		rotations[next] = rotation
		next = (next + 1) % capacity
		if (size < capacity) size++

		newestMicros = serverMicros
		oldestMicros = micros[physical(0)]
	}

	/**
	 * The tracker's rotation at [serverMicros], interpolated between the two
	 * samples that bracket it.
	 *
	 * Returns null only when there is no history at all. A reference outside
	 * the retained range is *clamped* to the nearest sample rather than
	 * extrapolated: extrapolating a rotation past the last thing the tracker
	 * actually reported invents motion, and inventing motion is the failure
	 * this whole mechanism exists to remove. Clamping to the newest sample is
	 * exactly the behaviour the server had before time alignment existed, so
	 * the worst case degrades to the status quo instead of to something new.
	 */
	@Synchronized
	fun rotationAt(serverMicros: Long): Quaternion? {
		if (size == 0) return null
		if (serverMicros >= newestMicros) return rotations[physical(size - 1)]
		if (serverMicros <= oldestMicros) return rotations[physical(0)]

		// Walk back from the newest sample to the first one at or after the
		// reference. The buffer holds a few tens of entries at most, so this is
		// cheaper than the arithmetic to bisect a ring.
		var hi = size - 1
		while (hi > 0 && micros[physical(hi - 1)] > serverMicros) hi--
		val lo = hi - 1

		val t0 = micros[physical(lo)]
		val t1 = micros[physical(hi)]
		val span = t1 - t0
		if (span <= 0L) return rotations[physical(hi)]

		val t = ((serverMicros - t0).toDouble() / span.toDouble())
			.toFloat()
			.coerceIn(0f, 1f)
		return rotations[physical(lo)]!!.interpR(rotations[physical(hi)]!!, t)
	}

	/** Physical slot of the [i]th oldest retained sample. */
	private fun physical(i: Int): Int = (((next - size) % capacity + capacity) % capacity + i) % capacity

	companion object {
		/**
		 * At the 100-200 Hz trackers report at, this is 150-300 ms of history --
		 * comfortably more than [TimeAlignment.DEFAULT_MAX_SKEW_MICROS], so the
		 * reference is bracketed by real samples whenever alignment is willing
		 * to use it at all.
		 */
		const val DEFAULT_CAPACITY = 32

		/**
		 * A sample this much older than the newest retained one is treated as
		 * a new epoch rather than as reordering. One second is far beyond any
		 * plausible network reordering and far below the intervals at which a
		 * reconnect or a clock-estimate rebuild moves timestamps.
		 */
		const val DISCONTINUITY_MICROS = 1_000_000L
	}
}
