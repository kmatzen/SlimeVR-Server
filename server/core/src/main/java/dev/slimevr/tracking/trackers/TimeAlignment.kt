package dev.slimevr.tracking.trackers

/**
 * Brings every tracker onto a common instant before the skeleton is solved.
 *
 * The server receives each tracker's samples over its own WiFi link, and until
 * now solved the skeleton from whatever each tracker most recently said. Those
 * statements describe *different instants*: with six to sixteen trackers the
 * spread is milliseconds and it varies frame to frame. During fast motion the
 * limbs genuinely disagree about what time it is, and the solver has no way to
 * tell that apart from the body actually being in that shape. Smoothing the
 * result is the wrong fix and some of it has probably already been applied.
 *
 * The two halves this needs both already exist: `ClockSync` estimates each
 * tracker's clock offset and rate, and `PACKET_ROTATION_DATA_TIMESTAMPED`
 * carries the sample instant. This is the step that spends them --
 * [TrackerSampleHistory] retains the recent samples and this class picks the
 * instant to resolve them all to.
 *
 * ## Choosing the reference
 *
 * The reference is the *oldest* of the trackers' newest samples. Every
 * participant therefore has a sample at or after it and the correction is an
 * interpolation, never an extrapolation. The alternative -- align to the
 * newest -- would need every other tracker's rotation predicted forward past
 * anything it has reported, which manufactures exactly the motion this is
 * supposed to stop manufacturing.
 *
 * The cost is latency: the pose is solved for an instant as old as the
 * laggiest tracker's newest sample. That is bounded by [maxSkewMicros], beyond
 * which a tracker is treated as a straggler rather than as a vote on what time
 * it is -- the reference stops following it, and it gets clamped to its own
 * newest sample, which is the behaviour it had before any of this existed. A
 * tracker that has dropped off entirely must not be able to drag the whole
 * skeleton into the past.
 *
 * ## When this does nothing
 *
 * Alignment needs at least two trackers reporting sample timestamps, which
 * needs firmware advertising `PROTOCOL_SAMPLE_TIMESTAMPS`. Anything else --
 * older firmware, a mixed set with only one new tracker, non-IMU sources --
 * leaves every rotation untouched. That is why this is on with no setting
 * attached: on an installation that cannot feed it, it is not a behaviour
 * change at all, and on one that can, arrival-order fusion has no defenders.
 */
class TimeAlignment(
	/**
	 * How far into the past the reference is allowed to be dragged by the
	 * laggiest tracker.
	 */
	private val maxSkewMicros: Long = DEFAULT_MAX_SKEW_MICROS,
) {

	/** Trackers that reported a sample timestamp on the last pass. */
	var participants: Int = 0
		private set

	/** Of those, how many were resolved by interpolation rather than clamped. */
	var interpolated: Int = 0
		private set

	/**
	 * Spread between the newest and oldest of the participants' newest samples,
	 * on the last pass. This is the quantity the whole issue is about: it is
	 * how far apart in time the observations being fused actually were.
	 */
	var spreadMicros: Long = 0
		private set

	/** Instant the last pass resolved to, in server microseconds. */
	var referenceMicros: Long = 0
		private set

	/**
	 * Passes on which at least one participant's newest sample was older than
	 * the reference, so it was clamped instead of interpolated. Expected to be
	 * zero in steady state; sustained counts mean a tracker is lagging by more
	 * than [maxSkewMicros] or its clock estimate is wrong.
	 */
	var stragglerPasses: Long = 0
		private set

	/**
	 * Resolves every eligible tracker in [trackers] to a common instant.
	 *
	 * Safe to call unconditionally: with fewer than two eligible trackers it
	 * returns having touched nothing.
	 */
	fun align(trackers: List<Tracker>) {
		var oldestNewest = Long.MAX_VALUE
		var newestNewest = Long.MIN_VALUE
		var eligible = 0

		for (tracker in trackers) {
			if (!isEligible(tracker)) continue
			eligible++
			val newest = tracker.sampleHistory.newestMicros
			if (newest < oldestNewest) oldestNewest = newest
			if (newest > newestNewest) newestNewest = newest
		}

		participants = eligible
		if (eligible < 2) {
			interpolated = 0
			spreadMicros = 0
			referenceMicros = 0
			return
		}

		spreadMicros = newestNewest - oldestNewest
		val reference = if (spreadMicros > maxSkewMicros) {
			newestNewest - maxSkewMicros
		} else {
			oldestNewest
		}
		referenceMicros = reference

		var resolved = 0
		var straggler = false
		for (tracker in trackers) {
			if (!isEligible(tracker)) continue
			// A tracker whose newest sample predates the reference cannot be
			// interpolated to it, only held at its last known value. Counting
			// that separately keeps "aligned" from quietly meaning "clamped".
			if (tracker.sampleHistory.newestMicros < reference) straggler = true else resolved++
			tracker.applyTimeAlignment(reference)
		}
		interpolated = resolved
		if (straggler) stragglerPasses++
	}

	/**
	 * A tracker takes part when it has timestamped history and is sending data.
	 *
	 * The status check matters for the reference rather than for the tracker
	 * itself: a timed-out tracker keeps its last sample time forever, and
	 * without this it would pin the reference to the moment it disappeared.
	 * [maxSkewMicros] would eventually catch that anyway, but only after the
	 * whole pose had spent that long being solved for the wrong instant.
	 */
	private fun isEligible(tracker: Tracker): Boolean = tracker.status.sendData && tracker.sampleHistory.newestMicros > 0

	companion object {
		/**
		 * 50 ms. Well past the few milliseconds of genuine WiFi jitter this is
		 * correcting, and short enough that a tracker which has actually
		 * dropped out cannot hold the pose in the past for long enough to be
		 * felt.
		 */
		const val DEFAULT_MAX_SKEW_MICROS = 50_000L
	}
}
