package dev.slimevr.replay

import dev.slimevr.tracking.processor.skeleton.FrameClock

/**
 * A [FrameClock] driven by the replayed sequence rather than by the host.
 *
 * The replay loop calls [advance] once per frame, so the interval the leg
 * corrections see is exactly the sequence's timestep no matter how fast or
 * slow the machine running the test happens to be. That is what makes the
 * corrected configuration reproducible enough to gate on a baseline.
 *
 * Reads do not advance time: several buffers may be constructed within one
 * frame, and they should all be stamped with that frame's time rather than
 * with an interval that depends on how many times the pipeline happened to ask.
 */
class FixedStepClock(private val stepNanos: Long) {
	private var now = 0L

	constructor(stepSeconds: Float) : this((stepSeconds.toDouble() * 1e9).toLong())

	val clock = FrameClock { now }

	fun advance() {
		now += stepNanos
	}
}
