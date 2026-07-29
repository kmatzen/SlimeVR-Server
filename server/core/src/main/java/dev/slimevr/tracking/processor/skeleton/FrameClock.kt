package dev.slimevr.tracking.processor.skeleton

/**
 * Source of frame timestamps for the leg-correction pipeline.
 *
 * [LegTweaksBuffer] derives foot velocities and accelerations from the interval
 * between consecutive frames. Reading that interval from the system clock makes
 * the result depend on how fast the host happened to run, which is fine live --
 * frames really do arrive in real time -- and fatal under replay, where the same
 * input must produce the same output or no baseline computed from it means
 * anything.
 *
 * Production keeps [SYSTEM]. Replay substitutes a clock advanced by the
 * sequence's own timestep, which makes the leg corrections reproducible and lets
 * their metrics be gated rather than merely reported.
 *
 * Only *differences* between values are meaningful; there is no defined epoch.
 */
fun interface FrameClock {
	/** Monotonically non-decreasing time, in nanoseconds. */
	fun nanos(): Long

	companion object {
		/** Wall-clock time. The production default. */
		val SYSTEM = FrameClock { System.nanoTime() }
	}
}
