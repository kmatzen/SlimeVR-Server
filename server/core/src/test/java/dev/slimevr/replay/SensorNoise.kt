package dev.slimevr.replay

import io.github.axisangles.ktmath.Quaternion
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Perturbs synthetic tracker rotations the way a real tracker's would be wrong.
 *
 * ## Why this exists
 *
 * Every measurement in this suite is taken on motion where each method's core
 * assumption holds exactly. Issue #5's conclusion carries the caveat explicitly:
 *
 * > a bare stillness radius has no defence against noise -- noise inflates
 * > apparent displacement, which is exactly what this rule thresholds on. The
 * > ordering could reverse on real recordings.
 *
 * That caveat names a specific mechanism, and a named mechanism can be probed
 * without waiting for hardware. Issue #7's head-to-head established the
 * principle and the limit of it: *"robustness on real data" is also robustness
 * to something nameable, and three of the four things it means are modelable:
 * white sensor noise, constant per-tracker mounting error, yaw drift.* The
 * fourth -- a real body not being this kinematic chain -- is not, and stays with
 * #15.
 *
 * So this models the three, and the questions it can settle are the ones that
 * turn on them.
 *
 * ## What this is not
 *
 * **It is not a substitute for a recording, and it is not calibrated to any real
 * tracker.** Nothing here says what σ a BMI270 actually delivers. What a sweep
 * over σ gives is the shape of the answer -- whether a method degrades gently or
 * falls over, and where the crossover between two methods sits -- and that is
 * worth having before a corpus exists, because it says what to look for in one.
 *
 * Two further limits worth stating, because they bound the claim:
 *
 * - **`.pfr` stores fused output, not raw IMU.** What reaches the skeleton has
 *   already been through the tracker's fusion filter, so real orientation error
 *   is smoother and more correlated than the white component here.
 *   [whiteNoiseDeg] is therefore pessimistic for a given magnitude, which is the
 *   right direction for a robustness probe and the wrong one for a realism
 *   claim. [smoothingFrames] exists to model the correlated case.
 * - **Noise is injected at the tracker, not at the sensor.** Fusion is not
 *   re-run, so this cannot show a fusion filter recovering from a disturbance.
 *
 * ## Reproducibility
 *
 * Bit-reproducible across machines and independent of call order, which the rest
 * of the suite depends on (#14, #16). Every draw is keyed by
 * `(seed, tracker, frame)` through [java.util.Random], whose generator is
 * specified exactly rather than left to the platform -- so replaying the same
 * configuration twice, on any JVM, perturbs every frame identically.
 */
class SensorNoise(
	/**
	 * Standard deviation of the per-frame, per-tracker orientation error, in
	 * degrees.
	 *
	 * Applied in the sensor's own frame, since that is where the error arises.
	 * This is the component contact detection by stillness is most exposed to:
	 * it is uncorrelated frame to frame, so it turns directly into apparent
	 * displacement of the computed foot.
	 */
	val whiteNoiseDeg: Float = 0f,
	/**
	 * Fixed per-tracker rotation between the sensor and the segment it is
	 * strapped to, in degrees. Drawn once per tracker and then constant.
	 *
	 * A useful control: it biases the pose substantially while adding no jitter
	 * at all, so a method that only degrades under [whiteNoiseDeg] is being hurt
	 * by variance rather than by being wrong about where the body is.
	 */
	val mountingErrorDeg: Float = 0f,
	/**
	 * Per-tracker heading drift, in degrees per second, drawn once per tracker
	 * in `[-yawDriftDegPerSec, +yawDriftDegPerSec]`.
	 *
	 * Applied in the world frame, matching [YawDriftReplayTest]'s injection, and
	 * the one component of IMU error that simulates faithfully.
	 */
	val yawDriftDegPerSec: Float = 0f,
	/**
	 * Exponential smoothing applied to the white component, in frames.
	 *
	 * Zero leaves it white. Above zero it becomes a correlated wander of the
	 * same standard deviation, which is closer to what survives a fusion filter
	 * -- and a much gentler thing to threshold a stillness radius against, since
	 * neighbouring frames move together.
	 */
	val smoothingFrames: Int = 0,
	val seed: Long = 20260730L,
) {
	val isNoiseless: Boolean
		get() = whiteNoiseDeg == 0f && mountingErrorDeg == 0f && yawDriftDegPerSec == 0f

	/** Smoothed white component, carried per tracker across frames. */
	private val smoothed = HashMap<Int, FloatArray>()

	/**
	 * Reset before each replay, so two replays of one configuration agree.
	 *
	 * Only [smoothingFrames] carries state; everything else is a pure function
	 * of `(seed, tracker, frame)`.
	 */
	fun reset() = smoothed.clear()

	/**
	 * The rotation a tracker on this segment would have reported.
	 *
	 * @param tracker stable index identifying the tracker, so its mounting error
	 *   and drift rate are the same on every frame and across replays.
	 * @param frame frame number, keying the white component.
	 * @param seconds elapsed time, over which the drift accumulates.
	 */
	fun perturb(tracker: Int, frame: Int, seconds: Float, rotation: Quaternion): Quaternion {
		if (isNoiseless) return rotation

		var out = rotation

		// Mounting error: a fixed rotation between sensor and segment, so it
		// composes on the sensor side.
		if (mountingErrorDeg != 0f) {
			out *= fixedRotation(tracker, MOUNTING_KEY, mountingErrorDeg)
		}

		// White (or smoothed) orientation error, also in the sensor frame.
		if (whiteNoiseDeg != 0f) {
			out *= whiteRotation(tracker, frame)
		}

		// Heading drift is an error about the world's vertical, not the
		// sensor's, so it premultiplies.
		if (yawDriftDegPerSec != 0f) {
			val rate = uniform(tracker, DRIFT_KEY) * yawDriftDegPerSec
			out = yawBy(rate * seconds) * out
		}

		return out
	}

	/**
	 * A small rotation whose axis is uniform on the sphere and whose angle is
	 * Gaussian, optionally smoothed across frames.
	 *
	 * The smoothed form is an exponential moving average of the underlying
	 * Gaussian draws, rescaled so its standard deviation still equals
	 * [whiteNoiseDeg] -- otherwise raising [smoothingFrames] would quietly
	 * reduce the noise as well as correlating it, and the two effects could not
	 * be told apart.
	 */
	private fun whiteRotation(tracker: Int, frame: Int): Quaternion {
		val random = java.util.Random(mix(seed, tracker.toLong(), frame.toLong()))
		val raw = floatArrayOf(
			random.nextGaussian().toFloat(),
			random.nextGaussian().toFloat(),
			random.nextGaussian().toFloat(),
		)

		val sample = if (smoothingFrames <= 0) {
			raw
		} else {
			val alpha = 1f / smoothingFrames
			val state = smoothed.getOrPut(tracker) { FloatArray(3) }
			// Rescaling factor for an EMA of iid unit-variance draws, whose
			// stationary variance is alpha / (2 - alpha).
			val gain = sqrt((2f - alpha) / alpha)
			for (i in 0..2) state[i] = state[i] * (1f - alpha) + raw[i] * alpha
			floatArrayOf(state[0] * gain, state[1] * gain, state[2] * gain)
		}

		// The three components become a rotation vector: direction is the axis,
		// magnitude the angle. Isotropic, and no special case at zero.
		val magnitude = sqrt(sample[0] * sample[0] + sample[1] * sample[1] + sample[2] * sample[2])
		if (magnitude < 1e-9f) return Quaternion.IDENTITY
		val angleDeg = whiteNoiseDeg * magnitude / SPHERE_NORM
		return axisAngle(sample[0] / magnitude, sample[1] / magnitude, sample[2] / magnitude, angleDeg)
	}

	/** A fixed rotation for this tracker: same axis and angle on every frame. */
	private fun fixedRotation(tracker: Int, key: Long, magnitudeDeg: Float): Quaternion {
		val random = java.util.Random(mix(seed, tracker.toLong(), key))
		var x = random.nextGaussian().toFloat()
		var y = random.nextGaussian().toFloat()
		var z = random.nextGaussian().toFloat()
		val magnitude = sqrt(x * x + y * y + z * z)
		if (magnitude < 1e-9f) return Quaternion.IDENTITY
		x /= magnitude
		y /= magnitude
		z /= magnitude
		return axisAngle(x, y, z, magnitudeDeg * uniform(tracker, key + 1))
	}

	/** Deterministic value in `[-1, 1]` for this tracker and key. */
	private fun uniform(tracker: Int, key: Long): Float = java.util.Random(mix(seed, tracker.toLong(), key))
		.nextFloat() *
		2f -
		1f

	private fun axisAngle(x: Float, y: Float, z: Float, deg: Float): Quaternion {
		val half = deg * (Math.PI.toFloat() / 180f) / 2f
		val s = sin(half)
		return Quaternion(cos(half), x * s, y * s, z * s)
	}

	private fun yawBy(deg: Float): Quaternion {
		val half = deg * (Math.PI.toFloat() / 180f) / 2f
		return Quaternion(cos(half), 0f, sin(half), 0f)
	}

	/** One-line description, for the report tables. */
	fun label(): String = when {
		isNoiseless -> "noiseless"

		else -> buildString {
			if (whiteNoiseDeg != 0f) {
				append("σ=%.2f°".format(whiteNoiseDeg))
				if (smoothingFrames > 0) append(" (smoothed ${smoothingFrames}fr)")
			}
			if (mountingErrorDeg != 0f) {
				if (isNotEmpty()) append(", ")
				append("mount=%.1f°".format(mountingErrorDeg))
			}
			if (yawDriftDegPerSec != 0f) {
				if (isNotEmpty()) append(", ")
				append("drift=%.2f°/s".format(yawDriftDegPerSec))
			}
		}
	}

	companion object {
		private const val MOUNTING_KEY = -1L
		private const val DRIFT_KEY = -3L

		/**
		 * Mean magnitude of a 3-vector of standard normals, `2·√(2/π)`.
		 *
		 * Dividing by it makes [whiteNoiseDeg] the *typical rotation angle*
		 * rather than the per-component standard deviation, so the parameter
		 * means the thing the tables report it as.
		 */
		private val SPHERE_NORM = 2f * sqrt(2f / Math.PI.toFloat())

		/**
		 * SplitMix64 finalisation over the three keys.
		 *
		 * [java.util.Random] seeds itself with a scramble that leaves nearby
		 * seeds producing correlated first draws, and consecutive frame numbers
		 * are exactly nearby seeds. Mixing first removes that.
		 */
		private fun mix(seed: Long, a: Long, b: Long): Long {
			var z = seed * -7046029254386353131L + a * -4658895280553007687L + b * -7723592293110705685L
			z = (z xor (z ushr 30)) * -4658895280553007687L
			z = (z xor (z ushr 27)) * -7723592293110705685L
			return z xor (z ushr 31)
		}
	}
}
