package dev.slimevr.tracking.processor

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Estimates the relative heading between two trackers from the joint that
 * connects them, without a magnetometer.
 *
 * A 6-DoF tracker has no absolute heading reference, so each one's yaw drifts
 * independently. It is the *relative* yaw between trackers that corrupts the
 * pose, and that quantity turns out to be observable from the kinematics alone:
 * two segments joined by a hinge cannot have arbitrary relative heading,
 * because both segments must agree about where the hinge axis points in the
 * world.
 *
 * Writing `a` for that axis expressed in the world frame from each side:
 *
 * ```
 *   a1 = R1 * j1        j1 = hinge axis in segment 1's local frame
 *   a2 = R2 * j2        j2 = hinge axis in segment 2's local frame
 * ```
 *
 * If both orientations were correct these would be the same vector. A heading
 * error is a rotation about world vertical -- Y in SlimeVR's frame -- so the
 * discrepancy is exactly a rotation of the horizontal (x-z) components, and the
 * angle between them is the relative heading error.
 *
 * ## Observability
 *
 * This is the part that matters in practice, and the part a naive
 * implementation gets wrong.
 *
 * A rotation about vertical does not move a vertical vector. So when the hinge
 * axis is near vertical its horizontal component is tiny, the angle between the
 * two horizontal projections is dominated by noise, and the estimate is
 * meaningless -- not merely imprecise. A knee whose axis happens to point
 * skyward tells you nothing about heading no matter how long you watch it.
 *
 * Each sample is therefore weighted by how horizontal the axis is, and the
 * estimator reports its own confidence so a caller can decline to act. Knowing
 * when to do nothing is the main advantage over a heuristic that always nudges
 * something.
 *
 * Nothing consumes this yet; it is intended to be evaluated against Stay
 * Aligned on recorded sessions before replacing anything.
 */
class KinematicHeading {

	private var sumSin = 0.0
	private var sumCos = 0.0
	private var sumWeight = 0.0
	private var samples = 0

	/** Samples folded into the estimate, including poorly observable ones. */
	val sampleCount: Int
		get() = samples

	/**
	 * Mean observability across the samples seen, in 0..1.
	 *
	 * This is how horizontal the joint axis has been. Near zero means the
	 * geometry never constrained heading, regardless of how much data was fed
	 * in.
	 */
	val observability: Double
		get() = if (samples == 0) 0.0 else sumWeight / samples

	/**
	 * Agreement between individual estimates, in 0..1.
	 *
	 * The circular resultant length. High means the samples concentrate on one
	 * answer; low means they disagree, which happens when the hinge assumption
	 * does not hold -- a joint being treated as a hinge that is really moving
	 * as a ball joint will produce a confident-looking mean with a low
	 * concentration behind it.
	 */
	val concentration: Double
		get() {
			if (sumWeight <= 0.0) return 0.0
			return sqrt(sumSin * sumSin + sumCos * sumCos) / sumWeight
		}

	/** True when the estimate is worth acting on. */
	val hasEstimate: Boolean
		get() = samples >= MIN_SAMPLES &&
			observability >= MIN_OBSERVABILITY &&
			concentration >= MIN_CONCENTRATION

	/**
	 * Relative heading of segment 2 with respect to segment 1, radians.
	 *
	 * Applying a rotation of this angle about world vertical to segment 2's
	 * orientation brings the two into agreement about the joint axis. Zero
	 * until there is any data.
	 */
	val relativeHeadingRad: Double
		get() = if (samples == 0) 0.0 else atan2(sumSin, sumCos)

	/**
	 * Folds in one observation.
	 *
	 * [axis1] and [axis2] are the hinge axis expressed in each segment's own
	 * local frame; they are properties of the skeleton, not of the motion.
	 */
	fun addSample(
		rotation1: Quaternion,
		axis1: Vector3,
		rotation2: Quaternion,
		axis2: Vector3,
	) {
		val a1 = rotation1.sandwich(axis1)
		val a2 = rotation2.sandwich(axis2)

		// SlimeVR's world frame is Y-up, so heading is rotation about Y and the
		// horizontal plane is x-z. Getting this wrong does not fail loudly: it
		// produces a plausible number from the wrong two components.
		val h1 = hypot(a1.x.toDouble(), a1.z.toDouble())
		val h2 = hypot(a2.x.toDouble(), a2.z.toDouble())

		// Weighted by the *less* horizontal of the two: a heading estimate is
		// only as good as the worse-conditioned side of the comparison.
		val weight = minOf(h1, h2)

		samples++
		sumWeight += weight
		if (weight <= 0.0) {
			return
		}

		// A rotation about Y by d maps the in-plane angle atan2(z, x) to
		// (angle - d), so the correction that brings segment 2 onto segment 1
		// is the difference taken in this order.
		val delta = atan2(a2.z.toDouble(), a2.x.toDouble()) -
			atan2(a1.z.toDouble(), a1.x.toDouble())

		// Accumulated as a vector so the mean is circular. A plain arithmetic
		// mean of angles is wrong across the +/-pi wrap, and silently so.
		sumSin += weight * sin(delta)
		sumCos += weight * cos(delta)
	}

	fun reset() {
		sumSin = 0.0
		sumCos = 0.0
		sumWeight = 0.0
		samples = 0
	}

	companion object {
		/** Below this the mean is not meaningfully averaged. */
		const val MIN_SAMPLES = 20

		/**
		 * Mean horizontal component below which the geometry never really
		 * constrained heading. sin(15 deg); below that a degree of axis noise
		 * becomes several degrees of heading error.
		 */
		const val MIN_OBSERVABILITY = 0.25

		/** Circular concentration below which the samples simply disagree. */
		const val MIN_CONCENTRATION = 0.6
	}
}
