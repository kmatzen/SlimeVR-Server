package dev.slimevr.tracking.processor.skeleton

import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Projects a corrected leg back onto the set of poses the skeleton can actually
 * hold.
 *
 * ## The problem this exists for
 *
 * `LegTweaks` corrects floor penetration and foot skating by translating joint
 * positions: `correctClipping()` moves the ankle and the knee by one
 * displacement and the hip by a different one, and `correctSkating()` sets a
 * locked ankle's horizontal position directly. Each of those is the right
 * *intent* -- the foot really was planted, the foot really was below the floor
 * -- expressed in the only notation available at that point in the pipeline,
 * which is position.
 *
 * Moving one end of a segment without the other changes the segment's length.
 * Measured on synthetic motion, the corrections deform the leg by up to 0.16 m,
 * 11% of a segment's length, on a squat. The output is not the forward
 * kinematic pose plus an error; it is not a pose the skeleton could hold at all.
 *
 * ## What this does instead
 *
 * Takes the same targets the corrections already computed and answers a
 * different question: *what is the closest pose the leg can actually reach that
 * puts the ankle there?*
 *
 * That is two-link inverse kinematics, and it has a closed form. Given the hip,
 * the ankle target and the two segment lengths, the knee lies on a circle -- the
 * intersection of two spheres -- and the only remaining freedom is where on that
 * circle, which is the knee's swing plane. That freedom is resolved by staying
 * as close as possible to where the knee already was, so the correction changes
 * the leg as little as it can.
 *
 * Segment lengths are preserved *structurally* rather than by being careful:
 * the output is constructed from the lengths, so it cannot violate them.
 *
 * ## What it cannot do, and says so
 *
 * When the ankle target is further from the hip than the leg is long, no pose
 * reaches it. The existing correction has no way to notice -- it moves the foot
 * and the leg stretches. This clamps to the reachable boundary instead, which
 * moves the ankle away from the target by exactly the amount the leg was being
 * asked to stretch.
 *
 * That residual is worth having rather than hiding. It is the correction
 * admitting the pose and the contact constraint disagree, which is information
 * a joint estimator would use -- issue #4's point is that a single objective can
 * trade the two off, where a chain of stages can only let the later one win.
 * Here the later one still wins, but the disagreement stops being silent.
 *
 * ## Scope
 *
 * This is not the sliding-window estimator issue #4 proposes. It is one
 * constraint -- fixed segment lengths -- applied to one stage, chosen because it
 * is the constraint the measurements show is being violated and because it can be
 * enforced in closed form with no optimiser and no per-frame allocation. It is a
 * step toward that proposal and a test of its central claim, not a substitute
 * for it.
 */
object JointSpaceProjection {

	/** Both segments of a projected leg, guaranteed consistent with their lengths. */
	data class Leg(val knee: Vector3, val ankle: Vector3)

	private const val EPSILON = 1e-6f

	/**
	 * @param hip root of the leg chain, taken as given
	 * @param ankleTarget where the corrections want the ankle
	 * @param kneeHint where the knee currently is, used only to pick the swing
	 *   plane among the poses that reach the target equally well
	 * @param upperLength hip-to-knee distance to preserve
	 * @param lowerLength knee-to-ankle distance to preserve
	 */
	fun project(
		hip: Vector3,
		ankleTarget: Vector3,
		kneeHint: Vector3,
		upperLength: Float,
		lowerLength: Float,
	): Leg {
		if (upperLength <= EPSILON || lowerLength <= EPSILON) {
			return Leg(kneeHint, ankleTarget)
		}

		val toTarget = ankleTarget - hip
		val distance = toTarget.len()

		// A target at the hip itself gives no direction to build the leg along.
		// Rare but not impossible with a bad correction, and the arithmetic below
		// divides by this.
		if (distance < EPSILON) {
			return Leg(kneeHint, ankleTarget)
		}

		val axis = toTarget / distance

		// Reachable range of a two-link chain: fully extended at the top, folded
		// back on itself at the bottom. Outside it, no configuration of the knee
		// puts the ankle at the target, so the ankle moves rather than the leg
		// stretching.
		val maxReach = upperLength + lowerLength
		val minReach = abs(upperLength - lowerLength)
		val reach = distance.coerceIn(minReach + EPSILON, maxReach - EPSILON)
		val ankle = hip + axis * reach

		// Distance along the hip-to-ankle line at which the knee circle sits, from
		// the two sphere equations. `height` is that circle's radius.
		val along = (upperLength * upperLength - lowerLength * lowerLength + reach * reach) / (2f * reach)
		val height = sqrt(maxOf(0f, upperLength * upperLength - along * along))

		val knee = hip + axis * along + kneeDirection(hip, axis, kneeHint) * height
		return Leg(knee, ankle)
	}

	/**
	 * Which way the knee bends: the component of the current knee offset
	 * perpendicular to the hip-to-ankle line.
	 *
	 * When the leg is straight the current knee is on the line, that component is
	 * zero, and every swing plane is equally good. Any perpendicular will do in
	 * that case, and one is constructed from whichever world axis is least
	 * aligned with the leg so the cross product is well conditioned.
	 */
	private fun kneeDirection(hip: Vector3, axis: Vector3, kneeHint: Vector3): Vector3 {
		val offset = kneeHint - hip
		val perpendicular = offset - axis * offset.dot(axis)
		if (perpendicular.len() > EPSILON) return perpendicular.unit()

		val fallback = if (abs(axis.y) < 0.9f) Vector3(0f, 1f, 0f) else Vector3(1f, 0f, 0f)
		val cross = axis.cross(fallback).cross(axis)
		return if (cross.len() > EPSILON) cross.unit() else Vector3(0f, 0f, 1f)
	}
}
