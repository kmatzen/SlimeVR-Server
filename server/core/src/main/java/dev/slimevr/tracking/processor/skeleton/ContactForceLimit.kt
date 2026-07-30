package dev.slimevr.tracking.processor.skeleton

import io.github.axisangles.ktmath.Vector3
import kotlin.math.sqrt

/**
 * What accelerations the centre of mass is allowed to have, given what is
 * touching the floor.
 *
 * This is issue #6's proposal (2): reject or damp translation estimates that
 * would require forces the contacts cannot supply.
 *
 * ## Why this is a hard constraint and not a heuristic
 *
 * The whole body's centre of mass obeys `m·a = Σ F_external` exactly. Swinging
 * an arm, tucking the legs, or twisting changes the body's configuration and
 * moves the CoM *within* the body, but the internal forces that do it cancel in
 * pairs and contribute nothing to the sum. So whatever the pose is doing, the
 * CoM acceleration is fixed by gravity plus whatever the floor is pushing with,
 * and nothing else -- provided the feet are the only contacts, which is the
 * assumption this rests on and the one that VR breaks (see below).
 *
 * The floor can only push, never pull, and it can only push along the friction
 * cone. Writing the required contact force per unit mass as
 *
 * ```
 * f = a − g          (g points down, so this is a + (0, 9.81, 0))
 * ```
 *
 * the two conditions are
 *
 * ```
 * f_y ≥ 0                        the floor pushes up, it does not pull down
 * √(f_x² + f_z²) ≤ μ · f_y       friction cannot exceed μ times the normal load
 * ```
 *
 * which is a second-order cone. It is worth writing it this way rather than as
 * two separate rules because the airborne case then needs no special handling:
 * with nothing touching the floor the available normal force is zero, the cone
 * collapses to the single point `f = 0`, and the constraint says the CoM
 * accelerates at exactly `g` in every axis. "No horizontal acceleration while
 * airborne" is not an extra rule, it is what the cone already says.
 *
 * ## Where the normal force comes from
 *
 * The cone needs a bound on `f_y`, and this deliberately does not read it from
 * the estimated vertical acceleration. That quantity is the thing being
 * checked; sizing the check from it would make the test vacuous. It comes from
 * the contact state instead, which is an independent measurement:
 *
 * - nothing on the floor: no support at all, `f = 0`
 * - a foot on the floor: up to [MAX_SUPPORT_G] times body weight
 *
 * [MAX_SUPPORT_G] is a bound on human capability, not a tuning knob. Vertical
 * ground reaction forces peak near 2.5x body weight in a maximal countermovement
 * jump and around 1.2x in walking, so 3x is comfortably above anything a user
 * will produce and the constraint only bites on estimates that are not merely
 * energetic but impossible.
 *
 * ## What this cannot see
 *
 * Every external force that is not the floor: leaning on a desk, a hand on a
 * wall, sitting on furniture, holding a rail. Issue #6 raises exactly this, and
 * it is why the projection here damps rather than rejects, and why it is off by
 * default. A user pushing off a wall genuinely accelerates outside the
 * foot-contact cone, and the honest reading of that is not "the estimate is
 * wrong" but "the contact model is incomplete".
 */
object ContactForceLimit {

	/** Downward acceleration, m/s^2. Shared with [LegTweaksBuffer.GRAVITY]. */
	private val GRAVITY: Vector3 = LegTweaksBuffer.GRAVITY

	/** Peak vertical ground reaction force, as a multiple of body weight. */
	const val MAX_SUPPORT_G: Float = 3.0f

	/**
	 * Coefficient of friction between shoe or sock and a domestic floor.
	 *
	 * Conservative on purpose. Rubber on wood runs higher, socks on laminate
	 * much lower; taking the high end means the constraint only fires on
	 * accelerations no floor would supply, rather than on a user who happens to
	 * have good grip.
	 */
	const val FRICTION: Float = 1.0f

	/**
	 * The contact force per unit mass an acceleration demands.
	 *
	 * `f = a − g`. Subtracting a downward gravity adds an upward term, which is
	 * why a body merely standing still requires `f = (0, 9.81, 0)` rather than
	 * zero.
	 */
	fun requiredForce(comAcceleration: Vector3): Vector3 = comAcceleration - GRAVITY

	/**
	 * Largest normal force per unit mass the current contacts can supply.
	 *
	 * @param footOnGround whether any foot is in contact.
	 */
	fun maxNormalForce(footOnGround: Boolean): Float = if (footOnGround) MAX_SUPPORT_G * -GRAVITY.y else 0f

	/**
	 * Whether [comAcceleration] is one the contacts could have produced.
	 *
	 * @param tolerance slack in m/s^2, absorbing the noise in a CoM estimate
	 * that is a second difference of eight segment positions.
	 */
	fun isPlausible(
		comAcceleration: Vector3,
		footOnGround: Boolean,
		friction: Float = FRICTION,
		tolerance: Float = 0f,
	): Boolean {
		val f = requiredForce(comAcceleration)
		val maxNormal = maxNormalForce(footOnGround)
		if (f.y < -tolerance || f.y > maxNormal + tolerance) return false
		return horizontal(f) <= friction * f.y + tolerance
	}

	/**
	 * The nearest acceleration the contacts could have produced.
	 *
	 * Euclidean projection onto the friction cone, then onto the normal-force
	 * ceiling. Nearest rather than clamped per axis: the cone couples the
	 * vertical and horizontal channels -- how much sideways force is available
	 * depends on how hard the floor is being pushed -- and clamping the axes
	 * independently would ignore that coupling and land outside the cone
	 * anyway.
	 *
	 * Returns [comAcceleration] unchanged when it is already plausible, so this
	 * is an identity on the overwhelming majority of frames.
	 */
	fun project(
		comAcceleration: Vector3,
		footOnGround: Boolean,
		friction: Float = FRICTION,
	): Vector3 {
		val f = requiredForce(comAcceleration)
		val maxNormal = maxNormalForce(footOnGround)

		val h = horizontal(f)
		var ny = f.y
		var nh = h

		if (h > friction * f.y) {
			if (friction * h + f.y <= 0f) {
				// Inside the polar cone: every point of the cone is further away
				// than its apex, so the nearest plausible force is none at all.
				ny = 0f
				nh = 0f
			} else {
				// Standard second-order cone projection. The nearest point on
				// the surface `h = μ·y` to `(h, f_y)`.
				val scale = (friction * h + f.y) / (1f + friction * friction)
				ny = scale
				nh = friction * scale
			}
		}

		if (ny > maxNormal) {
			// More support than the contacts can give. Cap it, and bring the
			// horizontal component back inside the cone the cap implies.
			ny = maxNormal
			nh = minOf(nh, friction * maxNormal)
		}

		val scaled = if (h > 0f) nh / h else 0f
		return Vector3(f.x * scaled, ny, f.z * scaled) + GRAVITY
	}

	/**
	 * Largest horizontal acceleration the contacts can produce, m/s^2.
	 *
	 * Friction times the available normal load. Zero with nothing on the floor,
	 * which is the statement that a body in free fall cannot change its
	 * horizontal velocity.
	 */
	fun maxHorizontalAcceleration(footOnGround: Boolean, friction: Float = FRICTION): Float = friction * maxNormalForce(footOnGround)

	/**
	 * [comAcceleration] with only its horizontal component constrained.
	 *
	 * This, rather than [project], is what `Localizer` applies, and the reason
	 * is specific rather than cautious. The full cone couples the two channels:
	 * how much sideways force is available depends on how hard the floor is
	 * being pushed. That coupling is only meaningful if the vertical
	 * acceleration being fed in is a measurement of the body, and in `Localizer`
	 * it is not -- the vertical channel is a torso accelerometer plus a constant
	 * downward `CONSTANT_ACCELERATION` fudge, deliberately not physical, whose
	 * job is to stop the skeleton drifting upward. Sizing a physics constraint
	 * from a number that exists to compensate for the absence of physics gives
	 * the constraint no meaning.
	 *
	 * So the vertical load is taken from the contact state, which is an
	 * independent measurement, and the vertical channel is left entirely alone
	 * for the ballistic arc (#25) to own. What remains is exactly the claim
	 * issue #6 makes: a body with nothing on the floor cannot accelerate
	 * sideways.
	 */
	fun limitHorizontal(
		comAcceleration: Vector3,
		footOnGround: Boolean,
		friction: Float = FRICTION,
		tolerance: Float = 0f,
	): Vector3 {
		val limit = maxHorizontalAcceleration(footOnGround, friction)
		val h = horizontal(comAcceleration)
		if (h <= limit + tolerance || h <= 0f) return comAcceleration
		val scale = limit / h
		return Vector3(comAcceleration.x * scale, comAcceleration.y, comAcceleration.z * scale)
	}

	private fun horizontal(f: Vector3): Float = sqrt(f.x * f.x + f.z * f.z)
}
