package dev.slimevr.tracking.processor

import dev.slimevr.tracking.processor.Constraint.Companion.ConstraintType
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Vector3

/**
 * A hinge joint between two *independently tracked* segments, with the hinge
 * axis expressed in each side's tracker frame.
 *
 * [KinematicHeading] can recover the relative heading of two trackers from the
 * joint between them, and [KinematicHeadingSolver] can reconcile several such
 * joints at once — but neither knows anything about this skeleton. The skeleton
 * is where hinges are declared, in the [Constraint] attached to each [Bone], and
 * this is the adapter between the two: it walks the bones and produces the joint
 * list a solver can consume.
 *
 * ## Where the axis comes from
 *
 * [Constraint.applyConstraint] resolves a bone's rotation into the frame of its
 * parent and constrains the twist about local `-X`. So `-X` *is* the hinge axis,
 * and the only work here is re-expressing it in the frame each tracker reports
 * in.
 *
 * Writing `P`/`C` for the parent and child bones' global rotations and `O` for a
 * bone's `rotationOffset`, the constraint frame is
 *
 * ```
 *   F = P * (O_parent⁻¹ * O_child)
 * ```
 *
 * and the hinge axis in the world is `F * (-X)`. A bone's global rotation is set
 * straight from its tracker as `bone = tracker * O_bone`, so substituting:
 *
 * ```
 *   parent side:  F * (-X) = tracker_parent * (O_child * (-X))
 *   child side:   C * (-X) = tracker_child  * (O_child * (-X))
 * ```
 *
 * The two sides reduce to the *same* local vector, `O_child * (-X)`, which is
 * the sense in which both trackers are supposed to agree about the axis. Both
 * fields are still carried separately because that is the shape
 * [KinematicHeading.addSample] takes, and because nothing guarantees a future
 * skeleton keeps the property.
 *
 * ## Why tracker rotations rather than bone rotations
 *
 * The identity above lets the caller feed raw `tracker.getRotation()` instead of
 * the bones. That matters: bone rotations are downstream of the extended knee
 * model, which blends the lower leg's pitch and roll into the upper leg bone.
 * Reading the bones would make the two sides of the knee partly the same
 * measurement, and a heading estimator fed its own input twice reports
 * confident agreement it has not earned.
 */
data class HingeJoint(
	/** The child bone — the one carrying the hinge constraint. */
	val boneType: BoneType,
	val parentTracker: Tracker,
	val childTracker: Tracker,
	/** Hinge axis in [parentTracker]'s frame. */
	val parentAxis: Vector3,
	/** Hinge axis in [childTracker]'s frame. */
	val childAxis: Vector3,
) {
	companion object {
		/** Matches the axis [Constraint] decomposes hinges about. */
		private val HINGE_AXIS = Vector3.NEG_X

		/**
		 * Collects every hinge whose two sides are driven by different trackers.
		 *
		 * Requires `attachedTracker` to be current — [Bone.attachedTracker] is
		 * refreshed when the tracker assignment changes, and is null for a bone
		 * with no tracker of its own.
		 */
		fun collect(bones: Iterable<Bone>): List<HingeJoint> = bones.mapNotNull { at(it) }

		private fun at(bone: Bone): HingeJoint? {
			val type = bone.rotationConstraint.constraintType
			if (type != ConstraintType.HINGE && type != ConstraintType.LOOSE_HINGE) return null

			val parent = bone.parent ?: return null
			val childTracker = bone.attachedTracker ?: return null
			val parentTracker = parent.attachedTracker ?: return null

			// A segment with no tracker of its own is driven from a neighbour —
			// the skeleton falls back to the other end of the same limb rather
			// than leaving the bone unset. Both ends then carry one measurement,
			// the joint angle is identically zero by construction, and the
			// heading estimate reads as a perfectly aligned pair. Excluding the
			// case is the whole reason this checks the trackers and not the
			// bones.
			if (parentTracker === childTracker) return null

			// Both sides reduce to the same local vector; see the class comment.
			val axis = bone.rotationOffset.sandwich(HINGE_AXIS)

			return HingeJoint(
				boneType = bone.boneType,
				parentTracker = parentTracker,
				childTracker = childTracker,
				parentAxis = axis,
				childAxis = axis,
			)
		}
	}
}
