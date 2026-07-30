package dev.slimevr.unit

import dev.slimevr.tracking.processor.BoneType
import dev.slimevr.tracking.processor.HingeJoint
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.KinematicHeading
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Tests for the adapter between the skeleton and the heading estimator.
 *
 * The estimator and the solver are already covered on synthetic input. What is
 * untested — and what actually decides whether any of it works on a real
 * body — is whether the joint list handed to them describes this skeleton
 * correctly: the right joints, only where two trackers genuinely observe them,
 * and an axis that matches the one [dev.slimevr.tracking.processor.Constraint]
 * decomposes hinges about.
 */
class HingeJointTests {

	private fun deg(d: Double) = d * PI / 180.0

	/** Rotation of [radians] about [axis], which need not be normalised. */
	private fun about(axis: Vector3, radians: Double): Quaternion {
		val half = radians / 2
		return Quaternion(cos(half).toFloat(), axis.unit() * sin(half).toFloat())
	}

	/** Heading is rotation about Y: SlimeVR's world frame is Y-up. */
	private fun yaw(radians: Double) = about(Vector3(0f, 1f, 0f), radians)

	private fun wrap(a: Double): Double = atan2(sin(a), cos(a))

	@Test
	fun findsBothKnees() {
		val trackers = TestTrackerSet()
		val hpm = HumanPoseManager(trackers.allL)

		val joints = HingeJoint.collect(hpm.skeleton.allHumanBones.asIterable())

		assertEquals(
			setOf(BoneType.LEFT_LOWER_LEG, BoneType.RIGHT_LOWER_LEG),
			joints.mapTo(HashSet()) { it.boneType },
			"Both knees have a thigh and a calf tracker, so both are observable joints",
		)

		val left = joints.first { it.boneType == BoneType.LEFT_LOWER_LEG }
		assertEquals(trackers.leftThigh, left.parentTracker, "Thigh is the parent side of the knee")
		assertEquals(trackers.leftCalf, left.childTracker, "Calf is the child side of the knee")
	}

	/**
	 * The trap this whole adapter exists to avoid.
	 *
	 * With no thigh tracker the skeleton still gives the thigh bone a rotation,
	 * borrowed from the calf. Both sides of the knee would then be the same
	 * measurement, the joint angle would be identically zero, and the heading
	 * estimate would come back as a confident, perfectly aligned pair — a
	 * fabricated agreement, which is worse than no reading at all.
	 */
	@Test
	fun skipsJointsWhoseSidesShareATracker() {
		val trackers = TestTrackerSet()
		val calvesOnly = listOf(trackers.head, trackers.chest, trackers.hip, trackers.leftCalf, trackers.rightCalf)
		val hpm = HumanPoseManager(calvesOnly)

		val joints = HingeJoint.collect(hpm.skeleton.allHumanBones.asIterable())

		assertTrue(
			joints.isEmpty(),
			"A knee with no thigh tracker observes nothing, but found $joints",
		)
	}

	/**
	 * Checks the collected axis against [dev.slimevr.tracking.processor.Constraint]
	 * itself rather than against the derivation that produced it.
	 *
	 * If the axis is right, bending the knee about it is a legal hinge motion and
	 * the constraint passes the rotation through untouched. The contrast case is
	 * what gives this teeth: the same angle about a perpendicular axis is not
	 * hinge motion, and the constraint has to alter it.
	 */
	@Test
	fun collectedAxisIsTheAxisTheConstraintUses() {
		val trackers = TestTrackerSet()
		val hpm = HumanPoseManager(trackers.allL)
		val skeleton = hpm.skeleton

		val knee = HingeJoint.collect(skeleton.allHumanBones.asIterable())
			.first { it.boneType == BoneType.LEFT_LOWER_LEG }

		val thighBone = skeleton.leftUpperLegBone
		val calfBone = skeleton.leftLowerLegBone

		val thighRot = Quaternion.IDENTITY
		thighBone.setRotation(thighRot)
		skeleton.updateBones()

		// In tracker space a hinge is exactly child = parent * R(axis, angle);
		// the bone's own rotation offset cancels out of the conjugation.
		val flexion = deg(60.0)
		val hinged = (thighRot * about(knee.childAxis, flexion)) * calfBone.rotationOffset
		val hingedAfter = calfBone.rotationConstraint.applyConstraint(hinged, calfBone)
		assertTrue(
			TrackerTestUtils.quatApproxEqual(hinged, hingedAfter, 1e-3f),
			"Bending about the collected axis is legal hinge motion, so the constraint " +
				"should pass it through. Expected <$hinged>, actual <$hingedAfter>.",
		)

		// A perpendicular axis, well past the 50 degrees of non-hinge deviation
		// the knee tolerates.
		val perpendicular = Vector3(0f, 0f, 1f)
		assertTrue(
			abs(knee.childAxis.unit().dot(perpendicular)) < 1e-3f,
			"Test assumes the knee axis is perpendicular to Z",
		)
		val twisted = (thighRot * about(perpendicular, deg(80.0))) * calfBone.rotationOffset
		val twistedAfter = calfBone.rotationConstraint.applyConstraint(twisted, calfBone)
		assertTrue(
			!TrackerTestUtils.quatApproxEqual(twisted, twistedAfter, 1e-3f),
			"Rotating about a perpendicular axis is not hinge motion and should be constrained",
		)
	}

	/**
	 * End to end: a known yaw error on one side of a real skeleton's knee,
	 * recovered through the collected joint.
	 *
	 * The knee is swept through its range rather than held still, which is what
	 * makes this more than a restatement of the geometry. A wrong axis would
	 * make the apparent heading move with the flexion angle, and the estimator's
	 * concentration — not its mean — is what would catch it.
	 */
	@Test
	fun recoversAnInjectedYawErrorThroughTheCollectedJoint() {
		val trackers = TestTrackerSet()
		val hpm = HumanPoseManager(trackers.allL)

		val knee = HingeJoint.collect(hpm.skeleton.allHumanBones.asIterable())
			.first { it.boneType == BoneType.LEFT_LOWER_LEG }

		val trueYawError = deg(7.0)
		val estimator = KinematicHeading()

		val frames = 120
		for (frame in 0 until frames) {
			val flexion = deg(80.0) * frame / (frames - 1)
			val thigh = Quaternion.IDENTITY
			val calf = thigh * about(knee.childAxis, flexion)

			estimator.addSample(
				thigh,
				knee.parentAxis,
				// The calf's own heading has drifted; everything else is true.
				yaw(trueYawError) * calf,
				knee.childAxis,
			)
		}

		assertTrue(estimator.hasEstimate, "A horizontal knee axis swept through 80 degrees is observable")
		assertTrue(
			estimator.concentration > 0.99,
			"Noise-free hinge motion about the right axis should agree with itself, " +
				"got ${estimator.concentration}",
		)
		assertEquals(
			-trueYawError,
			wrap(estimator.relativeHeadingRad),
			1e-3,
			"The correction that brings the drifted calf back onto the thigh",
		)
	}
}
