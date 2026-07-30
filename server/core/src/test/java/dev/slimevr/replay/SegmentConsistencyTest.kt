package dev.slimevr.replay

import dev.slimevr.metrics.SegmentConsistency
import dev.slimevr.metrics.SegmentConsistencyAccumulator
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertTrue

/**
 * What the layered leg corrections cost in kinematic consistency.
 *
 * Issue #4 argues the tracking pipeline's limit is its architecture: a chain of
 * stages each correcting symptoms produced by the one before it, with
 * information flowing only forwards, so a constraint discovered late cannot
 * inform the joint angles that violated it.
 *
 * That is an argument about structure. These tests turn it into a number.
 * `LegTweaks` cannot express "this foot is planted" as a fact about the pose,
 * because by the time it knows, the pose has been built; all it can do is move
 * the foot. Moving one end of a segment without moving the other changes the
 * segment's length, so the corrected output is not a pose the skeleton could
 * hold. How much it is not one is measurable, and it is measurable on synthetic
 * motion today.
 *
 * See [SegmentConsistencyAccumulator] for why this is not covered by the
 * existing pose metrics, and why it sidesteps the #15 prerequisite that issue #4
 * lists for evaluating the proposal.
 */
class SegmentConsistencyTest {

	private val rateHz = 100f
	private val frames = 400

	data class Run(
		val consistency: SegmentConsistency,
		val footSlideMPerSec: Float,
		val floorClipMaxM: Float,
	) {
		fun report(label: String): String = "%-26s slide=%.6f m/s  clip=%.6f m  segViolation mean=%.6f m max=%.6f m (%.2f%%)".format(
			label,
			footSlideMPerSec,
			floorClipMaxM,
			consistency.meanViolationM,
			consistency.maxViolationM,
			consistency.meanViolationFraction * 100f,
		)
	}

	/**
	 * The metric's own calibration.
	 *
	 * With the corrections off, the computed trackers are copied straight from
	 * the bone tree, so observed and reference lengths are the same quantity read
	 * twice and the violation must be zero to floating-point. If it is not, the
	 * metric is measuring some offset between tracker bones and joint positions
	 * rather than measuring deformation, and every number below is that offset.
	 */
	@Test
	fun forwardKinematicsAloneIsPerfectlyConsistent() {
		for (motion in SyntheticMotion.names) {
			val run = replay(motion, corrections = false)
			println(run.report("$motion, FK only"))

			assertTrue(
				run.consistency.maxViolationM < 1e-5f,
				"forward kinematics alone deformed a segment by " +
					"${run.consistency.maxViolationM} m on '$motion'. FK composes " +
					"rotations down fixed-length bones and cannot do that, so this " +
					"metric is measuring something other than deformation.",
			)
		}
	}

	/**
	 * The measurement issue #4 is about: the corrections deform the skeleton, and
	 * by how much.
	 */
	@Test
	fun theLegCorrectionsDeformTheSkeleton() {
		val rows = SyntheticMotion.names.map { motion ->
			val off = replay(motion, corrections = false)
			val on = replay(motion, corrections = true)
			println(off.report("$motion, FK only"))
			println(on.report("$motion, +legtweaks"))
			motion to on
		}

		val worst = rows.maxByOrNull { it.second.consistency.maxViolationM }!!

		println(
			"worst deformation: %s, %.4f m (%.1f%% of segment length)".format(
				worst.first,
				worst.second.consistency.maxViolationM,
				worst.second.consistency.meanViolationFraction * 100f,
			),
		)

		assertTrue(
			worst.second.consistency.maxViolationM > 1e-3f,
			"the leg corrections deformed no segment by more than a millimetre. " +
				"If that is now true they have stopped moving joint positions " +
				"independently, and issue #4's argument about this pipeline needs " +
				"revisiting.",
		)
	}

	/**
	 * The trade, stated directly: the corrections buy positional accuracy with
	 * kinematic validity.
	 *
	 * This is the shape of the claim issue #4 makes about every stage in the
	 * chain. It is worth pinning on the one case where both halves are large,
	 * because "the corrections are wrong" is not the claim -- they demonstrably
	 * fix what they set out to fix. The claim is that fixing it downstream of the
	 * pose costs something that fixing it *in* the pose would not.
	 */
	@Test
	fun theCorrectionsTradeKinematicValidityForFloorAccuracy() {
		val off = replay("squat", corrections = false)
		val on = replay("squat", corrections = true)

		println(off.report("squat, FK only"))
		println(on.report("squat, +legtweaks"))

		assertTrue(
			on.floorClipMaxM < off.floorClipMaxM,
			"the corrections did not reduce floor penetration on squat " +
				"(${on.floorClipMaxM} m against ${off.floorClipMaxM} m), so this is " +
				"not measuring the trade it claims to",
		)
		assertTrue(
			on.consistency.maxViolationM > off.consistency.maxViolationM,
			"the corrections reduced floor penetration without deforming anything, " +
				"which would mean they found a kinematically valid pose -- the thing " +
				"issue #4 says the architecture cannot do",
		)
	}

	/**
	 * The constructive half: the same corrections, re-solved in joint space.
	 *
	 * Issue #4's proposal is a single estimator whose parameters are the pose
	 * itself, so that a constraint discovered late changes the joint angles
	 * rather than being pasted onto the output. This is one constraint -- fixed
	 * segment lengths -- enforced that way, on the stage the measurements show is
	 * violating it.
	 *
	 * The claim is narrow and worth stating precisely. It is not that this is
	 * better tracking. It is that the deformation was not the price of the
	 * correction: the same ankle targets are reachable by a pose the skeleton can
	 * actually hold, so paying in segment length bought nothing.
	 */
	@Test
	fun projectingToJointSpaceRemovesTheDeformation() {
		for (motion in SyntheticMotion.names) {
			val plain = replay(motion, corrections = true)
			val projected = replay(motion, corrections = true, jointSpace = true)

			println(plain.report("$motion, +legtweaks"))
			println(projected.report("$motion, +jointspace"))

			assertTrue(
				projected.consistency.maxViolationM <= 1e-4f,
				"joint-space projection left ${projected.consistency.maxViolationM} m " +
					"of deformation on '$motion'. The output is built from the segment " +
					"lengths, so it cannot violate them -- if this fails the reference " +
					"lengths are not the ones being measured against.",
			)
		}
	}

	/**
	 * The projection must not undo what the corrections achieved.
	 *
	 * The whole point of the leg corrections is that they fix floor penetration
	 * and skating, and they demonstrably do. A replacement that restored kinematic
	 * validity by giving that back would not be an improvement, it would be a
	 * slower way of turning the corrections off.
	 */
	@Test
	fun projectingToJointSpaceKeepsWhatTheCorrectionsAchieved() {
		val uncorrected = replay("squat", corrections = false)
		val plain = replay("squat", corrections = true)
		val projected = replay("squat", corrections = true, jointSpace = true)

		println(uncorrected.report("squat, FK only"))
		println(plain.report("squat, +legtweaks"))
		println(projected.report("squat, +jointspace"))

		assertTrue(
			projected.floorClipMaxM < uncorrected.floorClipMaxM,
			"projected floor penetration ${projected.floorClipMaxM} m is no better " +
				"than uncorrected ${uncorrected.floorClipMaxM} m, so the correction " +
				"has been undone rather than made consistent",
		)
		assertTrue(
			projected.footSlideMPerSec < uncorrected.footSlideMPerSec,
			"projected foot slide ${projected.footSlideMPerSec} m/s is no better " +
				"than uncorrected ${uncorrected.footSlideMPerSec} m/s",
		)
	}

	private fun replay(
		motion: String,
		corrections: Boolean,
		jointSpace: Boolean = false,
	): Run {
		val hmd = mkTracker(0, TrackerPosition.HEAD, isHmd = true)
		val chest = mkTracker(1, TrackerPosition.CHEST)
		val hip = mkTracker(2, TrackerPosition.HIP)
		val leftThigh = mkTracker(3, TrackerPosition.LEFT_UPPER_LEG)
		val leftCalf = mkTracker(4, TrackerPosition.LEFT_LOWER_LEG)
		val rightThigh = mkTracker(5, TrackerPosition.RIGHT_UPPER_LEG)
		val rightCalf = mkTracker(6, TrackerPosition.RIGHT_LOWER_LEG)

		val trackers = listOf(hmd, chest, hip, leftThigh, leftCalf, rightThigh, rightCalf)
		val hpm = HumanPoseManager(trackers)
		val height = hpm.userHeightFromConfig
		hpm.skeleton.hasKneeTrackers = true

		hpm.skeleton.legTweaks.setConfig(
			dev.slimevr.config.LegTweaksConfig().apply { projectToJointSpace = jointSpace },
		)
		hpm.setLegTweaksEnabled(corrections)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, corrections)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, corrections)

		val clock = FixedStepClock(1f / rateHz)
		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val consistency = SegmentConsistencyAccumulator()
		val pose = dev.slimevr.metrics.PoseMetricsAccumulator()
		val dt = 1f / rateHz

		for (frame in SyntheticMotion.sequence(motion, frames, rateHz)) {
			clock.advance()

			hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(frame.leftCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			hpm.update()

			consistency.observe(hpm.skeleton)
			pose.observeAnkles(
				hpm.skeleton.computedLeftFootTracker!!.position,
				hpm.skeleton.computedRightFootTracker!!.position,
				dt,
			)
		}

		val poseResult = pose.result(height)
		return Run(
			consistency = consistency.result(),
			footSlideMPerSec = poseResult.footSlideMPerSec,
			floorClipMaxM = poseResult.floorClipMaxM,
		)
	}

	private fun mkTracker(
		id: Int,
		position: TrackerPosition,
		isHmd: Boolean = false,
	): Tracker {
		val tracker = Tracker(
			device = null,
			id = id,
			name = position.name,
			trackerPosition = position,
			trackerNum = 0,
			hasPosition = isHmd,
			hasRotation = true,
			isComputed = isHmd,
			imuType = null,
			allowReset = !isHmd,
			allowMounting = !isHmd,
			isHmd = isHmd,
			trackRotDirection = false,
		)
		tracker.status = TrackerStatus.OK
		return tracker
	}
}
