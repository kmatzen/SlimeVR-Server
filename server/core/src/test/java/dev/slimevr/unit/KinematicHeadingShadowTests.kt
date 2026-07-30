package dev.slimevr.unit

import dev.slimevr.replay.FixedStepClock
import dev.slimevr.tracking.processor.HumanPoseManager
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Tests for the shadow runner that reports the kinematic heading solve
 * alongside Stay Aligned.
 *
 * Whether the estimate is *right* is covered elsewhere. What is specific to
 * this class is how it partitions the body and when it speaks: hinges exist
 * only at knees and elbows, so the joint graph comes apart into one component
 * per limb, and each needs its own pinned reference because heading is not
 * observable across the gap between them.
 */
class KinematicHeadingShadowTests {

	/** 10 ms exactly, so the frame count below lands on the window boundary. */
	private val stepNanos = 10_000_000L
	private val framesPerWindow = 1000

	private fun about(axis: Vector3, radians: Double): Quaternion {
		val half = radians / 2
		return Quaternion(cos(half).toFloat(), axis.unit() * sin(half).toFloat())
	}

	@Test
	fun reportsOneComponentPerLegPinnedAtTheThigh() {
		val trackers = TestTrackerSet()
		val hpm = HumanPoseManager(trackers.allL)
		val shadow = hpm.skeleton.kinematicHeading

		val clock = FixedStepClock(stepNanos)
		shadow.clock = clock.clock

		val hinge = Vector3(-1f, 0f, 0f)

		// One window's worth of knee flexion, plus a frame to cross the boundary.
		for (frame in 0..framesPerWindow) {
			val flexion = 0.7 * sin(2 * PI * frame / 240.0)
			trackers.leftThigh.setRotation(Quaternion.IDENTITY)
			trackers.leftCalf.setRotation(about(hinge, flexion))
			trackers.rightThigh.setRotation(Quaternion.IDENTITY)
			trackers.rightCalf.setRotation(about(hinge, flexion))

			if (frame == framesPerWindow - 1) {
				assertTrue(
					shadow.lastReport.isEmpty(),
					"Nothing should be reported until the window has elapsed",
				)
			}

			shadow.update()
			clock.advance()
		}

		val report = shadow.lastReport
		assertEquals(2, report.size, "One connected component per leg, since no hinge joins them")

		val byLabel = report.associateBy { it.label }
		assertEquals(
			setOf("LEFT_LOWER_LEG", "RIGHT_LOWER_LEG"),
			byLabel.keys,
			"Each component is named for the hinge bones it contains",
		)

		val left = byLabel.getValue("LEFT_LOWER_LEG")
		assertEquals(
			trackers.leftThigh,
			left.reference,
			"The gauge is pinned at the most root-ward tracker, which no joint treats as a child",
		)
		assertEquals(1, left.totalJoints, "A leg contributes exactly one hinge")
		assertEquals(
			setOf(trackers.leftThigh, trackers.leftCalf),
			left.residuals.mapTo(HashSet()) { it.tracker },
			"Both sides of the knee are reported",
		)
	}

	/**
	 * With no thigh trackers there is no observable joint anywhere, and the
	 * runner has to stay quiet rather than report a body it cannot see.
	 */
	@Test
	fun reportsNothingWhenNoJointHasTrackersOnBothSides() {
		val trackers = TestTrackerSet()
		val hpm = HumanPoseManager(
			listOf(trackers.head, trackers.chest, trackers.hip, trackers.leftCalf, trackers.rightCalf),
		)
		val shadow = hpm.skeleton.kinematicHeading

		val clock = FixedStepClock(stepNanos)
		shadow.clock = clock.clock

		for (frame in 0..framesPerWindow) {
			shadow.update()
			clock.advance()
		}

		assertTrue(shadow.lastReport.isEmpty(), "No joints, so nothing to report")
	}
}
