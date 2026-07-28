package dev.slimevr.unit

import dev.slimevr.tracking.processor.KinematicHeadingSolver
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * Tests for the global yaw solve.
 *
 * Each case builds tracker orientations from known ground-truth yaw errors and
 * checks the solver recovers them relative to a pinned reference. The cases
 * that matter most are the ones where part of the body is *not* reachable
 * through observable joints — the solver has to say so rather than return a
 * fabricated zero.
 */
class KinematicHeadingSolverTests {

	/** Heading is rotation about Y: SlimeVR's world frame is Y-up. */
	private fun yaw(radians: Double) = Quaternion(
		cos(radians / 2).toFloat(),
		0f,
		sin(radians / 2).toFloat(),
		0f,
	)

	private fun aboutX(radians: Double) = Quaternion(
		cos(radians / 2).toFloat(),
		sin(radians / 2).toFloat(),
		0f,
		0f,
	)

	private fun deg(d: Double) = d * PI / 180.0

	private fun wrap(a: Double): Double {
		var x = a
		while (x > PI) x -= 2 * PI
		while (x <= -PI) x += 2 * PI
		return x
	}

	/** Local -X, matching the hinge axis used by Constraint.kt. */
	private val hinge = Vector3(-1f, 0f, 0f)

	/**
	 * Drives a chain of segments through a flex range, applying each tracker's
	 * true yaw error to what the solver is told.
	 */
	private fun run(
		solver: KinematicHeadingSolver,
		trueYaw: Map<Int, Double>,
		frames: Int = 120,
	) {
		for (i in 0 until frames) {
			val flex = deg(-40.0 + 0.7 * i)
			val rotations = trueYaw.keys.associateWith { id ->
				// Each segment flexes by a different amount so the joints are
				// genuinely articulating rather than moving rigidly together.
				val base = aboutX(flex * (0.2 + 0.25 * id))
				yaw(trueYaw.getValue(id)).times(base)
			}
			solver.observe(rotations)
		}
	}

	@Test
	fun recoversASingleJoint() {
		val solver = KinematicHeadingSolver()
		solver.addJoint(0, 1, hinge, hinge)
		run(solver, mapOf(0 to 0.0, 1 to deg(25.0)))

		val sol = solver.solve(referenceTracker = 0)
		assertTrue(sol.getValue(0).solved)
		assertTrue(sol.getValue(1).solved)
		// The reference is pinned.
		assertTrue(abs(sol.getValue(0).yawRad) < 1e-9)
		// Tracker 1 drifted +25 deg, so the correction is -25 deg.
		val err = abs(wrap(sol.getValue(1).yawRad + deg(25.0)))
		assertTrue(err < deg(1.5), "expected about -25 deg, got ${sol.getValue(1).yawRad}")
	}

	@Test
	fun propagatesAlongAChain() {
		// 0 -- 1 -- 2. Tracker 2's offset is only reachable through tracker 1,
		// which is the case a per-joint estimator cannot resolve on its own.
		val solver = KinematicHeadingSolver()
		solver.addJoint(0, 1, hinge, hinge)
		solver.addJoint(1, 2, hinge, hinge)
		run(solver, mapOf(0 to 0.0, 1 to deg(15.0), 2 to deg(-30.0)))

		val sol = solver.solve(referenceTracker = 0)
		assertTrue(sol.getValue(1).solved && sol.getValue(2).solved)
		assertTrue(abs(wrap(sol.getValue(1).yawRad + deg(15.0))) < deg(2.0))
		assertTrue(
			abs(wrap(sol.getValue(2).yawRad + deg(-30.0))) < deg(2.0),
			"chain end should be recovered, got ${sol.getValue(2).yawRad}",
		)
	}

	@Test
	fun reportsUnreachableTrackersAsUnsolved() {
		// Tracker 2 is registered but has no joint connecting it to anything.
		// Returning a confident zero for it would be worse than useless: it
		// would assert "no correction needed" where the truth is "no
		// information".
		val solver = KinematicHeadingSolver()
		solver.addJoint(0, 1, hinge, hinge)
		solver.addJoint(2, 3, hinge, hinge)
		run(solver, mapOf(0 to 0.0, 1 to deg(20.0), 2 to deg(40.0), 3 to deg(50.0)))

		val sol = solver.solve(referenceTracker = 0)
		assertTrue(sol.getValue(1).solved, "same component as the reference")
		assertFalse(sol.getValue(2).solved, "disconnected component must be unsolved")
		assertFalse(sol.getValue(3).solved, "disconnected component must be unsolved")
	}

	@Test
	fun unobservableJointDoesNotConnectTheGraph() {
		// A vertical hinge axis carries no heading information, so a tracker
		// reachable only through it is not really reachable at all.
		val solver = KinematicHeadingSolver()
		val vertical = Vector3(0f, 1f, 0f)
		solver.addJoint(0, 1, hinge, hinge)
		solver.addJoint(1, 2, vertical, vertical)
		run(solver, mapOf(0 to 0.0, 1 to deg(20.0), 2 to deg(35.0)))

		val sol = solver.solve(referenceTracker = 0)
		assertTrue(sol.getValue(1).solved)
		assertFalse(
			sol.getValue(2).solved,
			"a joint that cannot observe heading must not connect the graph",
		)
	}

	@Test
	fun overDeterminedGraphIsLeastSquares() {
		// A loop: 0-1, 1-2, 0-2. Every measurement is consistent here, so the
		// solve should reproduce the truth rather than being pulled off it by
		// the redundant edge.
		val solver = KinematicHeadingSolver()
		solver.addJoint(0, 1, hinge, hinge)
		solver.addJoint(1, 2, hinge, hinge)
		solver.addJoint(0, 2, hinge, hinge)
		run(solver, mapOf(0 to 0.0, 1 to deg(10.0), 2 to deg(-20.0)))

		val sol = solver.solve(referenceTracker = 0)
		assertTrue(sol.getValue(1).solved && sol.getValue(2).solved)
		assertTrue(abs(wrap(sol.getValue(1).yawRad + deg(10.0))) < deg(2.0))
		assertTrue(abs(wrap(sol.getValue(2).yawRad + deg(-20.0))) < deg(2.0))
	}

	@Test
	fun referenceChoiceShiftsButPreservesRelativeAngles() {
		// Only relative headings are observable, so changing the pinned tracker
		// must rigidly shift every answer and leave the differences alone.
		val solver = KinematicHeadingSolver()
		solver.addJoint(0, 1, hinge, hinge)
		solver.addJoint(1, 2, hinge, hinge)
		run(solver, mapOf(0 to 0.0, 1 to deg(15.0), 2 to deg(-30.0)))

		val a = solver.solve(referenceTracker = 0)
		val b = solver.solve(referenceTracker = 2)

		val relA = wrap(a.getValue(1).yawRad - a.getValue(0).yawRad)
		val relB = wrap(b.getValue(1).yawRad - b.getValue(0).yawRad)
		assertTrue(
			abs(wrap(relA - relB)) < deg(0.5),
			"relative angles must not depend on the gauge choice",
		)
		assertTrue(abs(b.getValue(2).yawRad) < 1e-9, "new reference must be pinned")
	}

	@Test
	fun emptySolverReturnsNothing() {
		val solver = KinematicHeadingSolver()
		assertTrue(solver.solve(referenceTracker = 0).isEmpty())
		assertTrue(solver.observableJoints == 0)
	}
}
