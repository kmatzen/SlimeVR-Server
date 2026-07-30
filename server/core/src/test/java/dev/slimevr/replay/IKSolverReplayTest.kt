package dev.slimevr.replay

import dev.slimevr.metrics.PoseMetrics
import dev.slimevr.metrics.PoseMetricsAccumulator
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerRole
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * What the IK solver does, now that it runs at all.
 *
 * `IKSolver.solve()` had no call site in this repository or in SlimeVR
 * upstream, and never had one: the introducing commit (upstream `0a08d574`,
 * "Positional tracker support (#920)", 31 Oct 2025) added the field, the chain
 * builder, the `USE_POSITION` wiring and the reset path, and no caller. The
 * flag it gates is read only by the early return at the top of `solve()`, so
 * toggling it could not do anything. See issue #4.
 *
 * It is now called from `HumanSkeleton.updatePose()`. This measures the
 * consequence rather than asserting a hoped-for one, because there is no prior
 * behaviour to compare against — the code has never executed, so nothing about
 * it is established.
 *
 * `USE_POSITION` now defaults to **false**, so the committed baselines in
 * [SkeletonReplayTest] describe the same pipeline they did before. That is
 * asserted here rather than assumed.
 */
class IKSolverReplayTest {

	private val rateHz = 100f
	private val frames = 400

	/**
	 * With the toggle off, wiring the solver in must change nothing at all.
	 *
	 * This is what makes the change safe to land: every committed baseline
	 * keeps its meaning, and the solver is opt-in.
	 */
	@Test
	fun theSolverIsInertWhenTheToggleIsOff() {
		val baseline = ReplayBaseline.load()
		val failures = mutableListOf<String>()

		for (motion in SyntheticMotion.names) {
			val metrics = replay(motion, usePosition = false)
			for ((metric, value) in metrics.toMap()) {
				val entry = baseline["$motion/$metric"] ?: continue
				if (abs(value - entry.value) > entry.tolerance) {
					failures.add("$motion/$metric: baseline ${entry.value}, got $value")
				}
			}
		}

		assertTrue(
			failures.isEmpty(),
			"calling ikSolver.solve() changed the pose even with USE_POSITION off, so the " +
				"early return in solve() is not the only thing gating it: $failures",
		)
	}

	/**
	 * The measured answer to "what does wiring it up change for a normal
	 * setup": **nothing at all**, and that is a property of the tracker set
	 * rather than of the call.
	 *
	 * `buildChains` discards the root chain unless some chain has a *tail*
	 * constraint -- its own comment is "check if there is any constraints
	 * (other than the head) in the model" -- and a rotation-only SlimeVR set
	 * has exactly one positional tracker, the headset, which is the root. So
	 * `rootChain` is null and `solve()` returns on its first line.
	 *
	 * This is worth pinning rather than just observing: it means turning
	 * `USE_POSITION` on cannot affect the overwhelmingly common configuration,
	 * which is the single most useful thing to know about the setting.
	 */
	@Test
	fun rotationOnlyTrackersGiveTheSolverNothingToDo() {
		val report = StringBuilder()
		var moved = false

		for (motion in SyntheticMotion.names) {
			val off = replay(motion, usePosition = false).toMap()
			val on = replay(motion, usePosition = true).toMap()
			for ((metric, offValue) in off) {
				val onValue = on.getValue(metric)
				val delta = onValue - offValue
				if (abs(delta) > 1e-6f) moved = true
				report.append(
					"IK %-28s %-26s off %12.6f  on %12.6f  delta %12.6f\n"
						.format(motion, metric, offValue, onValue, delta),
				)
			}
		}

		println(report)

		assertTrue(
			!moved,
			"enabling USE_POSITION moved a metric on a rotation-only tracker set. That would " +
				"mean a chain was built with no positional tail constraint, which buildChains " +
				"is supposed to discard",
		)
	}

	/**
	 * And the converse, which is what makes the wire-up worth anything: give
	 * the solver a positional tracker *below* the root and the pose moves.
	 *
	 * Without this the change would be unfalsifiable — every observation so far
	 * is equally consistent with `solve()` still doing nothing.
	 */
	@Test
	fun aPositionalTrackerBelowTheRootEngagesTheSolver() {
		val off = replayWithPositionalFoot(usePosition = false).toMap()
		val on = replayWithPositionalFoot(usePosition = true).toMap()

		val report = StringBuilder()
		var moved = false
		for ((metric, offValue) in off) {
			val onValue = on.getValue(metric)
			val delta = onValue - offValue
			if (abs(delta) > 1e-6f) moved = true
			report.append(
				"IK+foot %-26s off %12.6f  on %12.6f  delta %12.6f\n".format(metric, offValue, onValue, delta),
			)
		}
		println(report)

		assertTrue(
			moved,
			"with a positional foot tracker the solver still changed nothing, so solve() is " +
				"reached but inert and positional tracker support remains dead",
		)
	}

	/**
	 * `squat` with an extra left-foot tracker that reports a *position*, held
	 * at a fixed point on the floor. That is the shape of a Vive tracker or
	 * controller in an otherwise IMU-only setup, and it is the case
	 * `USE_POSITION` exists for.
	 */
	private fun replayWithPositionalFoot(usePosition: Boolean): PoseMetrics {
		val hmd = mkTracker(0, TrackerPosition.HEAD, isHmd = true)
		val chest = mkTracker(1, TrackerPosition.CHEST)
		val hip = mkTracker(2, TrackerPosition.HIP)
		val leftThigh = mkTracker(3, TrackerPosition.LEFT_UPPER_LEG)
		val leftCalf = mkTracker(4, TrackerPosition.LEFT_LOWER_LEG)
		val rightThigh = mkTracker(5, TrackerPosition.RIGHT_UPPER_LEG)
		val rightCalf = mkTracker(6, TrackerPosition.RIGHT_LOWER_LEG)
		val leftFoot = mkTracker(7, TrackerPosition.LEFT_FOOT, hasPosition = true)

		val trackers = listOf(hmd, chest, hip, leftThigh, leftCalf, rightThigh, rightCalf, leftFoot)
		val hpm = HumanPoseManager(trackers)
		val height = hpm.userHeightFromConfig
		hpm.skeleton.hasKneeTrackers = true

		hpm.setLegTweaksEnabled(false)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, false)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, false)
		hpm.setToggle(SkeletonConfigToggles.USE_POSITION, usePosition)

		val clock = FixedStepClock(1f / rateHz)
		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val accumulator = PoseMetricsAccumulator()
		val dt = 1f / rateHz

		for (frame in SyntheticMotion.sequence("squat", frames, rateHz)) {
			clock.advance()

			hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(frame.leftCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			// Planted on the floor, slightly to the left of centre.
			leftFoot.setRotation(frame.leftCalf)
			leftFoot.position = Vector3(-0.1f, 0f, 0f)

			hpm.update()

			accumulator.observeAnkles(
				hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position,
				hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position,
				dt,
			)
		}

		return accumulator.result(height)
	}

	/**
	 * Determinism, on the same terms as the rest of the replay suite. A solver
	 * that iterates to a tolerance is exactly the kind of thing that can be
	 * order- or state-dependent, and a baseline over it would be worthless.
	 */
	@Test
	fun solvedReplayIsDeterministic() {
		for (motion in SyntheticMotion.names) {
			assertEquals(
				replay(motion, usePosition = true).toMap(),
				replay(motion, usePosition = true).toMap(),
				"replay of '$motion' with the IK solver enabled is not reproducible",
			)
		}
	}

	private fun replay(motion: String, usePosition: Boolean): PoseMetrics {
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

		// The leg corrections must be off, and not for tidiness: on clean
		// synthetic input they drive every metric here to exactly zero, so a
		// run with them enabled cannot show a difference between any two
		// configurations. Leaving them on made the first version of this test
		// report "the solver changes nothing" and "the solver changes
		// everything" simultaneously, both meaningless.
		hpm.setLegTweaksEnabled(false)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, false)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, false)

		hpm.setToggle(SkeletonConfigToggles.USE_POSITION, usePosition)

		val clock = FixedStepClock(1f / rateHz)
		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val accumulator = PoseMetricsAccumulator()
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

			accumulator.observeAnkles(
				hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position,
				hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position,
				dt,
			)
		}

		return accumulator.result(height)
	}

	private fun mkTracker(
		id: Int,
		position: TrackerPosition,
		isHmd: Boolean = false,
		hasPosition: Boolean = false,
	): Tracker {
		val tracker = Tracker(
			device = null,
			id = id,
			name = position.name,
			trackerPosition = position,
			trackerNum = 0,
			hasPosition = isHmd || hasPosition,
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
