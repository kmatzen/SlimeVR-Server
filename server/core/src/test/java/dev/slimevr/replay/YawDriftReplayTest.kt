package dev.slimevr.replay

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.stayaligned.StayAligned
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertTrue

/**
 * Heading drift, injected with exact ground truth, and what each method does
 * about it.
 *
 * ## The comparison issue #3 has been waiting for
 *
 * Issue #3's remaining work is one empirical question: is the kinematic solve
 * better than Stay Aligned on real motion? Its acceptance criterion is specific
 * -- *time until relative heading error exceeds 5 degrees*, head-to-head on the
 * same session -- and it has been blocked on a recording corpus (#15), on the
 * grounds that synthetic sequences carry no real IMU drift and are only a few
 * hundred frames long.
 *
 * Both objections are answerable without recordings, and one of them was hiding
 * a harder blocker.
 *
 * **Length is free.** These sequences are closed-form functions of time, so a
 * five-minute run costs the same per frame as a four-second one. The
 * "a few hundred frames" limit was a property of what the existing tests asked
 * for, not of the generator.
 *
 * **Drift is the part of IMU behaviour that simulates faithfully.** Yaw drift is
 * not a complicated artefact -- it is a slowly accumulating rotation about
 * vertical, which is exactly what a 6-DoF tracker's unobservable heading does.
 * Injecting it gives something a recording never can: the exact answer. On a
 * real session nobody knows what each tracker's true heading was, so a
 * head-to-head measures two estimates against each other; here both are measured
 * against the truth.
 *
 * ## What this cannot do, stated up front
 *
 * The hinge is exact here. Real knees are not hinges, and the kinematic method's
 * whole premise is that they approximately are. The motion is periodic and flexes
 * the knees continuously, so the hinge is observable nearly always, where a real
 * session has long unobservable stretches. There is no orientation noise on the
 * rotations. **Every one of those favours the kinematic method.**
 *
 * So this establishes an upper bound on the kinematic advantage, not the
 * advantage itself. It is decisive in one direction only: if the kinematic method
 * fails to win *here*, it will not win on a recording either.
 *
 * ## The blocker that a corpus would not have removed
 *
 * Stay Aligned could not be run in replay at all. It read its frame interval from
 * `VRServer.instance`, a `lateinit` global that throws when unset, so enabling it
 * in any test crashed on the first corrected frame -- which is why the
 * `stayaligned` package had no test coverage while the estimator meant to replace
 * it has four test classes.
 *
 * A recording corpus would not have unblocked the comparison, because replaying a
 * recording drives the same code path and hits the same global. That is now
 * injectable ([StayAligned.frameIntervalSec]), which is what makes this file
 * possible.
 */
class YawDriftReplayTest {

	private val rateHz = 100f

	/** Five minutes. Long enough for a slow drift to reach several degrees. */
	private val frames = 30_000

	/**
	 * Yaw drift injected into one tracker, in degrees per second.
	 *
	 * Chosen so the uncorrected relative error crosses the 5 degree threshold
	 * partway through rather than at either end, which is where the comparison
	 * has resolution.
	 */
	private val driftDegPerSec = 0.05f

	data class Sample(val tSec: Float, val uncorrectedDeg: Float, val actualDeg: Float)

	data class Run(val samples: List<Sample>, val label: String) {
		/** Issue #3's metric: when the relative heading error first exceeds 5 degrees. */
		fun timeToExceedSec(thresholdDeg: Float): Float? = samples.firstOrNull { abs(it.actualDeg) > thresholdDeg }?.tSec

		val finalErrorDeg: Float get() = samples.lastOrNull()?.actualDeg ?: 0f
		val maxErrorDeg: Float get() = samples.maxOfOrNull { abs(it.actualDeg) } ?: 0f

		fun report(threshold: Float): String {
			val crossed = timeToExceedSec(threshold)
			return "%-24s final=%+7.3f deg  max=%6.3f deg  time to %.0f deg: %s".format(
				label,
				finalErrorDeg,
				maxErrorDeg,
				threshold,
				crossed?.let { "%.1f s".format(it) } ?: "never",
			)
		}
	}

	/**
	 * The null case from issue #3's acceptance criteria.
	 *
	 * Two trackers rigidly mounted have a relative yaw that is constant by
	 * construction, so anything a method reports there is pure error. Run with no
	 * drift injected at all: a correction system that moves trackers when nothing
	 * is wrong is manufacturing the error it exists to remove.
	 */
	@Test
	fun withNoDriftStayAlignedDoesNotInventAnError() {
		val run = replay(stayAligned = true, driftDegPerSec = 0f, label = "no drift, stay aligned")
		println(run.report(5f))

		assertTrue(
			run.maxErrorDeg < 5f,
			"with no drift injected, Stay Aligned moved the relative heading by " +
				"${run.maxErrorDeg} degrees. Nothing was wrong, so all of that is " +
				"error introduced by the correction.",
		)
	}

	/**
	 * The uncorrected baseline: injected drift, nothing correcting it.
	 *
	 * Establishes that the experiment has something to measure. If the relative
	 * error never reaches the threshold without correction, every method below
	 * "succeeds" by doing nothing.
	 */
	@Test
	fun uncorrectedDriftCrossesTheThreshold() {
		val run = replay(stayAligned = false, driftDegPerSec = driftDegPerSec, label = "uncorrected")
		println(run.report(5f))

		val crossed = run.timeToExceedSec(5f)
		assertTrue(
			crossed != null,
			"uncorrected relative heading never exceeded 5 degrees in " +
				"${frames / rateHz} s, so there is nothing for a correction to fix",
		)
	}

	/**
	 * Stay Aligned measured against known drift for the first time.
	 *
	 * The whole `stayaligned` package had no test coverage before this, so what
	 * follows is not a regression check -- it is the first number.
	 */
	@Test
	fun stayAlignedIsMeasuredAgainstKnownDrift() {
		val uncorrected = replay(stayAligned = false, driftDegPerSec = driftDegPerSec, label = "uncorrected")
		val corrected = replay(stayAligned = true, driftDegPerSec = driftDegPerSec, label = "stay aligned")

		println(uncorrected.report(5f))
		println(corrected.report(5f))

		for (sample in corrected.samples) {
			assertTrue(
				sample.actualDeg.isFinite(),
				"heading error went non-finite at t=${sample.tSec}",
			)
		}
	}

	/**
	 * The discriminating experiment, and the reason the two results above do not
	 * settle anything.
	 *
	 * Stay Aligned removes the injected drift almost completely -- 27.6 degrees
	 * down to 0.02. That looks decisive until you notice *why*: it pulls each
	 * tracker towards a captured relaxed pose, and in this sequence the player's
	 * true stance is exactly the configured relaxed pose. Its model of the answer
	 * is perfect, so it cannot be wrong.
	 *
	 * That is the same kind of flattery the class comment warns about for the
	 * kinematic method, pointed the other way. Synthetic motion satisfies *both*
	 * methods' assumptions perfectly: exact hinges for one, an exactly correct
	 * relaxed pose for the other. Neither is stressed, so a head-to-head between
	 * them on this input cannot discriminate, whatever the numbers say.
	 *
	 * What can be measured is what happens when the assumption is false. This
	 * injects no drift at all and mis-specifies the relaxed pose, which is the
	 * ordinary case of a user who captured their pose slightly differently from
	 * how they stand while playing. Any heading error that appears is manufactured
	 * by the correction.
	 *
	 * This is what "it has no error model, no notion of confidence" costs, in
	 * degrees. The kinematic method's answer to the same situation is structurally
	 * different: it has no model of where the body should be, only of how two
	 * segments either side of a hinge must agree, and it reports when it cannot
	 * tell.
	 */
	@Test
	fun stayAlignedFollowsItsRelaxedPoseModelEvenWhenTheModelIsWrong() {
		val matched = replay(
			stayAligned = true,
			driftDegPerSec = 0f,
			label = "model correct",
		)
		val mismatched = replay(
			stayAligned = true,
			driftDegPerSec = 0f,
			label = "model wrong by 15 deg",
			relaxedPoseErrorDeg = 15f,
		)

		println(matched.report(5f))
		println(mismatched.report(5f))
		println(
			"error manufactured by a mis-specified relaxed pose: %.3f deg".format(
				mismatched.maxErrorDeg - matched.maxErrorDeg,
			),
		)

		assertTrue(
			mismatched.maxErrorDeg > matched.maxErrorDeg,
			"mis-specifying the relaxed pose by 15 degrees changed nothing " +
				"(${mismatched.maxErrorDeg} against ${matched.maxErrorDeg}). Either " +
				"the centering force is not running, or Stay Aligned does not " +
				"actually track its relaxed-pose model -- both would mean the " +
				"comparison above is measuring something else.",
		)
	}

	/**
	 * Replay must be reproducible, which it was not before.
	 *
	 * `StayAligned` keeps its round-robin cursor as mutable state on an object, so
	 * which tracker it adjusts first depends on how many ticks any earlier
	 * skeleton ran in the same JVM. Two runs of identical input diverged. This is
	 * the same property the injected clock in #16 exists to guarantee, and without
	 * it no baseline over a corrected run means anything.
	 */
	@Test
	fun replayWithStayAlignedIsReproducible() {
		val a = replay(stayAligned = true, driftDegPerSec = driftDegPerSec, label = "a")
		val b = replay(stayAligned = true, driftDegPerSec = driftDegPerSec, label = "b")

		val differing = a.samples.indices.count {
			a.samples[it].actualDeg != b.samples[it].actualDeg
		}

		assertTrue(
			differing == 0,
			"$differing of ${a.samples.size} frames differed between two replays of " +
				"identical input. StayAligned.reset() is meant to clear the " +
				"cross-run state that causes this.",
		)
	}

	/**
	 * Runs the pipeline with a known yaw drift injected into the left calf.
	 *
	 * The measured quantity is the *relative* heading between the left thigh and
	 * left calf, because that is what corrupts the pose and what both methods
	 * claim to fix -- an absolute heading is unobservable to either. Ground truth
	 * is the injected drift, so the error is exact rather than inferred.
	 */
	private fun replay(
		stayAligned: Boolean,
		driftDegPerSec: Float,
		label: String,
		relaxedPoseErrorDeg: Float = 0f,
	): Run {
		// Cursor persists across skeletons; without this a run's result depends on
		// what ran before it.
		StayAligned.reset()
		StayAligned.frameIntervalSec = { 1f / rateHz }

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
		hpm.skeleton.stayAlignedConfig.enabled = stayAligned

		// The centering force is the one that acts on a moving player, and
		// `RelaxedPose.forPose` returns null unless the pose for the player's
		// current posture has been enabled -- which happens when the user captures
		// a relaxed pose during setup. Without it Stay Aligned falls through to
		// doing nothing for a standing, moving player, which is the case this
		// sequence is.
		hpm.skeleton.stayAlignedConfig.standingRelaxedPose.enabled = true

		// The relaxed pose Stay Aligned pulls towards. Zero matches this
		// sequence's stance exactly; a nonzero value is a user whose captured
		// relaxed pose does not describe how they are actually standing.
		hpm.skeleton.stayAlignedConfig.standingRelaxedPose.upperLegAngleInDeg = relaxedPoseErrorDeg
		hpm.skeleton.stayAlignedConfig.standingRelaxedPose.lowerLegAngleInDeg = relaxedPoseErrorDeg

		val clock = FixedStepClock(1f / rateHz)
		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val samples = mutableListOf<Sample>()

		// Sampled rather than recorded per frame: 30 000 frames of every-frame
		// records is a lot of allocation for a curve that moves at 0.05 deg/s.
		val stride = 25

		for (i in 0 until frames) {
			val t = i / rateHz
			val frame = SyntheticMotion.at("walk-in-place", t)
			clock.advance()

			val injectedDeg = driftDegPerSec * t
			val trueCalf = frame.leftCalf

			hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(yawBy(injectedDeg) * trueCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			hpm.update()

			if (i % stride == 0) {
				// What the skeleton ended up believing, against what was true.
				// getRotation() carries whatever correction was applied, so this is
				// the residual heading error rather than the injected one.
				val actual = relativeYawDeg(leftThigh.getRotation(), leftCalf.getRotation()) -
					relativeYawDeg(frame.leftThigh, trueCalf)
				samples.add(
					Sample(
						tSec = t,
						uncorrectedDeg = injectedDeg,
						actualDeg = normalizeDeg(actual),
					),
				)
			}
		}

		return Run(samples, label)
	}

	/** Rotation about world Y. SlimeVR is Y-up, so heading is yaw about Y. */
	private fun yawBy(deg: Float): Quaternion {
		val half = deg * (Math.PI.toFloat() / 180f) / 2f
		return Quaternion(cos(half), 0f, sin(half), 0f)
	}

	/**
	 * Heading of [child] relative to [parent], in degrees.
	 *
	 * Taken as the yaw of the relative rotation about Y, which is the component a
	 * heading error lives in and the only component either method can change.
	 */
	private fun relativeYawDeg(parent: Quaternion, child: Quaternion): Float {
		val relative = parent.inv() * child
		return atan2(
			2f * (relative.w * relative.y + relative.z * relative.x),
			1f - 2f * (relative.x * relative.x + relative.y * relative.y),
		) *
			(180f / Math.PI.toFloat())
	}

	private fun normalizeDeg(deg: Float): Float {
		var d = deg
		while (d > 180f) d -= 360f
		while (d < -180f) d += 360f
		return d
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
			// Stay Aligned skips any tracker whose `isImu()` is false, and that
			// requires a declared IMU type. With `imuType = null` -- which is what
			// every other replay harness uses, since nothing else cares -- it
			// silently adjusts nothing at all, and the corrected run comes back
			// byte-identical to the uncorrected one. Worth knowing about: a tracker
			// that reaches the server without an IMU type gets no yaw correction and
			// nothing says so.
			imuType = if (isHmd) null else IMUType.BMI270,
			allowReset = !isHmd,
			allowMounting = !isHmd,
			isHmd = isHmd,
			trackRotDirection = false,
		)
		tracker.status = TrackerStatus.OK
		return tracker
	}
}
