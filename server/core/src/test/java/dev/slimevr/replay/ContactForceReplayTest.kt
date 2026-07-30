package dev.slimevr.replay

import dev.slimevr.config.LocalizerConfig
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.processor.skeleton.MovementStates
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.test.assertTrue

/**
 * Issue #6 proposal (2): damp translation estimates the contacts could not have
 * produced.
 *
 * The physics itself is unit-tested in `ContactForceLimitTest`. This file
 * measures what the constraint does when wired into `Localizer` and run over
 * whole sequences, and pins the measurement that decided *not* to build it on
 * the existing ground-reaction-force machinery.
 */
class ContactForceReplayTest {

	private val rateHz = 100f

	/**
	 * ## Why proposal (2) does not build on `LegTweaksBuffer`'s force code
	 *
	 * The audit on #6 pointed at `getPressurePrediction`, `findForceVectors` and
	 * `detectOutsideForces` and noted that they compare a centre-of-mass term
	 * against `GRAVITY` without ever dividing by the timestep -- so the term is
	 * in metres and is being compared against 9.81 m/s^2. That is accurate, and
	 * the obvious conclusion, that dividing by the timestep would repair the
	 * test, is wrong.
	 *
	 * This measures the quantity the repair would produce. `centerOfMassDelta`
	 * and `centerOfMassDeltaChange` are a first and second difference of the
	 * mass-weighted centre of mass, so the correctly-scaled acceleration is
	 * `centerOfMassDeltaChange * rate^2`. On **noise-free** synthetic motion it
	 * still spikes far above `FORCE_ERROR_TOLERANCE_SQR`'s 4 m/s^2 -- the spikes
	 * are phase boundaries in the profile and leg-correction snaps, not motion.
	 *
	 * Adding real sensor noise only makes this worse: a second difference
	 * amplifies position noise by `1/dt^2`, which is 1e4 at this rate, so a
	 * millimetre of jitter is 10 m/s^2 on its own.
	 *
	 * The scale error is therefore load-bearing. Correcting it without also
	 * smoothing the input makes `detectOutsideForces` fire on nearly every
	 * moving frame, which drops `isStanding`, collapses the foot-lock pressure
	 * scalars to their fallback, and takes flight detection from 34/39 frames to
	 * 0/39 -- measured, by making exactly that change. Smoothing it is a change
	 * to how foot locking behaves and belongs to whoever owns that (#4, #5), not
	 * to this issue.
	 */
	@Test
	fun theRawSecondDifferenceIsTooNoisyToUseAsADynamicsSignal() {
		val tolerance = 4.0f // FORCE_ERROR_TOLERANCE_SQR is this, squared.
		var anyExceeded = false

		for (name in SyntheticMotion.allNames) {
			val magnitudes = comAccelerationMagnitudes(name)
			if (magnitudes.isEmpty()) continue
			val sorted = magnitudes.sorted()
			val p50 = sorted[sorted.size / 2]
			val p99 = sorted[(sorted.size - 1) * 99 / 100]
			println(
				"%-14s p50=%8.2f p99=%8.2f max=%8.2f m/s^2".format(name, p50, p99, sorted.last()),
			)
			if (sorted.last() > tolerance) anyExceeded = true
		}

		assertTrue(
			anyExceeded,
			"the correctly-scaled centre-of-mass acceleration stayed inside the $tolerance m/s^2 " +
				"force tolerance on every sequence, which would mean dividing by the timestep is " +
				"safe after all and LegTweaksBuffer's force code should simply be fixed",
		)
	}

	/**
	 * Standing still is plausible, and has to be, or the constraint would fire
	 * on the one case there is no doubt about.
	 */
	@Test
	fun standingStillIsNeverCorrected() {
		val run = replay("stand", contactForceLimits = true)
		println("stand: ${run.corrections} corrections over ${run.frames} frames")
		assertTrue(
			run.corrections == 0,
			"the contact-force limit fired ${run.corrections} times on a stationary body, so it is " +
				"correcting noise rather than implausible motion",
		)
	}

	/**
	 * The constraint has to be quiet on ordinary motion too. A gate that fires
	 * on most frames of walking is not enforcing physics, it is low-pass
	 * filtering the estimate, and it would be better described and tuned as one.
	 */
	@Test
	fun ordinaryMotionIsMostlyLeftAlone() {
		for (name in listOf("squat", "walk-in-place", "lean")) {
			val run = replay(name, contactForceLimits = true)
			val fraction = run.corrections.toFloat() / run.frames
			println("%-14s %3d / %3d frames corrected (%.1f%%)".format(name, run.corrections, run.frames, fraction * 100f))
			assertTrue(
				fraction < 0.25f,
				"$name had ${run.corrections} of ${run.frames} frames corrected, which is too many " +
					"for a plausibility gate -- at that rate it is acting as a filter on ordinary " +
					"motion rather than rejecting the impossible",
			)
		}
	}

	/**
	 * The case the constraint exists for, and the one the issue names directly:
	 * "bodies that accelerate horizontally with no foot on the ground".
	 *
	 * With nothing touching the floor the friction cone collapses to a point, so
	 * horizontal acceleration must be zero and horizontal velocity must be
	 * whatever it was at takeoff. Measured as the drift rate over unanchored
	 * frames, which is the comparison `LocalizerReplayTest` establishes as the
	 * fair one -- total drift is not, because a configuration that wrongly
	 * believes the feet are down reports less of it for that reason alone.
	 */
	@Test
	fun horizontalDriftWhileAirborneIsReduced() {
		val without = replay("jump", contactForceLimits = false)
		val with = replay("jump", contactForceLimits = true)

		println("jump, unanchored horizontal drift rate:")
		println("  without contact-force limits: %.4f m/s (%d unanchored frames)".format(without.driftRate, without.unanchored))
		println("  with contact-force limits:    %.4f m/s (%d unanchored frames)".format(with.driftRate, with.unanchored))
		println("  corrections applied: ${with.corrections} of ${with.frames} frames")

		assertTrue(
			with.unanchored > 0 && without.unanchored > 0,
			"no unanchored frames in either run, so there is no airborne phase to measure over " +
				"and this test is vacuous",
		)
		assertTrue(
			with.driftRate <= without.driftRate,
			"the contact-force limit made unanchored horizontal drift worse " +
				"(%.4f vs %.4f m/s), which is the opposite of what constraining horizontal ".format(with.driftRate, without.driftRate) +
				"acceleration to zero during free fall should do",
		)
	}

	/**
	 * The constraint must not disturb the vertical channel, which the ballistic
	 * arc from #25 owns. Free fall is exactly on the cone boundary -- required
	 * contact force zero -- so a correctly-implemented gate is an identity on a
	 * body doing nothing but falling.
	 */
	@Test
	fun theBallisticArcIsUndisturbed() {
		val arcOnly = replay("jump", contactForceLimits = false, ballistic = true)
		val both = replay("jump", contactForceLimits = true, ballistic = true)

		println("jump apex height, ballistic arc:")
		println("  arc alone:            %.4f m".format(arcOnly.apex))
		println("  arc + contact limits: %.4f m".format(both.apex))

		assertTrue(
			abs(both.apex - arcOnly.apex) < 0.01f,
			"adding the contact-force limit moved the ballistic apex from ${arcOnly.apex}m to " +
				"${both.apex}m; free fall sits exactly on the cone boundary, so the gate should " +
				"be an identity there",
		)
	}

	// #region harness

	private class Run(
		val frames: Int,
		val corrections: Int,
		val unanchored: Int,
		val driftRate: Float,
		val apex: Float,
	)

	/** Correctly-scaled CoM acceleration, the quantity a units fix would produce. */
	private fun comAccelerationMagnitudes(name: String): List<Float> {
		val out = mutableListOf<Float>()
		drive(name) { hpm, _ ->
			val buffer = hpm.skeleton.legTweaks.bufferHead
			val rate = buffer.getTimeDelta()
			if (rate > 0f) out.add((buffer.centerOfMassDeltaChange * (rate * rate)).len())
		}
		return out.drop(6)
	}

	private fun replay(
		name: String,
		contactForceLimits: Boolean,
		ballistic: Boolean = false,
	): Run {
		var frames = 0
		var unanchored = 0
		var path = 0f
		var apex = 0f
		var previousHorizontal: Vector3? = null
		var standingY: Float? = null
		var corrections = 0

		drive(name, contactForceLimits, ballistic) { hpm, _ ->
			val localizer = hpm.skeleton.localizer
			val com = hpm.skeleton.legTweaks.bufferHead.centerOfMass
			frames++
			corrections = localizer.contactForceCorrections

			if (standingY == null && frames > (0.2f * rateHz).toInt()) standingY = com.y
			standingY?.let { apex = maxOf(apex, com.y - it) }

			if (localizer.currentWorldReference == MovementStates.FOLLOW_COM) {
				unanchored++
				val horizontal = Vector3(com.x, 0f, com.z)
				previousHorizontal?.let { path += (horizontal - it).len() }
				previousHorizontal = horizontal
			} else {
				previousHorizontal = null
			}
		}

		val driftRate = if (unanchored > 1) path / (unanchored / rateHz) else 0f
		return Run(frames, corrections, unanchored, driftRate, apex)
	}

	/**
	 * Drives a synthetic sequence through a full skeleton with `Localizer` live.
	 *
	 * The lead-in matches `LocalizerReplayTest`: `Localizer` suppresses upward
	 * CoM acceleration until the feet have anchored for `WARMUP_FRAMES`, so a
	 * sequence started cold measures the warmup rather than the motion.
	 */
	private fun drive(
		name: String,
		contactForceLimits: Boolean = false,
		ballistic: Boolean = false,
		onFrame: (HumanPoseManager, Float) -> Unit,
	) {
		val head = mkTracker(0, TrackerPosition.HEAD)
		val chest = mkTracker(1, TrackerPosition.CHEST)
		val hip = mkTracker(2, TrackerPosition.HIP)
		val leftThigh = mkTracker(3, TrackerPosition.LEFT_UPPER_LEG)
		val leftCalf = mkTracker(4, TrackerPosition.LEFT_LOWER_LEG)
		val rightThigh = mkTracker(5, TrackerPosition.RIGHT_UPPER_LEG)
		val rightCalf = mkTracker(6, TrackerPosition.RIGHT_LOWER_LEG)
		val trackers = listOf(head, chest, hip, leftThigh, leftCalf, rightThigh, rightCalf)

		val hpm = HumanPoseManager(trackers)
		hpm.skeleton.hasKneeTrackers = true
		hpm.setLegTweaksEnabled(true)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, true)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, true)
		hpm.setToggle(SkeletonConfigToggles.SELF_LOCALIZATION, true)

		val clock = FixedStepClock(1f / rateHz)
		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		hpm.skeleton.localizer.config = LocalizerConfig().apply {
			useContactForceLimits = contactForceLimits
			useBallisticFlight = ballistic
		}

		val leadInFrames = (1.5f * rateHz).toInt()
		val bodyFrames = (3f * rateHz).toInt()

		for (i in 0 until leadInFrames + bodyFrames) {
			val sinceLeadIn = i - leadInFrames
			val t = if (sinceLeadIn < 0) 0f else sinceLeadIn / rateHz
			val frame = SyntheticMotion.at(name, t)
			clock.advance()

			head.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(frame.leftCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			setWorldAcceleration(hip, frame.torsoAccel)
			setWorldAcceleration(chest, frame.torsoAccel)

			hpm.update()
			if (sinceLeadIn >= 0) onFrame(hpm, t)
		}
	}

	private fun setWorldAcceleration(tracker: Tracker, worldAccel: Vector3) {
		val rot = tracker.getRawRotation()
		tracker.setAcceleration(rot.inv().sandwich(worldAccel))
	}

	private fun mkTracker(id: Int, position: TrackerPosition): Tracker {
		val tracker = Tracker(
			device = null,
			id = id,
			name = position.name,
			trackerPosition = position,
			trackerNum = 0,
			hasPosition = false,
			hasRotation = true,
			isComputed = false,
			imuType = null,
			allowReset = true,
			allowMounting = true,
			isHmd = false,
			trackRotDirection = false,
		)
		tracker.status = TrackerStatus.OK
		return tracker
	}

	// #endregion
}
