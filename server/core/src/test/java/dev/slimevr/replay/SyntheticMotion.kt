package dev.slimevr.replay

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Deterministic tracker motion for replay tests.
 *
 * The server has a full `.pfr` record/replay implementation
 * (`dev.slimevr.poseframeformat`), which is the right input for regression
 * testing against real captures. It needs recordings, though, and there are
 * none in the repository. Synthetic motion fills the gap: it needs no captured
 * data, it is byte-reproducible, and it can isolate one behaviour at a time in
 * a way a real recording never can.
 *
 * These sequences are anatomically approximate. That is fine and deliberate --
 * a regression baseline needs repeatability, not realism. Realism comes from
 * the `.pfr` corpus once recordings exist; the two are complementary.
 */
object SyntheticMotion {

	/** Tracker rotations for one frame, plus the headset height. */
	data class Frame(
		val chest: Quaternion,
		val hip: Quaternion,
		val leftThigh: Quaternion,
		val leftCalf: Quaternion,
		val rightThigh: Quaternion,
		val rightCalf: Quaternion,
		/** Headset height as a fraction of standing height. */
		val headHeightFraction: Float,
		/**
		 * Whether each foot is intended to be on the floor this frame.
		 *
		 * Ground truth for contact detection, and it is ground truth in the
		 * strong sense: these sequences are *defined* by their joint-angle
		 * functions, so which foot is planted is a property of the definition
		 * rather than something inferred from a signal. Nothing derives these
		 * from foot height, velocity, or any other observable -- if they were
		 * derived, comparing a detector against them would be comparing it
		 * against a slightly different detector.
		 *
		 * See [dev.slimevr.replay.ContactDetectionTest].
		 */
		val leftFootContact: Boolean = true,
		val rightFootContact: Boolean = true,
		/**
		 * World-frame linear acceleration of the torso, m/s^2, with gravity
		 * already removed -- which is what `Tracker.getAcceleration()` returns
		 * and therefore what `Localizer.getTorsoAccel()` consumes. A body
		 * standing still reads zero here; a body in free fall reads -9.81 on y.
		 *
		 * Only the sequences that drive [Localizer] set this. The others leave
		 * it at zero, which is what the HMD-driven harnesses have always
		 * implicitly fed.
		 */
		val torsoAccel: Vector3 = Vector3.NULL,
		/**
		 * Ground-truth vertical offset of the centre of mass from the standing
		 * pose, in metres. This is the quantity [torsoAccel] is the exact
		 * second derivative of, which is what makes it usable as ground truth:
		 * an estimator handed [torsoAccel] and asked for position has no excuse
		 * for not reproducing this.
		 */
		val comHeightM: Float = 0f,
		/** True while both feet are off the floor. */
		val inFlight: Boolean = false,
	)

	/**
	 * Motions covered by the committed regression baseline in
	 * `replay-baseline.txt`.
	 *
	 * `jump` is deliberately not in here. Every baselined motion is replayed by
	 * four harnesses, three of which assert things that a body with both feet
	 * off the floor legitimately violates, and none of which run [Localizer] at
	 * all -- they give the HMD a position, and `Localizer.update()` returns
	 * immediately when the head tracker has one. Baselining the jump across all
	 * of them is worth doing, but it is a change to those harnesses rather than
	 * to this file, so it is not bundled in here.
	 */
	val names = listOf("stand", "squat", "walk-in-place", "lean")

	/** Every motion [at] accepts, including ones outside the baselined set. */
	val allNames = names + "jump"

	/**
	 * Rotation of [angleRad] about the given axis, which must be unit length.
	 * Built directly rather than via Euler angles so there is no dependence on
	 * an axis-order convention.
	 */
	fun axisAngle(x: Float, y: Float, z: Float, angleRad: Float): Quaternion {
		val h = angleRad / 2f
		val s = sin(h)
		return Quaternion(cos(h), x * s, y * s, z * s)
	}

	fun sequence(name: String, frames: Int, rateHz: Float): List<Frame> {
		require(frames > 0) { "frames must be positive" }
		require(rateHz > 0f) { "rateHz must be positive" }

		return (0 until frames).map { i -> at(name, i / rateHz) }
	}

	/**
	 * The motion at an arbitrary instant, not just on a frame boundary.
	 *
	 * These sequences are closed-form functions of time, so there is no reason
	 * to restrict sampling to a grid -- and [dev.slimevr.replay.TimeSkewReplayTest]
	 * needs off-grid samples specifically. Simulating clock skew means each
	 * tracker sampling the *same* motion at its *own* instant, and if those
	 * instants land on frame boundaries the interpolation under test is never
	 * exercised: every lookup would be an exact hit.
	 *
	 * [tSec] may be negative, which is what a tracker whose samples are delayed
	 * reports during the first few frames.
	 */
	fun at(name: String, tSec: Float): Frame {
		require(name in allNames) { "unknown motion '$name' (known: $allNames)" }
		return when (name) {
			"stand" -> stand()
			"squat" -> squat(tSec)
			"walk-in-place" -> walkInPlace(tSec)
			"lean" -> lean(tSec)
			"jump" -> jump(tSec)
			else -> stand()
		}
	}

	/**
	 * Motions whose headset height does not vary with time.
	 *
	 * The headset is a position source with no rotation history, so time
	 * alignment cannot interpolate it -- see [dev.slimevr.replay.TimeSkewReplayTest].
	 * On these sequences that limitation is invisible, because there is nothing
	 * about the head to get wrong.
	 */
	val staticHeadHeight = listOf("stand", "walk-in-place", "lean")

	private fun stand() = Frame(
		chest = Quaternion.IDENTITY,
		hip = Quaternion.IDENTITY,
		leftThigh = Quaternion.IDENTITY,
		leftCalf = Quaternion.IDENTITY,
		rightThigh = Quaternion.IDENTITY,
		rightCalf = Quaternion.IDENTITY,
		headHeightFraction = 1f,
	)

	/**
	 * Symmetric knee bend at 0.25 Hz. Both feet stay planted throughout, so any
	 * horizontal ankle movement the solver produces is foot slide -- this is
	 * the cleanest case for that metric.
	 */
	private fun squat(t: Float): Frame {
		val phase = 0.5f - 0.5f * cos(2f * PI.toFloat() * 0.25f * t)
		val thigh = -deg(55f) * phase
		val calf = deg(70f) * phase
		return Frame(
			chest = axisAngle(1f, 0f, 0f, deg(10f) * phase),
			hip = axisAngle(1f, 0f, 0f, deg(20f) * phase),
			leftThigh = axisAngle(1f, 0f, 0f, thigh),
			leftCalf = axisAngle(1f, 0f, 0f, thigh + calf),
			rightThigh = axisAngle(1f, 0f, 0f, thigh),
			rightCalf = axisAngle(1f, 0f, 0f, thigh + calf),
			headHeightFraction = 1f - 0.28f * phase,
		)
	}

	/**
	 * Alternating leg lift at 1 Hz. Each foot leaves and returns to the floor,
	 * which is what exercises contact detection and the skating correction.
	 *
	 * The lift phase is `max(0, sin)`, so each foot spends exactly half of each
	 * cycle on the floor and the two are in antiphase. Liftoff and touchdown are
	 * the sine's zero crossings, which makes the contact intervals exact: a foot
	 * is planted precisely when its phase is zero. That is what
	 * [Frame.leftFootContact] reports, and it is why this sequence rather than
	 * the others is the one contact detection is measured on -- the rest keep
	 * both feet down throughout and only test that a detector does not invent
	 * liftoffs.
	 */
	private fun walkInPlace(t: Float): Frame {
		val w = 2f * PI.toFloat() * 1.0f * t
		val leftPhase = maxOf(0f, sin(w))
		val rightPhase = maxOf(0f, sin(w + PI.toFloat()))
		// Both labels come from the same `sin(w)` rather than each from its own
		// phase variable. The right leg's lift is driven by `sin(w + PI)`, which
		// is -sin(w) in exact arithmetic but not in float: PI is not exactly
		// representable, so at a zero crossing both phases can round marginally
		// positive and the labels would claim both feet are airborne at once --
		// which this motion never does. Deriving both from one sine makes them
		// exactly complementary. The disagreement with the rotation actually
		// applied is around 1e-7 radians of thigh angle, which is a sub-nanometre
		// difference in foot height.
		return Frame(
			leftFootContact = sin(w) <= 0f,
			rightFootContact = sin(w) >= 0f,
			chest = Quaternion.IDENTITY,
			hip = Quaternion.IDENTITY,
			leftThigh = axisAngle(1f, 0f, 0f, -deg(45f) * leftPhase),
			leftCalf = axisAngle(1f, 0f, 0f, -deg(45f) * leftPhase + deg(60f) * leftPhase),
			rightThigh = axisAngle(1f, 0f, 0f, -deg(45f) * rightPhase),
			rightCalf = axisAngle(
				1f,
				0f,
				0f,
				-deg(45f) * rightPhase + deg(60f) * rightPhase,
			),
			headHeightFraction = 1f,
		)
	}

	/** Upper-body lean with the legs straight; the feet must not move at all. */
	private fun lean(t: Float): Frame {
		val a = deg(25f) * sin(2f * PI.toFloat() * 0.2f * t)
		return Frame(
			chest = axisAngle(1f, 0f, 0f, a),
			hip = axisAngle(1f, 0f, 0f, a * 0.5f),
			leftThigh = Quaternion.IDENTITY,
			leftCalf = Quaternion.IDENTITY,
			rightThigh = Quaternion.IDENTITY,
			rightCalf = Quaternion.IDENTITY,
			headHeightFraction = 1f,
		)
	}

	// #region jump
	//
	// A single vertical jump: stand, crouch, push off, fly, absorb, stand.
	//
	// This is the target case of issue #6. `Localizer` says of itself that
	// "Jumping is quite unreliable", and the reason it is unreliable has never
	// been measured, because measuring it needs a jump and there was no jump to
	// replay. The prerequisite the issue lists for that is a `.pfr` corpus
	// containing one (#15), which needs hardware and a wearer.
	//
	// A synthetic jump does not substitute for that -- it has no IMU noise, no
	// yaw drift, no mounting error. What it does have is something the corpus
	// never will: an exact, closed-form answer. The trajectory below is defined
	// as a height profile, and the acceleration handed to the torso tracker is
	// that profile's analytic second derivative. So the input and the ground
	// truth are the same object differentiated twice, and any estimator given
	// the acceleration and asked for the height is being asked to invert an
	// operation with no noise in it. Error measured here is error the method
	// has, not error the data has.
	//
	// The two are complementary in exactly the way #15 argues: this bounds the
	// method, a recording bounds the method plus the sensor.

	/** Standard gravity, matching `LegTweaksBuffer.GRAVITY`. */
	private const val JUMP_G = 9.81f

	/** Vertical CoM speed at the instant the feet leave the floor. */
	private const val JUMP_TAKEOFF_SPEED = 1.9f

	private const val JUMP_CROUCH_START = 0.30f
	private const val JUMP_CROUCH_DURATION = 0.30f
	private const val JUMP_ABSORB_DURATION = 0.30f

	/**
	 * Standing height the [Frame.headHeightFraction] of a jump is expressed
	 * against. [Frame.comHeightM] is in metres and is the field to prefer; the
	 * fraction exists only because the HMD-driven harnesses need one, and it
	 * needs *some* height to divide by.
	 */
	const val JUMP_NOMINAL_HEIGHT_M = 1.75f

	/** Vertical CoM speed at takeoff, the quantity a ballistic arc must recover. */
	const val jumpTakeoffSpeedMPerSec = JUMP_TAKEOFF_SPEED

	val jumpTakeoffSec = JUMP_CROUCH_START + JUMP_CROUCH_DURATION

	/**
	 * How long both feet are off the floor: the flight time of a body launched
	 * at [JUMP_TAKEOFF_SPEED] and landing at the height it left from.
	 */
	val jumpFlightDurationSec = 2f * JUMP_TAKEOFF_SPEED / JUMP_G

	val jumpLandingSec = jumpTakeoffSec + jumpFlightDurationSec

	/** Peak height of the CoM above standing, `v0^2 / 2g`. */
	val jumpApexHeightM = JUMP_TAKEOFF_SPEED * JUMP_TAKEOFF_SPEED / (2f * JUMP_G)

	/** One full jump plus a little standing on each end. */
	val jumpDurationSec = jumpLandingSec + JUMP_ABSORB_DURATION + 0.30f

	/**
	 * Ground-truth vertical CoM offset from the standing pose, in metres.
	 *
	 * Four phases, each a polynomial in time, chosen so the profile and its
	 * first derivative are continuous across every boundary:
	 *
	 *  - **stand** -- flat zero.
	 *  - **crouch** -- a cubic that starts at rest, dips, and returns to
	 *    standing height with an upward velocity of exactly
	 *    [JUMP_TAKEOFF_SPEED]. That last condition is what makes takeoff a
	 *    boundary the flight phase can be continuous with rather than a
	 *    discontinuity the estimator has to guess across.
	 *  - **flight** -- the parabola. This is the phase under test.
	 *  - **absorb** -- the crouch cubic run backwards: enters at the landing
	 *    speed and settles at rest.
	 *
	 * Acceleration is *not* continuous, by design. It steps at every boundary,
	 * most sharply at takeoff, where the ground reaction force vanishes the
	 * instant the foot leaves the floor. That step is the physical event this
	 * issue is about and smoothing it would be inventing data.
	 */
	fun jumpComHeight(t: Float): Float = when {
		t < JUMP_CROUCH_START -> 0f

		t < jumpTakeoffSec -> {
			// h(s) = v0*Tc*(s^3 - s^2), so h(0)=0, h'(0)=0, h(1)=0, h'(1)=v0.
			val s = (t - JUMP_CROUCH_START) / JUMP_CROUCH_DURATION
			JUMP_TAKEOFF_SPEED * JUMP_CROUCH_DURATION * s * s * (s - 1f)
		}

		t < jumpLandingSec -> {
			val tau = t - jumpTakeoffSec
			JUMP_TAKEOFF_SPEED * tau - 0.5f * JUMP_G * tau * tau
		}

		t < jumpLandingSec + JUMP_ABSORB_DURATION -> {
			// h(s) = -v0*Tr*s*(s-1)^2, so h(0)=0, h'(0)=-v0, h(1)=0, h'(1)=0.
			val s = (t - jumpLandingSec) / JUMP_ABSORB_DURATION
			-JUMP_TAKEOFF_SPEED * JUMP_ABSORB_DURATION * s * (s - 1f) * (s - 1f)
		}

		else -> 0f
	}

	/**
	 * The exact second derivative of [jumpComHeight] -- the linear (gravity
	 * removed) vertical acceleration a perfect torso IMU would report.
	 *
	 * Kept as closed form rather than finite-differenced so the ground truth
	 * carries no discretisation error of its own: at 100 Hz a numerical second
	 * difference of the crouch cubic would be off by enough to matter against
	 * the errors being measured.
	 */
	fun jumpComAccel(t: Float): Float = when {
		t < JUMP_CROUCH_START -> 0f

		t < jumpTakeoffSec -> {
			val s = (t - JUMP_CROUCH_START) / JUMP_CROUCH_DURATION
			JUMP_TAKEOFF_SPEED * (6f * s - 2f) / JUMP_CROUCH_DURATION
		}

		// Free fall. The only force is gravity, so the linear-acceleration
		// reading is -g.
		t < jumpLandingSec -> -JUMP_G

		t < jumpLandingSec + JUMP_ABSORB_DURATION -> {
			val s = (t - jumpLandingSec) / JUMP_ABSORB_DURATION
			-JUMP_TAKEOFF_SPEED * (6f * s - 4f) / JUMP_ABSORB_DURATION
		}

		else -> 0f
	}

	/** True while both feet are off the floor. */
	fun jumpInFlight(t: Float): Boolean = t >= jumpTakeoffSec && t < jumpLandingSec

	private fun jump(t: Float): Frame {
		val h = jumpComHeight(t)
		val flight = jumpInFlight(t)

		// Knee tuck during flight. Without it, whether the feet are seen to
		// leave the floor would depend on the vertical estimate being correct,
		// which is the thing under test -- a bad estimate would keep the feet
		// planted, flight would never be detected, and the test would pass by
		// never exercising anything. The tuck lifts the ankles by pose alone.
		//
		// A smoothstep trapezoid rather than a sine bump: it reaches full tuck
		// within the first 15% of flight and holds it, where a sine spends most
		// of the phase part-way. That matters because the feet only read as off
		// the floor once the tuck has lifted them, so a slow ramp spends the
		// early and late frames of flight looking like stance and the flight
		// metrics get measured over the middle only.
		//
		// Smoothstep at both ends so the tuck arrives and leaves with zero
		// derivative; a corner here shows up as a spike in the FK-derived foot
		// velocities that `LegTweaksBuffer` thresholds on.
		//
		// The frames right at takeoff and landing still read as stance, and no
		// choice of profile fixes that -- it is contact detection being least
		// certain exactly at the transitions, which is what makes the learned
		// classifier in #5 a listed prerequisite for this work.
		val tuck = if (flight) {
			val s = (t - jumpTakeoffSec) / jumpFlightDurationSec
			smoothstep(0f, 0.15f, s) * smoothstep(1f, 0.85f, s)
		} else {
			0f
		}

		// Crouch and absorb both bend the knees, and both are phases where h is
		// below standing. Driving the bend from -h ties the pose to the height
		// profile instead of tracking the phases separately, so the legs are
		// bent exactly when the body is low. The scale converts metres of dip
		// into radians of knee angle and is approximate -- see the class note on
		// these sequences being anatomically approximate on purpose.
		val dip = if (flight) 0f else maxOf(0f, -h)
		val dipBend = dip * deg(320f)

		val thigh = -(dipBend + tuck * deg(70f))
		val knee = dipBend * 2f + tuck * deg(100f)

		return Frame(
			// The torso stays upright. A real jumper pitches forward on the
			// crouch, but pitch would rotate the accelerometer out of the world
			// frame and the point of this sequence is a clean vertical channel.
			chest = Quaternion.IDENTITY,
			hip = Quaternion.IDENTITY,
			leftThigh = axisAngle(1f, 0f, 0f, thigh),
			leftCalf = axisAngle(1f, 0f, 0f, thigh + knee),
			rightThigh = axisAngle(1f, 0f, 0f, thigh),
			rightCalf = axisAngle(1f, 0f, 0f, thigh + knee),
			headHeightFraction = 1f + h / JUMP_NOMINAL_HEIGHT_M,
			// Both feet leave together, which is what makes this a jump rather
			// than a step. Set explicitly because the field defaults to planted,
			// and a sequence that is airborne for a third of its length while
			// claiming both feet are down would be ground truth that is wrong --
			// the one thing a ground-truth label may not be.
			leftFootContact = !flight,
			rightFootContact = !flight,
			torsoAccel = Vector3(0f, jumpComAccel(t), 0f),
			comHeightM = h,
			inFlight = flight,
		)
	}
	// #endregion

	private fun deg(d: Float): Float = d * PI.toFloat() / 180f

	/**
	 * Hermite ramp from 0 at [edge0] to 1 at [edge1], flat outside.
	 * [edge1] may be below [edge0], which gives a falling ramp.
	 */
	private fun smoothstep(edge0: Float, edge1: Float, x: Float): Float {
		val s = ((x - edge0) / (edge1 - edge0)).coerceIn(0f, 1f)
		return s * s * (3f - 2f * s)
	}
}
