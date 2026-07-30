package dev.slimevr.replay

import io.github.axisangles.ktmath.Quaternion
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
	)

	val names = listOf("stand", "squat", "walk-in-place", "lean")

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
		require(name in names) { "unknown motion '$name' (known: $names)" }
		return when (name) {
			"stand" -> stand()
			"squat" -> squat(tSec)
			"walk-in-place" -> walkInPlace(tSec)
			"lean" -> lean(tSec)
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

	private fun deg(d: Float): Float = d * PI.toFloat() / 180f
}
