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
	 */
	private fun walkInPlace(t: Float): Frame {
		val w = 2f * PI.toFloat() * 1.0f * t
		val leftPhase = maxOf(0f, sin(w))
		val rightPhase = maxOf(0f, sin(w + PI.toFloat()))
		return Frame(
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
