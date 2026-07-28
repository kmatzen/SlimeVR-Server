package dev.slimevr.unit

import dev.slimevr.tracking.processor.KinematicHeading
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
 * Tests for magnetometer-free relative heading from a hinge joint.
 *
 * Each case builds two segment orientations from a known ground-truth heading
 * error, so the expected answer is exact. The interesting cases are not the
 * ones where it works -- they are the ones where the geometry does not
 * constrain heading at all, and the estimator has to say so rather than return
 * a confident wrong number.
 */
class KinematicHeadingTests {

	/** Heading is rotation about Y: SlimeVR's world frame is Y-up. */
	private fun yaw(radians: Double): Quaternion = Quaternion(
		cos(radians / 2).toFloat(),
		0f,
		sin(radians / 2).toFloat(),
		0f,
	)

	private fun aboutX(radians: Double): Quaternion = Quaternion(
		cos(radians / 2).toFloat(),
		sin(radians / 2).toFloat(),
		0f,
		0f,
	)

	private fun deg(d: Double) = d * PI / 180.0

	/** Wraps to (-pi, pi]. */
	private fun wrap(a: Double): Double {
		var x = a
		while (x > PI) x -= 2 * PI
		while (x <= -PI) x += 2 * PI
		return x
	}

	@Test
	fun recoversRelativeHeadingFromAHorizontalHinge() {
		// Knee-like: hinge axis along local X, which stays horizontal as the
		// joint flexes about it. Segment 2's tracker has drifted 20 degrees.
		val est = KinematicHeading()
		val trueError = deg(20.0)
		val axis = Vector3(1f, 0f, 0f)

		for (i in 0 until 100) {
			// Flex the joint through a range, so the segments are not simply
			// identical to each other.
			val flex = deg(-40.0 + 0.8 * i)
			val r1 = aboutX(flex * 0.3)
			val r2 = aboutX(flex)
			// Segment 2's estimated orientation carries the heading error.
			val r2Drifted = yaw(trueError).times(r2)
			est.addSample(r1, axis, r2Drifted, axis)
		}

		assertTrue(est.hasEstimate, "a horizontal hinge should be observable")
		val err = abs(wrap(est.relativeHeadingRad + trueError))
		assertTrue(
			err < deg(1.0),
			"expected about ${-trueError} rad, got ${est.relativeHeadingRad} (err $err)",
		)
	}

	@Test
	fun reportsNoEstimateWhenTheAxisIsVertical() {
		// The failure this whole design exists to avoid. A rotation about world
		// vertical does not move a vertical vector, so a vertical hinge axis
		// carries no heading information whatsoever -- and an estimator that
		// did not notice would return a confident number built from noise.
		val est = KinematicHeading()
		val axis = Vector3(0f, 1f, 0f) // Y is up, so this axis is vertical

		for (i in 0 until 200) {
			val r = yaw(deg(i.toDouble()))
			est.addSample(Quaternion.IDENTITY, axis, r, axis)
		}

		assertTrue(est.sampleCount >= 200)
		assertTrue(
			est.observability < 0.05,
			"a vertical axis should be unobservable, got ${est.observability}",
		)
		assertFalse(est.hasEstimate, "must decline to give an estimate")
	}

	@Test
	fun declinesBeforeEnoughSamples() {
		val est = KinematicHeading()
		val axis = Vector3(1f, 0f, 0f)
		repeat(5) {
			est.addSample(Quaternion.IDENTITY, axis, yaw(deg(10.0)), axis)
		}
		assertFalse(est.hasEstimate, "five samples is not an estimate")
	}

	@Test
	fun detectsDisagreementWhenTheHingeAssumptionIsWrong() {
		// A joint treated as a hinge but actually moving as a ball joint
		// produces inconsistent per-sample answers. The mean would look
		// plausible; the concentration is what reveals it.
		val est = KinematicHeading()
		val axis = Vector3(1f, 0f, 0f)

		for (i in 0 until 200) {
			// Segment 2 rotates about the axis in a way segment 1 does not
			// share, so the "axis" is not actually common to both.
			val bogus = yaw(deg((i * 37 % 360).toDouble()))
			est.addSample(Quaternion.IDENTITY, axis, bogus, axis)
		}

		assertTrue(
			est.concentration < 0.5,
			"scattered samples should show low concentration, got ${est.concentration}",
		)
		assertFalse(est.hasEstimate)
	}

	@Test
	fun averagesCorrectlyAcrossThePiWrap() {
		// A plain arithmetic mean of angles is wrong near +/-pi and silently
		// so: averaging 179 and -179 gives 0 instead of 180.
		val est = KinematicHeading()
		val trueError = deg(179.0)
		val axis = Vector3(1f, 0f, 0f)

		for (i in 0 until 100) {
			val flex = deg(-30.0 + 0.6 * i)
			val r1 = aboutX(flex * 0.3)
			val r2 = aboutX(flex)
			est.addSample(r1, axis, yaw(trueError).times(r2), axis)
		}

		assertTrue(est.hasEstimate)
		val err = abs(wrap(est.relativeHeadingRad + trueError))
		assertTrue(err < deg(2.0), "wrap-safe mean expected, err was $err rad")
	}

	@Test
	fun zeroErrorGivesZeroHeading() {
		val est = KinematicHeading()
		val axis = Vector3(1f, 0f, 0f)
		for (i in 0 until 100) {
			val flex = deg(-40.0 + 0.8 * i)
			est.addSample(aboutX(flex * 0.3), axis, aboutX(flex), axis)
		}
		assertTrue(est.hasEstimate)
		assertTrue(
			abs(wrap(est.relativeHeadingRad)) < deg(0.5),
			"two agreeing trackers should show no relative heading, got ${est.relativeHeadingRad}",
		)
	}

	@Test
	fun resetClearsState() {
		val est = KinematicHeading()
		val axis = Vector3(1f, 0f, 0f)
		repeat(50) { est.addSample(Quaternion.IDENTITY, axis, yaw(deg(30.0)), axis) }
		est.reset()
		assertFalse(est.hasEstimate)
		assertTrue(est.sampleCount == 0)
		assertTrue(est.observability == 0.0)
	}
}
