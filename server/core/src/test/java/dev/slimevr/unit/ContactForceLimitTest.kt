package dev.slimevr.unit

import dev.slimevr.tracking.processor.skeleton.ContactForceLimit
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.test.assertTrue

/**
 * The physics behind issue #6's proposal (2), tested on its own.
 *
 * `ContactForceReplayTest` measures what the constraint does to whole
 * sequences; this pins the model itself, where the answers are known in closed
 * form and do not depend on a skeleton, a recording or a solver.
 */
class ContactForceLimitTest {

	private val g = 9.81f
	private val tolerance = 1e-3f

	private fun horizontal(v: Vector3) = sqrt(v.x * v.x + v.z * v.z)

	// #region what the model says

	/**
	 * A body standing still needs the floor to hold up its whole weight. If this
	 * came out as zero the sign convention would be inverted and every other
	 * answer here would be wrong in a way that still looked self-consistent.
	 */
	@Test
	fun holdingStillRequiresSupportEqualToWeight() {
		val f = ContactForceLimit.requiredForce(Vector3.NULL)
		println("force required to hold still: $f")
		assertTrue(abs(f.y - g) < tolerance, "expected +$g upward, got ${f.y}")
		assertTrue(horizontal(f) < tolerance)
	}

	/** Free fall is the one motion that needs no contact at all. */
	@Test
	fun freeFallRequiresNoForce() {
		val f = ContactForceLimit.requiredForce(Vector3(0f, -g, 0f))
		println("force required to fall freely: $f")
		assertTrue(f.len() < tolerance, "free fall should need no contact force, got $f")
	}

	@Test
	fun standingStillIsPlausibleOnTheGroundAndNotInTheAir() {
		assertTrue(
			ContactForceLimit.isPlausible(Vector3.NULL, footOnGround = true),
			"a body standing on the floor is holding itself up, which the floor can do",
		)
		assertTrue(
			!ContactForceLimit.isPlausible(Vector3.NULL, footOnGround = false),
			"a body with nothing underneath it cannot hold itself at a constant height",
		)
	}

	/**
	 * The claim issue #6 makes in words -- "bodies that accelerate horizontally
	 * with no foot on the ground" -- falls out of the cone rather than needing
	 * its own rule: with no contact the available normal load is zero, so the
	 * cone has zero width.
	 */
	@Test
	fun nothingOnTheFloorMeansNoHorizontalAcceleration() {
		val sideways = Vector3(0.5f, -g, 0f)
		assertTrue(
			!ContactForceLimit.isPlausible(sideways, footOnGround = false),
			"a falling body accelerating sideways at 0.5 m/s^2 was reported as plausible",
		)

		val limited = ContactForceLimit.limitHorizontal(sideways, footOnGround = false)
		println("airborne sideways acceleration $sideways limited to $limited")
		assertTrue(horizontal(limited) < tolerance, "horizontal component survived free fall: $limited")
		assertTrue(
			abs(limited.y - sideways.y) < tolerance,
			"the vertical channel was altered, but it belongs to the ballistic arc",
		)
	}

	/** Sinking through the floor needs the floor to pull, which it cannot do. */
	@Test
	fun acceleratingDownwardFasterThanGravityIsImpossible() {
		assertTrue(
			!ContactForceLimit.isPlausible(Vector3(0f, -2f * g, 0f), footOnGround = true),
			"accelerating downward at 2g requires the floor to pull the body down",
		)
	}

	/** And nobody pushes off at ten times body weight. */
	@Test
	fun supportBeyondHumanCapabilityIsImpossible() {
		val absurd = Vector3(0f, 10f * g, 0f)
		assertTrue(
			!ContactForceLimit.isPlausible(absurd, footOnGround = true),
			"a 10g push-off was reported as plausible; MAX_SUPPORT_G is ${ContactForceLimit.MAX_SUPPORT_G}",
		)
	}

	// #endregion

	// #region the projection

	/**
	 * Projection has to leave plausible accelerations exactly alone, or it is a
	 * filter rather than a constraint and it would be quietly reshaping every
	 * frame it touched.
	 */
	@Test
	fun projectionIsAnIdentityOnPlausibleAccelerations() {
		val cases = listOf(
			Vector3.NULL,
			Vector3(0f, -g, 0f),
			Vector3(1f, 0f, 0.5f),
			Vector3(0f, g, 0f),
		)
		for (a in cases) {
			if (!ContactForceLimit.isPlausible(a, footOnGround = true)) continue
			val projected = ContactForceLimit.project(a, footOnGround = true)
			assertTrue(
				(projected - a).len() < tolerance,
				"$a is plausible but projection moved it to $projected",
			)
		}
	}

	/** Whatever goes in, what comes out has to satisfy the constraint. */
	@Test
	fun projectionAlwaysLandsInsideTheCone() {
		val cases = listOf(
			Vector3(50f, 0f, 0f),
			Vector3(0f, -100f, 0f),
			Vector3(-30f, 40f, 20f),
			Vector3(0f, 500f, 0f),
			Vector3(5f, -50f, -5f),
		)
		for (footOnGround in listOf(true, false)) {
			for (a in cases) {
				val projected = ContactForceLimit.project(a, footOnGround)
				assertTrue(
					ContactForceLimit.isPlausible(projected, footOnGround, tolerance = 1e-2f),
					"projecting $a with footOnGround=$footOnGround gave $projected, which is " +
						"still outside the cone",
				)
			}
		}
	}

	/** Projecting twice must change nothing the second time. */
	@Test
	fun projectionIsIdempotent() {
		val a = Vector3(40f, 30f, -10f)
		val once = ContactForceLimit.project(a, footOnGround = true)
		val twice = ContactForceLimit.project(once, footOnGround = true)
		println("$a -> $once -> $twice")
		assertTrue(
			(twice - once).len() < 1e-2f,
			"projection was not idempotent: $once then $twice",
		)
	}

	/**
	 * A body accelerating hard sideways and downward is in the polar cone --
	 * every point of the friction cone is further from it than the cone's apex
	 * is. The nearest plausible acceleration is then free fall, not some
	 * rescaled version of what was asked for.
	 */
	@Test
	fun deeplyImpossibleMotionProjectsToFreeFall() {
		val a = Vector3(5f, -100f, 0f)
		val projected = ContactForceLimit.project(a, footOnGround = true)
		println("$a projects to $projected (free fall is (0, ${-g}, 0))")
		assertTrue(
			horizontal(projected) < tolerance && abs(projected.y + g) < tolerance,
			"expected free fall, got $projected",
		)
	}

	/**
	 * More friction has to permit at least as much sideways acceleration, never
	 * less. A monotonicity check rather than a specific number, because the
	 * specific number is the thing being modelled and pinning it would only
	 * restate the implementation.
	 */
	@Test
	fun moreFrictionAllowsMoreSidewaysAcceleration() {
		var previous = -1f
		for (friction in listOf(0.1f, 0.5f, 1.0f, 2.0f)) {
			val limit = ContactForceLimit.maxHorizontalAcceleration(footOnGround = true, friction = friction)
			println("friction $friction -> max horizontal $limit m/s^2")
			assertTrue(limit >= previous, "friction $friction allowed less than the step below it")
			previous = limit
		}
		assertTrue(
			ContactForceLimit.maxHorizontalAcceleration(footOnGround = false, friction = 2.0f) == 0f,
			"friction cannot produce sideways force with nothing to press against",
		)
	}

	// #endregion
}
