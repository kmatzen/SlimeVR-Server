package dev.slimevr.tracking.processor.skeleton

import io.github.axisangles.ktmath.Vector3

/**
 * The centre-of-mass trajectory of a body with both feet off the floor.
 *
 * ## Why this is not integration
 *
 * While a foot is planted it is a zero-velocity anchor, and translation follows
 * from it directly. In flight there is no anchor, and the only remaining
 * observation is the accelerometer -- which has to be integrated twice to give a
 * position, so any bias in it grows quadratically and there is nothing left to
 * correct against.
 *
 * But flight is the one phase where the answer is known in advance. The only
 * force on an airborne body is gravity, so the centre of mass follows a parabola
 * whose shape is fixed and whose only free parameters are the position and
 * velocity it launched with. That turns "integrate a noisy signal for half a
 * second" into "measure two vectors at one instant" -- and that instant is
 * takeoff, the moment a foot was still planted and the measurement was at its
 * most reliable.
 *
 * The accelerometer is not read at all here. That is the point. `Localizer`'s
 * vertical channel integrates the torso accelerometer plus a hand-tuned constant
 * downward pull, under a clamp that suppresses upward acceleration until the
 * feet have been anchored for a hundred consecutive frames. None of those three
 * pieces has anything to say about a body in free fall, and the parabola
 * replaces all of them.
 *
 * ## What this is not
 *
 * It is causal and it is a prediction. Issue #6 notes that a ballistic *fit*
 * needs the flight phase to be over before it can be fit, which is fine for
 * offline analysis and useless for live VR. So this does not fit -- it
 * extrapolates from the takeoff measurement, with zero latency, and is wrong by
 * however wrong that one measurement was. The error shows up as a mismatch at
 * landing; see [landingErrorM].
 *
 * A genuine refinement would re-estimate the launch velocity mid-flight, but
 * there is nothing to re-estimate it *against*: during flight the body is
 * unobserved apart from the accelerometer this deliberately ignores. The
 * remaining honest improvement is to make the takeoff measurement better, which
 * is where a learned contact classifier (#5) would help -- it would say when
 * takeoff happened, and with what confidence, instead of leaving it to a
 * threshold that is least reliable at exactly that transition.
 */
class BallisticFlight(
	/**
	 * Downward acceleration, m/s^2. Matches [LegTweaksBuffer.GRAVITY] rather
	 * than being a separate tunable -- this is a physical constant and the whole
	 * argument for the arc is that it is not something to fit.
	 */
	private val gravity: Float = -LegTweaksBuffer.GRAVITY.y,
) {

	/** True between [takeoff] and [land]. */
	var inFlight = false
		private set

	/** Seconds since takeoff. */
	var flightTimeSec = 0f
		private set

	/** Position the arc launched from. */
	var launchPosition: Vector3 = Vector3.NULL
		private set

	/** Velocity the arc launched with. */
	var launchVelocity: Vector3 = Vector3.NULL
		private set

	/** Where the arc currently says the centre of mass is. */
	var position: Vector3 = Vector3.NULL
		private set

	/**
	 * Predicted flight duration: the time for the arc to return to the height it
	 * launched from.
	 *
	 * Only meaningful for an upward launch. A body that leaves the floor moving
	 * downward -- which contact detection will occasionally report, for instance
	 * when a foot lock is lost mid-step -- has no such time, and this is zero
	 * for it.
	 */
	val predictedFlightSec: Float
		get() = if (launchVelocity.y > 0f) 2f * launchVelocity.y / gravity else 0f

	fun takeoff(position: Vector3, velocity: Vector3) {
		launchPosition = position
		launchVelocity = velocity
		this.position = position
		flightTimeSec = 0f
		inFlight = true
	}

	/**
	 * Advance by [dtSec] and return the predicted position.
	 *
	 * Evaluated in closed form from the launch state rather than stepped, so the
	 * result depends only on the elapsed flight time and not on how that time was
	 * divided into frames. A dropped or doubled frame shifts nothing.
	 */
	fun advance(dtSec: Float): Vector3 {
		if (!inFlight) return position
		if (dtSec > 0f) flightTimeSec += dtSec

		val t = flightTimeSec
		position = Vector3(
			launchPosition.x + launchVelocity.x * t,
			launchPosition.y + launchVelocity.y * t - 0.5f * gravity * t * t,
			launchPosition.z + launchVelocity.z * t,
		)
		return position
	}

	/**
	 * How far the arc's final position was from where the body was actually
	 * observed to land, given the observed landing position.
	 *
	 * This is the arc's own error report, and it is available with no ground
	 * truth: takeoff and landing happen on the same floor, so the vertical
	 * component is a direct measurement of how wrong the launch velocity was.
	 * Positive means the arc finished above the observed landing.
	 */
	fun landingErrorM(observedLanding: Vector3): Vector3 = position - observedLanding

	fun land() {
		inFlight = false
		flightTimeSec = 0f
	}
}
