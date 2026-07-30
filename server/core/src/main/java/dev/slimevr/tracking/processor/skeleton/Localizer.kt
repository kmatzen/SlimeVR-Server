package dev.slimevr.tracking.processor.skeleton

import com.jme3.math.FastMath
import dev.slimevr.config.LocalizerConfig
import dev.slimevr.tracking.trackers.Tracker
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3

/* Handles localizing the skeleton in 3d space when no 6dof device is present.
 * This is accomplished by using the foot state calculated by legtweaks. This of course
 * has the problem of the true position and predicted location drifting apart over time.
 * Jumping is quite unreliable
 */

enum class MovementStates {
	LEFT_LOCKED,
	RIGHT_LOCKED,
	NONE_LOCKED,
	FOLLOW_FOOT,
	FOLLOW_COM,
	FOLLOW_SITTING,
}

class Localizer(humanSkeleton: HumanSkeleton) {
	// hyper parameters
	companion object {
		private const val WARMUP_FRAMES = 100 // ~0.1 seconds
		private const val MAX_FOOT_PERCENTAGE = 50.0f
		private const val MAX_ACCEL_UP = 2.0f
		private const val SITTING_KNEE_THRESHOLD = 1.1f
		private const val SITTING_EARLY = 1000f
		private const val VELOCITY_SAMPLE_RATE: Long = 100000000 // 10ms
		private const val CONSTANT_ACCELERATION: Float = 2.0f
	}

	private val skeleton: HumanSkeleton = humanSkeleton
	private val legTweaks: LegTweaks = skeleton.legTweaks
	private var bufCur: LegTweaksBuffer = legTweaks.bufferHead

	// Placeholder until update() replaces it with bufCur.parent; built on the
	// same clock as the rest of the chain so it cannot introduce a cross-clock
	// interval if it is ever differenced.
	private var bufPrev: LegTweaksBuffer = LegTweaksBuffer(legTweaks.clock)

	// state variables
	private var enabled: Boolean = false
	private var targetFoot: Vector3 = Vector3.NULL
	private var currentCOM: Vector3 = Vector3.NULL
	private var targetCOM: Vector3 = Vector3.NULL
	private var targetHip: Vector3 = Vector3.NULL
	private var comVelocity: Vector3 = Vector3.NULL
	private var comAccel: Vector3 = Vector3.NULL
	private var plantedFoot = MovementStates.LEFT_LOCKED
	private var worldReference = MovementStates.FOLLOW_FOOT
	private var uncorrectedFloor = 0.0f - LegTweaks.FLOOR_CALIBRATION_OFFSET
	private var floor = 0.0f
	private var warmupFrames = 0
	private var comFrames = 0
	private var footFrames = 0
	private var sittingFrames = 0

	// travel from different sources
	private var footTravel: Vector3 = Vector3.NULL
	private var comTravel: Vector3 = Vector3.NULL
	private var sittingTravel: Vector3 = Vector3.NULL

	var config: LocalizerConfig = LocalizerConfig()

	/** The ballistic arc, live only while [config] enables it and feet are free. */
	private val ballistic = BallisticFlight()

	/**
	 * Centre-of-mass velocity as measured from CoM positions, before the
	 * vertical channel is overwritten.
	 *
	 * [getCOMVelocity] computes this and then replaces its `y` with a value
	 * integrated from the torso accelerometer, so the measured vertical velocity
	 * is discarded on every frame. The ballistic path wants it: at takeoff, while
	 * a foot is still planted and therefore acting as a zero-velocity anchor, the
	 * kinematically-derived CoM velocity is the best launch-velocity measurement
	 * available, and it is the one quantity the whole arc depends on.
	 */
	private var kinematicComVelocity: Vector3 = Vector3.NULL

	/**
	 * Last frame's [comVelocity], for measuring the acceleration the estimate
	 * implies.
	 *
	 * Held separately rather than differenced from the buffer chain because what
	 * has to be checked is the acceleration of the *estimate*, which includes
	 * the substituted vertical channel, not the acceleration of the kinematic
	 * CoM the buffer records.
	 */
	private var previousComVelocity: Vector3 = Vector3.NULL

	/** Set when an airborne stretch was rejected as implausibly long. */
	private var flightAbandoned = false

	/**
	 * Called on each takeoff, for tests and diagnostics.
	 *
	 * The launch velocity determines the entire arc and is read at a single
	 * instant, so it is the one number worth watching. It is not recoverable
	 * afterwards -- by the time a test sees the pose, several takeoffs may have
	 * come and gone.
	 */
	var onTakeoff: ((BallisticFlight) -> Unit)? = null

	/**
	 * Called when an arc ends, with the arc and the position the body was
	 * actually observed at.
	 *
	 * This is the arc's own error report and it needs no ground truth: a jump
	 * ends on the floor it started from, so the gap between where the arc
	 * finished and where the body was seen to land measures how wrong the launch
	 * velocity was. It is the only feedback the method gets, and the quantity a
	 * deployment would log to know whether the arc is working at all.
	 */
	var onLanding: ((BallisticFlight, Vector3) -> Unit)? = null

	fun getEnabled(): Boolean = enabled

	/**
	 * Which source the last [update] took its translation from.
	 *
	 * Exposed for replay tests. A test that measures the flight phase has to be
	 * able to confirm the flight phase was actually entered -- if contact
	 * detection never reports both feet off the floor, every metric about
	 * flight is measured over an empty set and passes vacuously.
	 */
	val currentWorldReference: MovementStates
		get() = worldReference

	fun setEnabled(enabled: Boolean) {
		this.enabled = enabled
		legTweaks.setLocalizerMode(enabled)
	}

	fun update() {
		if (!enabled) {
			return
		}

		// if there is a 6dof device just use it
		if (skeleton.headTracker != null && skeleton.headTracker!!.hasPosition) {
			return
		}

		// set the acceleration of the com for this frame
		comAccel = getTorsoAccel()

		if (warmupFrames < WARMUP_FRAMES) {
			comVelocity = Vector3.NULL
			previousComVelocity = Vector3.NULL
			targetFoot = Vector3.NULL
		}
		warmupFrames++

		// set the buffers for easy access
		bufCur = legTweaks.bufferHead
		if (bufCur.parent == null) {
			return
		}
		bufPrev = bufCur.parent!!

		var finalTravel: Vector3

		// get the movement of the skeleton by foot travel
		footTravel = getPlantedFootTravel()

		// get the movement of the skeleton by the previous COM velocity
		comTravel = getCOMTravel()

		sittingTravel = computeSittingTravel()

		// get the metric that this frame should rely on
		worldReference = getWorldReference()

		// update the final travel vector
		if (worldReference == MovementStates.FOLLOW_FOOT || warmupFrames < WARMUP_FRAMES) {
			finalTravel = footTravel
		} else if (worldReference == MovementStates.FOLLOW_COM) {
			finalTravel = comTravel
		} else if (worldReference == MovementStates.FOLLOW_SITTING) {
			finalTravel = sittingTravel
			if (sittingFrames < SITTING_EARLY) {
				finalTravel = footTravel
			}
		} else {
			finalTravel = Vector3.NULL
		}

		// update the y value
		if (worldReference != MovementStates.FOLLOW_SITTING || sittingFrames < SITTING_EARLY) {
			finalTravel = Vector3(
				finalTravel.x,
				comTravel.y,
				finalTravel.z,
			)
		}

		updateSkeletonPos(finalTravel)
	}

	// resets to the starting position
	fun reset() {
		if (!enabled) return

		skeleton.headBone.setPosition(Vector3.NULL)
		comVelocity = Vector3.NULL
		// Left stale, the first frame after a reset would difference a fresh
		// zero against the last velocity from before it and report an
		// acceleration of thousands of m/s^2.
		previousComVelocity = Vector3.NULL
		contactForceCorrections = 0

		// when localizing without a 6 dof device we choose the floor level
		// 0 happens to be an easy number to use
		legTweaks.setLocalizerMode(enabled)
		floor = 0.0f
		uncorrectedFloor = 0.0f - LegTweaks.FLOOR_CALIBRATION_OFFSET
		warmupFrames = 0

		// A reset teleports the skeleton back to the origin, so an arc launched
		// before it describes a body that no longer exists. Carrying it across
		// would apply the old launch position to the new frame.
		ballistic.land()
	}

	private fun getPlantedFoot(): MovementStates {
		// if locked in legtweaks it's the locked foot
		if (bufCur.leftLegState == LegTweaksBuffer.LOCKED) return MovementStates.LEFT_LOCKED
		if (bufCur.rightLegState == LegTweaksBuffer.LOCKED) return MovementStates.RIGHT_LOCKED

		// if the state is not locked, use the numerical state to determine a
		// foot to follow
		val leftNumericalState = bufCur.leftLegNumericalState
		val rightNumericalState = bufCur.rightLegNumericalState

		return if (leftNumericalState < rightNumericalState &&
			leftNumericalState < MAX_FOOT_PERCENTAGE &&
			bufCur.leftFootAcceleration.y < MAX_ACCEL_UP
		) {
			return MovementStates.LEFT_LOCKED
		} else if (rightNumericalState < leftNumericalState &&
			rightNumericalState < MAX_FOOT_PERCENTAGE &&
			bufCur.rightFootAcceleration.y < MAX_ACCEL_UP
		) {
			MovementStates.RIGHT_LOCKED
		} else {
			MovementStates.NONE_LOCKED
		}
	}

	// check if the foot that is planted is actually planted
	private fun getWorldReference(): MovementStates {
		// check for sitting position
		if (isUserSitting()) {
			return MovementStates.FOLLOW_SITTING
		}

		// if the foot is not on the ground, use the COM
		return if (!isFootOnGround()) {
			MovementStates.FOLLOW_COM
		} else {
			MovementStates.FOLLOW_FOOT
		}
	}

	// get the foot or feet that are planted
	// also sets the planted foot, foot init, and target pos variables
	private fun getPlantedFootTravel(): Vector3 {
		// get the foot that is planted
		val foot: MovementStates = getPlantedFoot()

		if (foot == MovementStates.LEFT_LOCKED) {
			val footLoc: Vector3 = bufCur.leftFootPosition
			updateTargetPos(footLoc, foot)
			return getFootTravel(footLoc)
		} else if (foot == MovementStates.RIGHT_LOCKED) {
			val footLoc: Vector3 = bufCur.rightFootPosition
			updateTargetPos(footLoc, foot)
			return getFootTravel(footLoc)
		}
		return Vector3.NULL
	}

	// get the travel of a foot over a frame
	private fun getFootTravel(loc: Vector3): Vector3 = loc - targetFoot

	// update the target position of the foot
	private fun updateTargetPos(loc: Vector3, foot: MovementStates) {
		if (foot == plantedFoot) {
			if (worldReference == MovementStates.FOLLOW_COM) {
				targetFoot = loc
			}
		} else {
			targetFoot = loc
			plantedFoot = foot
		}
	}

	// get the sitting travel (emulates hip lock)
	private fun computeSittingTravel(): Vector3 {
		val hip = skeleton.computedHipTracker?.position ?: Vector3.NULL

		// get the distance to move the waist to the target waist
		val dist: Vector3 = hip - targetHip

		val lowTracker = getLowestTracker()

		if (lowTracker != null) {
			if (lowTracker.position.y < uncorrectedFloor) {
				targetHip = Vector3(targetHip.x, targetHip.y + (uncorrectedFloor - lowTracker.position.y), targetHip.z)
			}
		}

		// if the world reference is not sitting update the target waist
		if (worldReference != MovementStates.FOLLOW_SITTING || sittingFrames < SITTING_EARLY) {
			targetHip = hip
		}
		return dist
	}

	// returns the travel of the COM from its last position
	private fun getCOMTravel(): Vector3 {
		// update COM attributes
		updateCOMAttributes()
		return bufCur.centerOfMass - targetCOM
	}

	// get the movement of the COM based on the last velocity
	private fun updateCOMAttributes() {
		getCOMVelocity()
		updateTargetCOM()

		// update how long the COM has been the reference and how long the foot
		// has been
		comFrames = if (worldReference == MovementStates.FOLLOW_COM) comFrames + 1 else 0
		footFrames = if (worldReference == MovementStates.FOLLOW_FOOT) footFrames + 1 else 0
		sittingFrames = if (worldReference == MovementStates.FOLLOW_SITTING) sittingFrames + 1 else 0
	}

	// gets the position the COM should be at based on the velocity of the com and
	// the location of the floor
	private fun updateTargetCOM() {
		val grounded = worldReference == MovementStates.FOLLOW_FOOT ||
			worldReference == MovementStates.FOLLOW_SITTING

		// if not in COM tracking mode, just use the current COM
		if (grounded) {
			targetCOM = bufCur.centerOfMass
		} else {
			currentCOM = targetCOM
		}

		if (config.useBallisticFlight) {
			advanceTargetCOMBallistic(grounded)
		} else {
			targetCOM += (comVelocity / bufCur.getTimeDelta())
		}

		val lowTracker = getLowestTracker()

		// update the target COM and velocity to reflect this new distance
		if (lowTracker != null) {
			if (lowTracker.position.y < uncorrectedFloor) {
				targetCOM = Vector3(targetCOM.x, targetCOM.y + (uncorrectedFloor - lowTracker.position.y), targetCOM.z)
				comVelocity = Vector3(comVelocity.x, 0.0f, comVelocity.z)

				// The floor is a hard constraint and the arc is only a
				// prediction, so if the two disagree the arc is what gives way:
				// something has touched down, whatever the leg states say. Ending
				// flight here rather than letting the clamp fight the arc matters
				// because the arc recomputes targetCOM from its launch state every
				// frame and would otherwise discard this correction on the next
				// one -- the floor would stop being enforced for the whole of
				// flight, silently.
				//
				// It also makes contact-with-the-floor a landing detector in its
				// own right, which is a useful second opinion at the transition
				// the leg-state thresholds are least reliable at.
				if (ballistic.inFlight) {
					onLanding?.invoke(ballistic, targetCOM)
					ballistic.land()
				}
			}
		}
	}

	/**
	 * Ballistic state, for tests and diagnostics.
	 *
	 * The arc's accuracy is not directly observable in the pose -- it shows up as
	 * a mismatch at landing, one number per jump -- so a test that wants to know
	 * whether the arc was launched with a sane velocity has to be able to ask.
	 */
	val ballisticState: BallisticFlight
		get() = ballistic

	/**
	 * Advance [targetCOM] along a ballistic arc while the feet are free.
	 *
	 * Replaces the accelerometer integration for the airborne case only. On the
	 * ground this defers to the existing path unchanged, so walking, standing and
	 * sitting are untouched by the flag -- the arc exists for the one phase where
	 * there is no contact to anchor to.
	 *
	 * Note this runs before [worldReference] is recomputed for the frame, so
	 * `grounded` describes the previous frame. That is pre-existing ordering, and
	 * it means takeoff is recognised one frame after the feet actually leave the
	 * floor. At 100 Hz that is 10 ms of launch velocity already integrated by the
	 * old path, which is small; it is called out because it biases the launch
	 * measurement in a knowable direction rather than randomly.
	 */
	private fun advanceTargetCOMBallistic(grounded: Boolean) {
		val rate = bufCur.getTimeDelta()
		// getTimeDelta() is 1/dt and returns 0 for a frame carrying no interval.
		val dt = if (rate > 0f) 1f / rate else 0f

		if (grounded) {
			if (ballistic.inFlight) {
				onLanding?.invoke(ballistic, bufCur.centerOfMass)
				ballistic.land()
			}
			// Feet are back down, so whatever made the last airborne stretch
			// implausible is over and the next one gets a fresh arc.
			flightAbandoned = false
			targetCOM += (comVelocity / rate)
			return
		}

		// A "flight" longer than any real jump is not a jump. Contact detection
		// reporting no planted foot is not the same claim as the body being
		// airborne, and the cases that break it -- leaning on furniture,
		// kneeling, sitting on the floor -- are common in VR. Hand those back to
		// the old path rather than continuing an arc that is by then describing
		// a body that fell several metres.
		//
		// Latched rather than tested per frame. Without the latch, abandoning
		// flight clears `inFlight`, the next airborne frame sees no arc and
		// launches a new one, and the guard trips again -- so an over-long
		// airborne stretch turns into a rapid relaunch cycle that is neither the
		// arc nor the fallback. It has to stay abandoned until the feet return.
		if (flightAbandoned) {
			targetCOM += (comVelocity / rate)
			return
		}

		if (!ballistic.inFlight) {
			ballistic.takeoff(targetCOM, getTakeoffVelocity())
			onTakeoff?.invoke(ballistic)
		}

		if (ballistic.flightTimeSec + dt > config.ballisticMaxFlightSec) {
			ballistic.land()
			flightAbandoned = true
			targetCOM += (comVelocity / rate)
			return
		}

		// All three axes. Constant horizontal velocity through flight follows from
		// "the only force is gravity" exactly as the vertical parabola does, so
		// this is the complete statement rather than a vertical special case.
		//
		// The vertical win is large and directly measured. The horizontal claim is
		// weaker and worth stating precisely: this sequence has no clean
		// horizontal ground truth, because its legs bend in the sagittal plane and
		// that genuinely moves the centre of mass fore and aft -- "a purely
		// vertical jump" is true of the trajectory, not of the pose. So horizontal
		// residual here is not attributable, and the claim is only that it is not
		// made worse.
		//
		// Checked, not assumed. Total horizontal drift does rise, but for a
		// legitimate reason: getting the height right lifts the feet, flight is
		// detected across far more of its true duration, and a planted foot is
		// what pins horizontal translation -- the old path's smaller total was
		// bought by believing the feet were down for half of flight. Compared per
		// unanchored frame, which removes that, the arc's horizontal drift rate is
		// indistinguishable from the path it replaces -- the two agree to within a
		// fraction of a percent, which is the "not made worse" this can support.
		targetCOM = ballistic.advance(dt)
	}

	/**
	 * Launch velocity of the centre of mass, measured over a short window ending
	 * at takeoff.
	 *
	 * Measured from CoM positions rather than taken from [comVelocity], for two
	 * reasons. The vertical component of `comVelocity` is not a measurement at
	 * all -- it is the accelerometer integration this path exists to avoid. And
	 * its horizontal components are averaged over `VELOCITY_SAMPLE_RATE`, 100 ms,
	 * which spans the entire push-off: the average velocity over a window ending
	 * at takeoff is roughly half the velocity at takeoff, and using it would
	 * launch every arc at about half the true speed.
	 *
	 * Falls back to [kinematicComVelocity] if the buffer chain is too short to
	 * measure over, which happens only in the first frames after a reset.
	 */
	private fun getTakeoffVelocity(): Vector3 {
		var buf = bufCur
		var frames = 0
		while (frames < config.ballisticTakeoffWindowFrames) {
			buf = buf.parent ?: break
			frames++
		}

		if (frames == 0) return kinematicComVelocity

		val elapsedSec = (bufCur.timeOfFrame - buf.timeOfFrame) / LegTweaksBuffer.NS_CONVERT
		if (elapsedSec <= 0f) return kinematicComVelocity

		val shortWindow = (bufCur.centerOfMass - buf.centerOfMass) / elapsedSec

		// A different measurement window per axis, because the bias a window
		// costs is proportional to the acceleration in that axis, and the two
		// axes are nothing alike at takeoff.
		//
		// Vertically the push-off is the largest acceleration in the whole motion
		// -- of order 25 m/s^2 -- so averaging over any appreciable window reads
		// far below the launch speed. The existing 100 ms window would read about
		// half of it. That channel needs the short window and pays the noise.
		//
		// Horizontally there is no push-off. Whatever horizontal acceleration a
		// jump has is small next to the vertical, so a long window costs little
		// bias and buys a lot: the short window's horizontal reading is dominated
		// by the legs swinging, which moves the centre of mass sideways relative
		// to the body at exactly the instant the launch is read. Extrapolating
		// that across the flight was measurably worse than not using the arc
		// horizontally at all.
		//
		// So: short window for the axis that is accelerating hard, the existing
		// long window for the axes that are not.
		return Vector3(kinematicComVelocity.x, shortWindow.y, kinematicComVelocity.z)
	}

	// get the velocity of the COM
	private fun getCOMVelocity(): Vector3 {
		val comY = comVelocity.y

		var buf = bufCur
		val timeStart: Long = buf.timeOfFrame
		var timeEnd = timeStart - VELOCITY_SAMPLE_RATE
		val comPosStart: Vector3 = buf.centerOfMass

		// get the buffer that occurred VELOCITY_SAMPLE_RATE ago in time
		while (buf.timeOfFrame > timeEnd && buf.parent != null) {
			buf = buf.parent!!
		}

		val comPosEnd: Vector3 = buf.centerOfMass
		timeEnd = buf.timeOfFrame

		// calculate the velocity
		comVelocity = (comPosEnd - comPosStart) / ((timeEnd - timeStart) / LegTweaksBuffer.NS_CONVERT)

		// Keep the measured vertical component before the accelerometer channel
		// below overwrites it. Note this window is VELOCITY_SAMPLE_RATE wide --
		// 100 ms, despite the comment on it saying 10 ms -- which is too long to
		// read a launch velocity from, so the ballistic path measures its own
		// over a short window rather than reusing this. See
		// getTakeoffVelocity().
		kinematicComVelocity = comVelocity

		// if the feet have been the reference for a short amount of time nullify any upwards acceleration to prevent flying away
		if (footFrames < WARMUP_FRAMES) {
			comAccel = Vector3(
				comAccel.x,
				FastMath.clamp(comAccel.y, -9999.0f, 0.0f),
				comAccel.z,
			)
		}

		// constantly pull the skeleton down a little to account for acceleration
		// inaccuracy
		val gravity = comAccel.y - CONSTANT_ACCELERATION

		// add the acceleration of gravity
		comVelocity = Vector3(
			comVelocity.x,
			comY + (gravity / bufCur.getTimeDelta()),
			comVelocity.z,
		)

		if (config.useContactForceLimits) {
			comVelocity = limitToContactForces(comVelocity)
		}
		previousComVelocity = comVelocity

		return comVelocity
	}

	/**
	 * Damp [velocity] so the acceleration it implies is one the floor could have
	 * produced.
	 *
	 * Applied at the end of [getCOMVelocity], after the vertical channel has been
	 * substituted, because the quantity that has to be plausible is the one that
	 * actually drives translation -- constraining an intermediate would leave the
	 * accelerometer free to put back whatever was removed.
	 *
	 * The correction is applied to the *change* in velocity rather than to the
	 * velocity itself. A constant velocity needs no force at all, so there is
	 * nothing implausible about a large one; what physics constrains is how fast
	 * it is allowed to change. Damping the velocity directly would fight steady
	 * motion, which is the common case and the one this must not touch.
	 */
	private fun limitToContactForces(velocity: Vector3): Vector3 {
		val rate = bufCur.getTimeDelta()
		// No interval, so no acceleration is implied and there is nothing to
		// check. Also avoids the multiply-then-divide by zero below.
		if (rate <= 0f) return velocity

		// During warmup `comVelocity` and `previousComVelocity` are both reset to
		// zero at the top of every frame, so differencing them yields the whole
		// velocity divided by one timestep -- an apparent acceleration a hundred
		// times too large, on every warmup frame, on a body that may well be
		// standing perfectly still. The estimate is held stationary by
		// construction during warmup, so there is nothing here worth
		// constraining anyway.
		if (warmupFrames < WARMUP_FRAMES) return velocity

		val implied = (velocity - previousComVelocity) * rate

		// The previous frame's world reference, not a foot height. `worldReference`
		// is recomputed after this runs, so this is last frame's -- the same
		// one-frame lag the ballistic path already carries and for the same
		// reason. It is used in preference to isFootOnGround() because that is a
		// bare inequality on foot height against a calibrated floor, and floor
		// clipping routinely leaves a planted foot a few millimetres above it; a
		// standing body would intermittently read as unsupported and be told it
		// could not be holding itself up.
		val footOnGround = worldReference == MovementStates.FOLLOW_FOOT ||
			worldReference == MovementStates.FOLLOW_SITTING

		val allowed = ContactForceLimit.limitHorizontal(
			implied,
			footOnGround,
			config.contactFriction,
			config.contactForceToleranceMPerSec2,
		)
		if (allowed == implied) return velocity

		contactForceCorrections++
		return previousComVelocity + (allowed / rate)
	}

	/**
	 * How many frames the contact-force limit has corrected since the last
	 * reset, for tests and diagnostics.
	 *
	 * A constraint that never fires is doing nothing and a constraint that fires
	 * on most frames is not a constraint but a filter. Neither is visible in the
	 * pose, so the count has to be readable directly.
	 */
	var contactForceCorrections: Int = 0
		private set

	// returns true if either foot is below 0.0
	private fun isFootOnGround(): Boolean = (
		bufCur.leftFootPosition.y <= floor ||
			bufCur.rightFootPosition.y <= floor
		)

	// returns the tracker closest to or the furthest in the ground
	private fun getLowestTracker(): Tracker? {
		val trackerList = arrayOf(
			skeleton.computedHeadTracker,
			skeleton.computedChestTracker,
			skeleton.computedHipTracker,
			skeleton.computedLeftElbowTracker,
			skeleton.computedRightElbowTracker,
			skeleton.computedLeftHandTracker,
			skeleton.computedRightHandTracker,
			skeleton.computedLeftKneeTracker,
			skeleton.computedRightKneeTracker,
			skeleton.computedLeftFootTracker,
			skeleton.computedRightFootTracker,
		)

		var minVal = trackerList[0]?.position?.y
		var retVal: Tracker? = trackerList[0]
		for (tracker in trackerList) {
			if (tracker == null) {
				continue
			}

			if (tracker.position.y < minVal!!) {
				minVal = tracker.position.y
				retVal = tracker
			}
		}

		return retVal
	}

	// returns true if the user is likely sitting
	// (assumes the floor is flat at 0.0)
	private fun isUserSitting(): Boolean {
		// based on the waist to knee vector decide if the user is sitting or
		// standing (ie, if the user is sitting the vector will be pointing off
		// to the side for both feet)
		var leftKnee: Vector3 = bufCur.leftKneePosition
		var rightKnee: Vector3 = bufCur.rightKneePosition
		val hip: Vector3 = skeleton.computedHipTracker?.position ?: Vector3.NULL
		leftKnee = hip - leftKnee
		rightKnee = hip - rightKnee

		// if the y component of the vectors is small then the user is probably
		// sitting
		var left = false
		var right = false
		if (leftKnee.y * SITTING_KNEE_THRESHOLD < leftKnee.x + leftKnee.z) {
			left = true
		}
		if (rightKnee.y * SITTING_KNEE_THRESHOLD < rightKnee.x + rightKnee.z) {
			right = true
		}
		return !bufCur.isStanding || (left && right)
	}

	/**
	 * Acceleration of the highest-priority available torso tracker, in world
	 * space and with gravity already removed.
	 *
	 * This used to be written as an average: it accumulated into a running sum,
	 * counted contributors, and divided by the count. But the body was an
	 * `if / else if / else if` chain, so exactly one tracker ever contributed
	 * and the count was never anything but 0 or 1 -- the division never divided.
	 * It read as "combine the torso trackers" and behaved as "take the first one
	 * that exists", which is a real preference order (waist sits nearest the
	 * body's centre of mass, hip next, chest last) and not an average at all.
	 *
	 * Written as the selection it always was. Averaging the torso trackers may
	 * well be the better estimate -- they are rigidly related and averaging
	 * would cut noise -- but that is a change to what the vertical channel
	 * integrates, and it belongs with the measurement of that channel rather
	 * than smuggled in under a rename. Noted on issue #6.
	 */
	private fun getTorsoAccel(): Vector3 {
		val torso = skeleton.waistTracker
			?: skeleton.hipTracker
			?: skeleton.chestTracker
			?: return Vector3.NULL
		return torso.getAcceleration()
	}

	// update the head position and rotation
	private fun updateSkeletonPos(travel: Vector3) {
		val rot = skeleton.headTracker?.getRotation() ?: Quaternion.IDENTITY
		val temp = skeleton.headBone.getPosition() - travel

		skeleton.headBone.setPosition(temp)
		skeleton.headBone.setRotation(rot)
	}
}
