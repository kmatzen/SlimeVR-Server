package dev.slimevr.config

/**
 * Settings for `Localizer`, the global-translation estimator used when nothing
 * supplies an absolute position.
 *
 * Everything here only affects setups with no positional head tracking.
 * `Localizer.update()` returns on its first line when the head tracker has a
 * position, so for a user in a headset none of this is in the path.
 */
class LocalizerConfig {

	// #region ballistic flight

	/**
	 * During flight, predict the centre of mass along a ballistic arc anchored
	 * at takeoff instead of integrating the torso accelerometer.
	 *
	 * Off by default. Issue #6 asks for the existing path to be kept for
	 * comparison, and until the two have been compared on real recordings the
	 * existing one is the one users have been getting.
	 */
	var useBallisticFlight = false

	/**
	 * How many frames back to look when measuring the launch velocity at
	 * takeoff.
	 *
	 * The whole arc follows from this one number, so it is the only thing in the
	 * ballistic path worth tuning, and it is a straight bias-versus-noise
	 * trade. A one-frame difference estimates the velocity at the midpoint of
	 * that frame, so on a push-off accelerating at 25 m/s^2 it reads about
	 * 0.12 m/s low at 100 Hz -- small. Widening the window cuts noise but the
	 * bias grows in proportion, because the average velocity over a window that
	 * ends at takeoff is not the velocity at takeoff. The existing code's own
	 * 100 ms velocity window would read roughly half the true launch speed.
	 *
	 * One frame is right for clean input. Real IMU data may want more, which is
	 * a decision for when there are recordings to make it on (#15).
	 */
	var ballisticTakeoffWindowFrames = 1

	/**
	 * Longest flight the ballistic arc is trusted for, in seconds.
	 *
	 * A guard against the failure mode issue #6 raises directly: VR users lean
	 * on furniture, kneel, sit on the floor and hold onto things, and a contact
	 * detector that reports no feet on the ground does not thereby mean the body
	 * is airborne. Beyond this the arc is abandoned and the frame falls back to
	 * the existing path, so a mis-detected contact degrades rather than launching
	 * the skeleton on a parabola for as long as the mis-detection lasts.
	 *
	 * 0.9 s of flight is a 1.0 m rise of the centre of mass, which is beyond
	 * standing-jump range for essentially everyone, so a longer "flight" than
	 * this is far more likely to be a detection failure than a jump.
	 */
	var ballisticMaxFlightSec = 0.9f

	// #endregion
}
