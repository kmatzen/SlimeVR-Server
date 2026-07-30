package dev.slimevr.config

class LegTweaksConfig {
	var correctionStrength = 0.3f
	var alwaysUseFloorclip = false

	/**
	 * After correcting the legs, project them back onto a pose the skeleton can
	 * actually hold.
	 *
	 * The corrections move joint positions independently, which changes segment
	 * lengths -- measured at up to 0.16 m, 11% of a segment, on a squat. This
	 * re-solves the leg in joint space for the same ankle target, so the lengths
	 * are preserved by construction.
	 *
	 * Off by default. Issue #4 asks for work in this direction to be evaluated
	 * against the existing heuristics rather than assumed better, and the
	 * existing behaviour is what users have been getting. See
	 * `JointSpaceProjection`.
	 */
	var projectToJointSpace = false
}
