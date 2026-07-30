package dev.slimevr.autobone.leastsquares

import dev.slimevr.autobone.errors.BodyProportionError
import dev.slimevr.autobone.errors.FootHeightOffsetError
import dev.slimevr.autobone.errors.HeightError
import dev.slimevr.autobone.errors.OffsetSlideError
import dev.slimevr.autobone.errors.PositionError
import dev.slimevr.autobone.errors.PositionOffsetError
import dev.slimevr.autobone.errors.SlideError

/**
 * The error terms, in one place.
 *
 * `AutoBone` holds these as seven separate fields. They are all stateless --
 * every one of them reads the step and returns a number -- so sharing a set
 * between the existing optimiser and the least-squares path costs nothing and
 * guarantees both are measuring the same thing. If a term ever gains state,
 * this is the type that has to grow a copy method, which is a better place for
 * that to surface than seven fields on a 700-line class.
 */
class AutoBoneErrorSet(
	val slideError: SlideError = SlideError(),
	val offsetSlideError: OffsetSlideError = OffsetSlideError(),
	val footHeightOffsetError: FootHeightOffsetError = FootHeightOffsetError(),
	val bodyProportionError: BodyProportionError = BodyProportionError(),
	val heightError: HeightError = HeightError(),
	val positionError: PositionError = PositionError(),
	val positionOffsetError: PositionOffsetError = PositionOffsetError(),
)
