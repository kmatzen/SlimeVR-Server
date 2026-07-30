package dev.slimevr.unit

import com.google.flatbuffers.FlatBufferBuilder
import org.junit.jupiter.api.Test
import solarxr_protocol.rpc.SkeletonPart
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * Wire-level behaviour of the optional `sigma` field on `SkeletonPart`.
 *
 * The distinction this pins is the whole reason the field is optional rather
 * than defaulted: **absent means "no error model", not "zero uncertainty"**. A
 * consumer has to be able to tell an estimate nobody put error bars on from one
 * measured to be exact, and rendering the first as "± 0" would be a confident
 * claim nobody made.
 *
 * The generated setter is `builder.addFloat(slot, sigma, 0f)`, which skips
 * writing when the value equals the default. So there is one edge worth knowing
 * about and pinning rather than discovering later: a sigma of *exactly* zero is
 * indistinguishable on the wire from an absent one. That is harmless here --
 * a real solve never produces exactly zero once the input carries any noise --
 * but it is a property of the encoding, not an accident of this test.
 */
class SkeletonPartUncertaintyTests {

	private fun encode(bone: Int, value: Float, sigma: Float?): SkeletonPart {
		val fbb = FlatBufferBuilder(64)
		SkeletonPart.startSkeletonPart(fbb)
		SkeletonPart.addBone(fbb, bone)
		SkeletonPart.addValue(fbb, value)
		sigma?.let { SkeletonPart.addSigma(fbb, it) }
		fbb.finish(SkeletonPart.endSkeletonPart(fbb))
		return SkeletonPart.getRootAsSkeletonPart(fbb.dataBuffer())
	}

	@Test
	fun anEstimateWithoutAnErrorModelReportsNoSigma() {
		val part = encode(bone = 3, value = 0.42f, sigma = null)

		assertEquals(3, part.bone())
		assertEquals(0.42f, part.value())
		assertFalse(
			part.hasSigma(),
			"a part built without addSigma reported one; consumers would render an " +
				"uncertainty nobody measured",
		)
	}

	@Test
	fun aReportedSigmaSurvivesTheRoundTrip() {
		val part = encode(bone = 3, value = 0.42f, sigma = 0.013f)

		assertTrue(part.hasSigma(), "sigma was written but did not come back")
		assertEquals(0.013f, part.sigma())
	}

	/**
	 * Documents the encoding edge rather than asserting it is desirable.
	 *
	 * If this ever needs to change, the fix is a force-added field in the
	 * schema, not a workaround in the caller.
	 */
	@Test
	fun aSigmaOfExactlyZeroIsIndistinguishableFromAbsent() {
		val part = encode(bone = 3, value = 0.42f, sigma = 0f)

		assertFalse(
			part.hasSigma(),
			"the generated setter skips values equal to the default, so this is " +
				"expected -- if it now writes, the schema changed and the 'absent " +
				"means unknown' contract should be re-checked",
		)
	}

	/** Old readers must keep working against messages that carry the new field. */
	@Test
	fun theExistingFieldsAreUnaffectedByTheNewOne() {
		val without = encode(bone = 7, value = 0.31f, sigma = null)
		val with = encode(bone = 7, value = 0.31f, sigma = 0.02f)

		assertEquals(without.bone(), with.bone())
		assertEquals(without.value(), with.value())
	}
}
