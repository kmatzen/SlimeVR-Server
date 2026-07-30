package dev.slimevr.replay

import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.udp.IMUType
import org.junit.jupiter.api.Test
import java.io.File
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

/**
 * Tests for the `.meta` sidecar parser.
 *
 * These matter more than they look. While the corpus is empty, every validation
 * rule in [CorpusRecording] is unreachable from [CorpusReplayTest] -- the first
 * time any of it would run is the day someone adds a recording, which is
 * exactly the wrong moment to discover that the check meant to catch their
 * mistake is itself broken.
 */
class CorpusMetadataTests {

	private val complete = """
		# a comment
		rate_hz     = 100
		description = 1 Hz in-place stepping
		captured    = 2026-08-14
		capturer    = A. Person
		consent     = Agreed to redistribution under the repository's licence
		trackers    = 7 - head, chest, hip, both legs
	""".trimIndent()

	private fun parse(text: String) = CorpusRecording.parse("test", File("test.pfr"), text)

	// #region yaw-correction fidelity
	//
	// Two independent ways a replayed recording silently runs with Stay Aligned
	// switched off. Both are recorded here because both are invisible: the replay
	// completes, every metric is produced, and the yaw correction the recording
	// was captured to exercise never ran.

	@Test
	fun aSidecarCanDeclareItsImuType() {
		val recording = parse("$complete\nimu_type = BMI270")
		assertEquals(IMUType.BMI270, recording.imuType)
	}

	/**
	 * Absent by default, and that is a real default rather than a neutral one.
	 *
	 * `TrackerFrames.toTracker()` builds a tracker with no IMU type, so
	 * `Tracker.isImu()` is false and `AdjustTrackerYaw.adjust` returns before
	 * doing anything. A recording that does not declare its IMU replays with no
	 * yaw correction at all.
	 */
	@Test
	fun aSidecarWithoutAnImuTypeReportsNone() {
		assertEquals(null, parse(complete).imuType)
	}

	/**
	 * A typo must fail rather than be ignored.
	 *
	 * Ignoring it does not degrade gracefully -- it disables yaw correction for
	 * the whole recording, which is precisely the failure the field exists to
	 * prevent, arrived at by a different route.
	 */
	@Test
	fun anUnrecognisedImuTypeIsRejected() {
		val error = assertFailsWith<IllegalArgumentException> {
			parse("$complete\nimu_type = BMI720")
		}
		assertTrue(
			error.message!!.contains("BMI720"),
			"the error should name the value that was not understood: ${error.message}",
		)
	}

	@Test
	fun aSidecarCanDeclareTheRelaxedPoseItWasCapturedWith() {
		val recording = parse(
			"$complete\n" +
				"stay_aligned.standing.enabled = true\n" +
				"stay_aligned.standing.upper_leg_deg = 3.5\n" +
				"stay_aligned.sitting.enabled = false",
		)

		val config = recording.stayAligned!!
		assertTrue(config.standingRelaxedPose.enabled)
		assertEquals(3.5f, config.standingRelaxedPose.upperLegAngleInDeg)
		assertTrue(!config.sittingRelaxedPose.enabled)
	}

	/**
	 * The other silent path: with every relaxed pose disabled,
	 * `RelaxedPose.forPose` returns null and `adjustMovingTracker` returns on it,
	 * so a standing, moving player gets no centring force. A default config has
	 * them all disabled, which is what a recording that says nothing gets.
	 */
	@Test
	fun aSidecarWithoutARelaxedPoseReportsNone() {
		assertEquals(null, parse(complete).stayAligned)
	}

	@Test
	fun anUnknownRelaxedPoseFieldIsRejected() {
		assertFailsWith<IllegalArgumentException> {
			parse("$complete\nstay_aligned.standing.knee_deg = 3")
		}
		assertFailsWith<IllegalArgumentException> {
			parse("$complete\nstay_aligned.kneeling.enabled = true")
		}
	}
	// #endregion

	@Test
	fun parsesACompleteSidecar() {
		val recording = parse(complete)

		assertEquals(100f, recording.rateHz)
		assertEquals("1 Hz in-place stepping", recording.description)
		assertEquals("A. Person", recording.fields["capturer"])
		assertTrue(recording.offsets.isEmpty())
	}

	@Test
	fun commentsAndBlankLinesAreIgnored() {
		val recording = parse("$complete\n\n# trailing comment\n   \n")
		assertEquals(100f, recording.rateHz)
	}

	@Test
	fun valuesMayContainEqualsSigns() {
		// Split on the *first* '=' only. Free-text fields like `notes` are the
		// obvious place for a second one, and silently truncating a note is
		// worse than rejecting it.
		val recording = parse("$complete\nnotes = ratio = 2:1")
		assertEquals("ratio = 2:1", recording.fields["notes"])
	}

	@Test
	fun everyRequiredFieldIsEnforced() {
		for (field in CorpusRecording.REQUIRED) {
			val without = complete.lines()
				.filterNot { it.substringBefore('=').trim() == field }
				.joinToString("\n")

			val error = assertFailsWith<IllegalArgumentException>(
				"dropping '$field' was accepted; the requirement is not enforced",
			) { parse(without) }

			assertTrue(
				error.message!!.contains(field),
				"the error for a missing '$field' does not name it: ${error.message}",
			)
		}
	}

	@Test
	fun anEmptyRequiredValueIsNotAValue() {
		val error = assertFailsWith<IllegalArgumentException> {
			parse(complete.replace("capturer    = A. Person", "capturer    ="))
		}
		assertTrue(error.message!!.contains("capturer"))
	}

	/**
	 * A sidecar copied from the README example and left with its placeholders
	 * intact is worse than no sidecar, because it reads as provenance.
	 */
	@Test
	fun templatePlaceholdersAreRejected() {
		for (placeholder in listOf("TODO", "tbd", "Unknown", "?")) {
			assertFailsWith<IllegalArgumentException>(
				"'$placeholder' was accepted as a capturer",
			) { parse(complete.replace("A. Person", placeholder)) }
		}
	}

	@Test
	fun rateMustBeAPositiveNumber() {
		for (bad in listOf("0", "-100", "fast", "")) {
			assertFailsWith<IllegalArgumentException>(
				"rate_hz '$bad' was accepted",
			) { parse(complete.replace("rate_hz     = 100", "rate_hz     = $bad")) }
		}
	}

	@Test
	fun offsetsAreParsedAndResolvedToConfigKeys() {
		val recording = parse("$complete\noffset.UPPER_LEG = 0.42\noffset.LOWER_LEG = 0.44")

		assertEquals(
			mapOf(
				SkeletonConfigOffsets.UPPER_LEG to 0.42f,
				SkeletonConfigOffsets.LOWER_LEG to 0.44f,
			),
			recording.offsets,
		)
	}

	/**
	 * A misspelled offset must fail rather than be dropped. Silently ignoring
	 * it would replay the recording against default proportions while its
	 * sidecar claims otherwise -- the metrics would be wrong and the file would
	 * look right.
	 */
	@Test
	fun anUnknownOffsetIsAnError() {
		val error = assertFailsWith<IllegalArgumentException> {
			parse("$complete\noffset.UPPER_LEGG = 0.42")
		}
		assertTrue(error.message!!.contains("UPPER_LEGG"))
	}

	@Test
	fun aNonNumericOffsetIsAnError() {
		assertFailsWith<IllegalArgumentException> {
			parse("$complete\noffset.UPPER_LEG = tall")
		}
	}

	@Test
	fun aDuplicateKeyIsAnError() {
		val error = assertFailsWith<IllegalArgumentException> {
			parse("$complete\ncapturer = Someone Else")
		}
		assertTrue(error.message!!.contains("duplicate"))
	}

	@Test
	fun aLineWithoutASeparatorIsAnError() {
		val error = assertFailsWith<IllegalArgumentException> {
			parse("$complete\nthis line has no equals sign")
		}
		// The line number makes the message actionable on a long sidecar.
		assertTrue(error.message!!.contains("test.meta:8"), error.message!!)
	}
}
