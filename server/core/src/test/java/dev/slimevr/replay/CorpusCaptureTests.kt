package dev.slimevr.replay

import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.poseframeformat.CorpusCapture
import dev.slimevr.poseframeformat.CorpusCaptureConsole
import dev.slimevr.poseframeformat.CorpusMetadata
import dev.slimevr.poseframeformat.CorpusMetadataWriter
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.udp.IMUType
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.DisplayName
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertThrows
import java.io.BufferedReader
import java.io.ByteArrayOutputStream
import java.io.File
import java.io.PrintStream
import java.io.StringReader

/**
 * Tests for the capture path: the half of issue #15 that does not need
 * hardware.
 *
 * The corpus loader was built first and is thoroughly checked by
 * [CorpusMetadataTests]. What these cover is the writer on the other side of
 * the same schema, and the reason they matter is the asymmetry between the two
 * ways a sidecar can be wrong. A sidecar that is *malformed* fails loudly at
 * load. A sidecar that is merely *incomplete* -- no `imu_type`, no
 * `stay_aligned.*` -- loads fine, replays fine, produces every metric, and has
 * yaw correction switched off for its entire length.
 *
 * So the tests here are mostly about the second kind: that what the capture
 * writes is what the suite reads back, and that the fields whose absence is
 * silent are either present or reported.
 */
class CorpusCaptureTests {
	private fun derived(
		rateHz: Float = 100f,
		imuType: IMUType? = IMUType.BMI270,
		imuTypeBreakdown: String? = null,
		offsets: Map<SkeletonConfigOffsets, Float> = emptyMap(),
		stayAligned: StayAlignedConfig? = null,
	) = CorpusMetadataWriter.Derived(
		rateHz = rateHz,
		captured = "2026-08-14",
		trackers = "7 -- CHEST, HEAD, HIP, LEFT_LOWER_LEG, LEFT_UPPER_LEG, " +
			"RIGHT_LOWER_LEG, RIGHT_UPPER_LEG",
		firmware = "v0.5.0",
		imuType = imuType,
		imuTypeBreakdown = imuTypeBreakdown,
		offsets = offsets,
		stayAligned = stayAligned,
	)

	private fun attestation(
		description: String = "1 Hz in-place stepping; the main LegTweaks skating case",
		capturer: String = "A. Person <a@example.com>",
		consent: String = "I agree -- redistribution under the repository's licence",
		notes: String? = null,
	) = CorpusMetadataWriter.Attestation(description, capturer, consent, notes)

	private fun stayAlignedConfig(
		standingEnabled: Boolean = true,
		upperLeg: Float = 3.5f,
	) = StayAlignedConfig().apply {
		enabled = true
		standingRelaxedPose.enabled = standingEnabled
		standingRelaxedPose.upperLegAngleInDeg = upperLeg
		standingRelaxedPose.lowerLegAngleInDeg = 1f
		standingRelaxedPose.footAngleInDeg = -2.25f
		sittingRelaxedPose.enabled = false
		flatRelaxedPose.enabled = false
	}

	@Test
	@DisplayName("what the capture writes, the corpus loader accepts")
	fun `render round trips through parse`() {
		val text = CorpusMetadataWriter.render("walk-in-place", attestation(), derived())
		val parsed = CorpusMetadata.parse("walk-in-place", text)

		assertEquals(100f, parsed.rateHz)
		assertEquals(
			"1 Hz in-place stepping; the main LegTweaks skating case",
			parsed.description,
		)
		assertEquals("A. Person <a@example.com>", parsed.fields["capturer"])
		assertEquals("2026-08-14", parsed.fields["captured"])
		assertEquals("v0.5.0", parsed.fields["firmware"])
	}

	/**
	 * The specific gap issue #15's follow-up comment describes: without
	 * `imu_type`, `Tracker.isImu()` is false, `AdjustTrackerYaw.adjust` returns
	 * before doing anything, and the replay silently measures nothing.
	 *
	 * Asserted through [CorpusMetadata.parse] rather than by grepping the text,
	 * because what matters is not that the key was written but that the loader
	 * comes back with the type the trackers actually had.
	 */
	@Test
	@DisplayName("imu_type survives the write, so Stay Aligned is not silently inert")
	fun `imu type round trips`() {
		val text = CorpusMetadataWriter.render(
			"standing-still",
			attestation(),
			derived(imuType = IMUType.BMI160),
		)
		assertEquals(IMUType.BMI160, CorpusMetadata.parse("standing-still", text).imuType)
	}

	@Test
	@DisplayName("relaxed poses survive the write, angles included")
	fun `stay aligned round trips`() {
		val text = CorpusMetadataWriter.render(
			"standing-still",
			attestation(),
			derived(stayAligned = stayAlignedConfig()),
		)
		val parsed = CorpusMetadata.parse("standing-still", text)

		val config = parsed.stayAligned!!
		assertTrue(config.standingRelaxedPose.enabled)
		assertEquals(3.5f, config.standingRelaxedPose.upperLegAngleInDeg)
		assertEquals(1f, config.standingRelaxedPose.lowerLegAngleInDeg)
		assertEquals(-2.25f, config.standingRelaxedPose.footAngleInDeg)
		assertFalse(config.sittingRelaxedPose.enabled)
		assertFalse(config.flatRelaxedPose.enabled)
	}

	/**
	 * `CorpusMetadata.parse` turns Stay Aligned on for the replay as soon as any
	 * `stay_aligned.*` key is present. So a capture made with the feature off
	 * must write none of them: emitting the poses of a disabled config would
	 * make the replay run a correction the recording was never made under, which
	 * is a subtler version of the same silent-mismatch problem.
	 */
	@Test
	@DisplayName("a capture with Stay Aligned off writes no relaxed poses at all")
	fun `disabled stay aligned writes nothing`() {
		val text = CorpusMetadataWriter.render("crouch", attestation(), derived(stayAligned = null))

		assertFalse(text.lines().any { it.substringBefore('#').contains("stay_aligned.") })
		assertNull(CorpusMetadata.parse("crouch", text).stayAligned)
	}

	@Test
	@DisplayName("skeleton proportions survive the write")
	fun `offsets round trip`() {
		val offsets = mapOf(
			SkeletonConfigOffsets.UPPER_LEG to 0.42f,
			SkeletonConfigOffsets.LOWER_LEG to 0.445f,
		)
		val text = CorpusMetadataWriter.render("crouch", attestation(), derived(offsets = offsets))
		val parsed = CorpusMetadata.parse("crouch", text)

		assertEquals(0.42f, parsed.offsets[SkeletonConfigOffsets.UPPER_LEG])
		assertEquals(0.445f, parsed.offsets[SkeletonConfigOffsets.LOWER_LEG])
	}

	/**
	 * A small offset formatted as `1.0E-4` parses back correctly but reads as
	 * noise in the diff the sidecar exists to be read in.
	 */
	@Test
	@DisplayName("small values are written in fixed notation, not exponent form")
	fun `no scientific notation`() {
		val text = CorpusMetadataWriter.render(
			"crouch",
			attestation(),
			derived(offsets = mapOf(SkeletonConfigOffsets.UPPER_LEG to 0.0001f)),
		)

		assertFalse(text.contains("E-", ignoreCase = true))
		assertEquals(
			0.0001f,
			CorpusMetadata.parse("crouch", text).offsets[SkeletonConfigOffsets.UPPER_LEG],
		)
	}

	/**
	 * Every field the schema requires must be one the writer actually emits.
	 * A writer that silently omits a required field produces a file rejected at
	 * commit time, after the session it came from is over.
	 */
	@Test
	@DisplayName("the writer emits every field the schema requires")
	fun `all required fields are written`() {
		val text = CorpusMetadataWriter.render("walk-in-place", attestation(), derived())
		val keys = text.lines()
			.map { it.substringBefore('#').trim() }
			.filter { it.contains('=') }
			.map { it.substringBefore('=').trim() }

		for (required in CorpusMetadata.REQUIRED) {
			assertTrue(required in keys, "writer never emits required field '$required'")
		}
	}

	@Test
	@DisplayName("a mixed-IMU capture records the breakdown as a comment")
	fun `mixed imu types are disclosed`() {
		val text = CorpusMetadataWriter.render(
			"walk-in-place",
			attestation(),
			derived(imuType = IMUType.BMI270, imuTypeBreakdown = "BMI270 x5, BMI160 x2"),
		)

		assertTrue(text.contains("BMI270 x5, BMI160 x2"))
		assertEquals(IMUType.BMI270, CorpusMetadata.parse("walk-in-place", text).imuType)
	}

	/**
	 * The attestation is checked before a recording starts rather than when the
	 * file is loaded. The saving is not the round trip -- it is that a wearer
	 * does not stand still for five minutes producing a recording that will be
	 * rejected the moment someone tries to commit it.
	 */
	@Test
	@DisplayName("a placeholder attestation is rejected up front, not at load")
	fun `placeholder attestation rejected`() {
		for (placeholder in CorpusMetadata.PLACEHOLDERS) {
			assertThrows<IllegalArgumentException> {
				attestation(capturer = placeholder).validate()
			}
			assertThrows<IllegalArgumentException> {
				attestation(capturer = placeholder.uppercase()).validate()
			}
		}
	}

	@Test
	@DisplayName("a blank attestation field is rejected up front")
	fun `blank attestation rejected`() {
		assertThrows<IllegalArgumentException> { attestation(description = "  ").validate() }
		assertThrows<IllegalArgumentException> { attestation(capturer = "").validate() }
		assertThrows<IllegalArgumentException> { attestation(consent = "").validate() }
	}

	/**
	 * The sidecar is one key per line, so a newline inside a value would silently
	 * truncate the field and turn its tail into a parse error on the next line.
	 */
	@Test
	@DisplayName("a multi-line attestation value is rejected")
	fun `multiline attestation rejected`() {
		assertThrows<IllegalArgumentException> {
			attestation(description = "first line\nsecond line").validate()
		}
	}

	@Test
	@DisplayName("rendering refuses an attestation it would not accept at load")
	fun `render validates`() {
		assertThrows<IllegalArgumentException> {
			CorpusMetadataWriter.render("walk-in-place", attestation(capturer = "TODO"), derived())
		}
	}

	// --- console flow -------------------------------------------------------

	private class Console(input: String) {
		val captured = mutableListOf<CorpusMetadataWriter.Attestation>()
		val out = ByteArrayOutputStream()
		val console = CorpusCaptureConsole(
			{ name, _, rate, dir, attestation, _ ->
				captured += attestation
				CorpusCapture.Result(
					name = name,
					pfr = File(dir, "$name.pfr"),
					meta = File(dir, "$name.meta"),
					frames = 100,
					trackers = 7,
					derived = CorpusMetadataWriter.Derived(
						rateHz = rate,
						captured = "2026-08-14",
						trackers = "7 -- ...",
						firmware = null,
						imuType = IMUType.BMI270,
						imuTypeBreakdown = null,
						offsets = emptyMap(),
						stayAligned = null,
					),
					warnings = emptyList(),
				)
			},
			BufferedReader(StringReader(input)),
			PrintStream(out),
		)

		fun handle(line: String) = console.handle(line)

		fun output() = out.toString()
	}

	private fun answers(vararg lines: String) = lines.joinToString("\n", postfix = "\n")

	@Test
	@DisplayName("a completed prompt flow records, with consent stored as a sentence")
	fun `console happy path`() {
		val console = Console(
			answers("walking in place", "A. Person <a@example.com>", "I agree", "hardwood floor"),
		)

		assertTrue(console.handle("record-corpus walk-in-place 30"))
		assertEquals(1, console.captured.size)

		val attestation = console.captured.single()
		assertEquals("walking in place", attestation.description)
		assertEquals("A. Person <a@example.com>", attestation.capturer)
		assertEquals("hardwood floor", attestation.notes)
		assertTrue(attestation.consent.contains("A. Person"))
		attestation.validate()
	}

	/**
	 * The consent gate is the reason the flow is interactive rather than a set
	 * of flags. These are recordings of an identifiable person's body movement
	 * committed to a public repository; a field that accepts any string to make
	 * the command run is not a gate.
	 */
	@Test
	@DisplayName("consent not given records nothing")
	fun `console refuses without consent`() {
		val console = Console(answers("walking in place", "A. Person", "sure whatever", "notes"))

		assertTrue(console.handle("record-corpus walk-in-place 30"))
		assertTrue(console.captured.isEmpty())
		assertTrue(console.output().contains("Consent not given"))
	}

	@Test
	@DisplayName("an abandoned prompt records nothing")
	fun `console cancels on blank`() {
		val console = Console(answers("", "", ""))

		assertTrue(console.handle("record-corpus walk-in-place 30"))
		assertTrue(console.captured.isEmpty())
		assertTrue(console.output().contains("cancelled"))
	}

	@Test
	@DisplayName("other console input is left alone")
	fun `console ignores other commands`() {
		val console = Console("")

		assertFalse(console.handle("exit"))
		assertFalse(console.handle(""))
		assertTrue(console.captured.isEmpty())
	}

	/**
	 * A bad argument reports a sentence rather than a stack trace. A capture is
	 * a live session with a person standing in a room; the console has to stay
	 * usable enough to retype the command.
	 */
	@Test
	@DisplayName("a bad argument reports and leaves the console usable")
	fun `console reports bad arguments`() {
		val console = Console("")

		assertTrue(console.handle("record-corpus walk-in-place soon"))
		assertTrue(console.captured.isEmpty())
		assertTrue(console.output().contains("not a number of seconds"))

		assertTrue(console.handle("record-corpus"))
		assertTrue(console.output().contains("usage:"))
	}
}
