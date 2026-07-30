package dev.slimevr.poseframeformat

import java.io.BufferedReader
import java.io.File
import java.io.PrintStream

/**
 * The `record-corpus` console command.
 *
 * Drives [CorpusCapture] from the server's existing stdin loop. Interactive
 * rather than a flag soup because two of the three fields a person must supply
 * are prose -- what the recording is for, and an attestation of consent -- and
 * because the consent gate is worth being a gate rather than an argument that
 * can be filled with anything to make the command run.
 *
 * Reads from a [BufferedReader] and writes to a [PrintStream] rather than
 * touching stdin directly, so the whole flow -- including the refusal paths --
 * is exercised in tests without a server or a wearer.
 *
 * ## Usage
 *
 * ```
 * record-corpus <name> <seconds> [rate-hz] [output-dir]
 * ```
 *
 * Then answer the prompts. `rate-hz` defaults to
 * [CorpusCapture.DEFAULT_RATE_HZ] and `output-dir` to `./corpus`.
 */
class CorpusCaptureConsole(
	private val capture: Capture,
	private val input: BufferedReader,
	private val output: PrintStream,
) {
	/**
	 * What the console does once it has arguments and an attestation.
	 *
	 * An interface rather than [CorpusCapture] itself so the prompt flow -- and
	 * particularly the paths where it refuses to record -- can be tested without
	 * a [dev.slimevr.VRServer], a wearer, or trackers.
	 */
	fun interface Capture {
		fun run(
			name: String,
			seconds: Float,
			rateHz: Float,
			outputDir: File,
			attestation: CorpusMetadataWriter.Attestation,
			onProgress: (Int, Int) -> Unit,
		): CorpusCapture.Result
	}

	/**
	 * Handles one console line.
	 *
	 * Returns false when the line is not a `record-corpus` command, so the
	 * caller's own command handling is unaffected.
	 */
	fun handle(line: String): Boolean {
		val parts = line.trim().split(Regex("\\s+"))
		if (parts.firstOrNull() != COMMAND) return false

		try {
			execute(parts.drop(1))
		} catch (e: Exception) {
			// A capture is a live session with a person standing in a room. A
			// stack trace on the console ends that session; a sentence lets them
			// fix the argument and go again.
			output.println("record-corpus: ${e.message}")
		}
		return true
	}

	private fun execute(args: List<String>) {
		if (args.size !in 2..4) {
			output.println("usage: $COMMAND <name> <seconds> [rate-hz] [output-dir]")
			return
		}

		val name = args[0]
		val seconds = args[1].toFloatOrNull()
			?: throw IllegalArgumentException("'${args[1]}' is not a number of seconds")
		val rateHz = args.getOrNull(2)?.let {
			it.toFloatOrNull() ?: throw IllegalArgumentException("'$it' is not a rate in Hz")
		} ?: CorpusCapture.DEFAULT_RATE_HZ
		val outputDir = File(args.getOrNull(3) ?: DEFAULT_OUTPUT_DIR)

		val attestation = prompt() ?: run {
			output.println("record-corpus: cancelled, nothing recorded")
			return
		}

		output.println()
		output.println("Recording '$name' for ${"%.1f".format(seconds)} s at $rateHz Hz.")
		output.println("Start moving now.")

		var lastPercent = -1
		val result = capture.run(name, seconds, rateHz, outputDir, attestation) { frame, total ->
			val percent = frame * 100 / total.coerceAtLeast(1)
			if (percent / 10 != lastPercent / 10) {
				lastPercent = percent
				output.println("  $percent%  ($frame / $total frames)")
			}
		}

		report(result)
	}

	/**
	 * Collects the three fields no machine can supply.
	 *
	 * Returns null when the operator abandons the capture, which includes
	 * declining consent. Nothing is recorded in that case -- the alternative,
	 * recording first and asking afterwards, is how a file of someone's motion
	 * ends up on disk without their agreement.
	 */
	private fun prompt(): CorpusMetadataWriter.Attestation? {
		output.println("Three fields cannot be read off the server. Blank cancels.")

		val description = ask("What is this recording for (the failure mode it exercises)?")
			?: return null
		val capturer = ask("Who wore the trackers? Name and contact, committed to the repo.")
			?: return null

		output.println()
		output.println("This is a motion recording of a real person and will be committed")
		output.println("to a public repository. The wearer must agree to its redistribution")
		output.println("under the repository's licence.")
		val agreement = ask("Type '$CONSENT_PHRASE' to record that agreement.")
			?: return null
		if (!agreement.equals(CONSENT_PHRASE, ignoreCase = true)) {
			output.println("Consent not given -- nothing recorded.")
			return null
		}

		val notes = ask("Notes (environment, anything worth knowing)? Optional.")

		return CorpusMetadataWriter.Attestation(
			description = description,
			capturer = capturer,
			// Stored as a sentence rather than the bare phrase, because the field
			// is read years later by someone deciding whether the file may stay.
			consent = "$CONSENT_PHRASE -- redistribution under the repository's licence, " +
				"given by $capturer",
			notes = notes,
		)
	}

	private fun ask(question: String): String? {
		output.println(question)
		output.print("> ")
		output.flush()
		return input.readLine()?.trim()?.takeIf { it.isNotEmpty() }
	}

	private fun report(result: CorpusCapture.Result) {
		output.println()
		output.println("Recorded ${result.frames} frames from ${result.trackers} trackers.")
		output.println("  ${result.pfr.absolutePath}")
		output.println("  ${result.meta.absolutePath}")
		output.println()
		output.println("Derived from the running server:")
		output.println("  rate      ${result.derived.rateHz} Hz")
		output.println("  trackers  ${result.derived.trackers}")
		output.println("  imu       ${result.derived.imuType?.name ?: "(none reported)"}")
		output.println(
			"  aligned   " +
				if (result.derived.stayAligned != null) {
					"relaxed poses recorded"
				} else {
					"disabled at capture"
				},
		)

		for (warning in result.warnings) {
			output.println()
			output.println("WARNING: $warning")
		}

		output.println()
		output.println("Both files are needed. Commit them together into")
		output.println("server/core/src/test/resources/corpus/, then regenerate the")
		output.println("corpus baseline block -- see corpus/README.md.")
	}

	companion object {
		const val COMMAND = "record-corpus"

		/** Typed verbatim; an argument that accepts anything is not a gate. */
		const val CONSENT_PHRASE = "I agree"

		const val DEFAULT_OUTPUT_DIR = "corpus"
	}
}
