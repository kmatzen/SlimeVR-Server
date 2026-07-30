package dev.slimevr.replay

import dev.slimevr.poseframeformat.PfrIO
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import java.io.File

/**
 * A committed `.pfr` recording together with the capture metadata needed to
 * interpret it.
 *
 * ## Why a sidecar file is mandatory rather than nice to have
 *
 * The `.pfr` container does not carry a frame rate. [PoseFrames.frameInterval]
 * exists in memory and defaults to 0.02 s, but [PfrIO.writeFrames] never writes
 * it and [PfrIO.readFrames] never reads it -- the format is a tracker count,
 * then per tracker a name, a frame count, and the frames. So a `.pfr` on disk
 * is a sequence of poses with no statement of how fast they were sampled.
 *
 * Every time-normalised metric depends on that number. `foot_slide_m_per_sec`
 * is metres divided by planted *seconds*, and the seconds come entirely from
 * the assumed rate; replay a 100 Hz capture as 50 Hz and the same file reports
 * half the skating. The leg corrections are worse than that, because
 * `LegTweaksBuffer` derives foot velocities from the frame interval and
 * compares them against fixed thresholds -- a wrong rate does not scale the
 * output, it changes which frames are considered planted at all.
 *
 * [rateHz] is therefore required, and a recording without a sidecar is an error
 * rather than a file replayed at a guessed rate. The rest of the required
 * fields exist for the reason issue #15 gives: a recording whose provenance is
 * unknown cannot be interpreted when its metrics move two years from now, and
 * will eventually be deleted by someone who cannot tell whether it still means
 * anything.
 *
 * See `test/resources/corpus/README.md` for the schema and the capture
 * protocol.
 */
class CorpusRecording(
	val name: String,
	val file: File,
	val fields: Map<String, String>,
	val offsets: Map<SkeletonConfigOffsets, Float>,
) {
	/** Sample rate of the capture. Absent from the `.pfr` container itself. */
	val rateHz: Float get() = fields.getValue(RATE_HZ).toFloat()

	/** What this recording is for -- the failure mode it exercises. */
	val description: String get() = fields.getValue("description")

	fun load(): PoseFrames = PfrIO.readFromFile(file)

	/** One-line inventory entry, for the suite's report. */
	fun summary(): String = "%-20s %6.1f Hz  %s".format(name, rateHz, description)

	companion object {
		/** Resource directory holding the corpus, relative to test resources. */
		const val RESOURCE_DIR = "/corpus"

		private const val RATE_HZ = "rate_hz"

		/**
		 * Fields every recording must declare.
		 *
		 * `rate_hz` because the format omits it and the metrics cannot be
		 * computed without it; the others because issue #15 asks for a capture
		 * protocol recorded alongside each file, and a protocol that is not
		 * enforced is a protocol that is not followed.
		 */
		val REQUIRED = listOf(
			RATE_HZ,
			"description",
			"captured",
			"capturer",
			"consent",
			"trackers",
		)

		/**
		 * Values that look like an unfilled template. Rejected explicitly: a
		 * sidecar copied from the example and left with its placeholders intact
		 * is worse than no sidecar, because it reads as provenance.
		 */
		private val PLACEHOLDERS = setOf("todo", "tbd", "unknown", "?", "xxx", "fixme")

		/**
		 * Every recording in the committed corpus.
		 *
		 * Returns empty when the directory holds no `.pfr` files, which is the
		 * state of the repository until captures are made. It throws rather
		 * than skipping when a recording is *present but unusable*, because
		 * those are different situations: the first is work not yet done, the
		 * second is a committed file nobody can interpret.
		 */
		fun discover(): List<CorpusRecording> {
			val dir = directory() ?: return emptyList()
			val recordings = dir.listFiles { f -> f.isFile && f.name.endsWith(".pfr") }
				?: return emptyList()

			return recordings.sortedBy { it.name }.map { pfr ->
				val name = pfr.name.removeSuffix(".pfr")
				val meta = File(pfr.parentFile, "$name.meta")
				require(meta.isFile) {
					"corpus recording '$name' has no $name.meta sidecar. The .pfr " +
						"format carries no sample rate, so the file cannot be " +
						"replayed without one -- see corpus/README.md"
				}
				parse(name, pfr, meta.readText())
			}
		}

		/**
		 * Resolved through the classloader rather than a source-relative path,
		 * so this reads the same copy of the corpus the rest of the suite loads
		 * its resources from.
		 */
		fun directory(): File? {
			val url = CorpusRecording::class.java.getResource(RESOURCE_DIR) ?: return null
			if (url.protocol != "file") return null
			return File(url.toURI()).takeIf { it.isDirectory }
		}

		/**
		 * Parses a sidecar. `key = value`, `#` comments, blank lines ignored --
		 * the same plain-text-over-JSON reasoning as `replay-baseline.txt`: the
		 * file exists to be read in a pull request.
		 */
		fun parse(name: String, file: File, text: String): CorpusRecording {
			val fields = linkedMapOf<String, String>()
			val offsets = linkedMapOf<SkeletonConfigOffsets, Float>()

			for ((lineNo, raw) in text.lines().withIndex()) {
				val line = raw.substringBefore('#').trim()
				if (line.isEmpty()) continue

				val eq = line.indexOf('=')
				require(eq > 0) {
					"$name.meta:${lineNo + 1}: expected 'key = value', got '$line'"
				}
				val key = line.substring(0, eq).trim()
				val value = line.substring(eq + 1).trim()

				if (key.startsWith("offset.")) {
					val offsetName = key.removePrefix("offset.")
					val offset = SkeletonConfigOffsets.values.firstOrNull { it.name == offsetName }
					requireNotNull(offset) {
						"$name.meta:${lineNo + 1}: '$offsetName' is not a SkeletonConfigOffsets " +
							"constant. Known: ${SkeletonConfigOffsets.values.joinToString { it.name }}"
					}
					val parsed = value.toFloatOrNull()
					requireNotNull(parsed) {
						"$name.meta:${lineNo + 1}: offset '$offsetName' is not a number: '$value'"
					}
					offsets[offset] = parsed
				} else {
					require(fields.put(key, value) == null) {
						"$name.meta:${lineNo + 1}: duplicate key '$key'"
					}
				}
			}

			val missing = REQUIRED.filter { fields[it].isNullOrBlank() }
			require(missing.isEmpty()) {
				"$name.meta is missing required field(s): ${missing.joinToString()}. " +
					"See corpus/README.md for what each one is for"
			}

			val placeholder = REQUIRED.filter { fields.getValue(it).lowercase() in PLACEHOLDERS }
			require(placeholder.isEmpty()) {
				"$name.meta leaves ${placeholder.joinToString()} at a template placeholder. " +
					"An unfilled field reads as provenance without being any"
			}

			val rate = fields.getValue(RATE_HZ).toFloatOrNull()
			require(rate != null && rate > 0f && rate.isFinite()) {
				"$name.meta: rate_hz must be a positive number, got '${fields.getValue(RATE_HZ)}'"
			}

			return CorpusRecording(name, file, fields, offsets)
		}
	}
}
