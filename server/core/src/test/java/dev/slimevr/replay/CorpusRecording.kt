package dev.slimevr.replay

import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.poseframeformat.PfrIO
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.udp.IMUType
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
	/**
	 * Which IMU the trackers used, when the sidecar says.
	 *
	 * Not in the `.pfr`, and not cosmetic. `TrackerFrames.toTracker()` builds a
	 * tracker with no IMU type unless told one, `Tracker.isImu()` is then false,
	 * and `AdjustTrackerYaw.adjust` returns on that before doing anything --
	 * **Stay Aligned is inert for the entire replay, and nothing says so.**
	 *
	 * Same class of gap as the missing sample rate, and it matters for the same
	 * reason: a recording captured to answer issue #3's question about Stay
	 * Aligned would replay with Stay Aligned switched off.
	 */
	val imuType: IMUType?,
	/**
	 * The Stay Aligned relaxed pose in force at capture, when the sidecar says.
	 *
	 * The second way a replay silently runs without yaw correction.
	 * `RelaxedPose.forPose` returns null when the config for the player's current
	 * posture is disabled and `adjustMovingTracker` returns on that null, so a
	 * standing, moving player gets no centring force at all. Every pose is
	 * disabled in a default config, so a recording replayed without this gets
	 * that behaviour by default.
	 *
	 * Recorded per recording rather than assumed: it is whatever the wearer had
	 * captured at the time, and a recording made to exercise Stay Aligned is
	 * uninterpretable without it.
	 */
	val stayAligned: StayAlignedConfig?,
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
		private const val IMU_TYPE = "imu_type"

		/** Sidecar prefix for the relaxed poses, e.g. `stay_aligned.standing.enabled`. */
		private const val STAY_ALIGNED_PREFIX = "stay_aligned."

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
		 * `imu_type = BMI270`, matched against the [IMUType] enum.
		 *
		 * Optional, because a recording made to exercise the leg corrections has
		 * no need of it. Rejected rather than ignored when unrecognised: a typo
		 * here does not fail, it silently disables yaw correction for the whole
		 * recording, which is the failure this field exists to prevent.
		 */
		private fun parseImuType(name: String, fields: Map<String, String>): IMUType? {
			val raw = fields[IMU_TYPE]?.takeIf { it.isNotBlank() } ?: return null
			val imu = IMUType.entries.firstOrNull { it.name.equals(raw, ignoreCase = true) }
			requireNotNull(imu) {
				"$name.meta: '$raw' is not an IMUType. Known: " +
					IMUType.entries.joinToString { it.name }
			}
			return imu
		}

		/**
		 * `stay_aligned.<pose>.<field>`, where pose is `standing`, `sitting` or
		 * `flat` and field is `enabled`, `upper_leg_deg`, `lower_leg_deg` or
		 * `foot_deg`.
		 *
		 * Returns null when the sidecar says nothing, which leaves the replay on
		 * the default config -- every pose disabled, so no centring force. That is
		 * the right default for a recording that says nothing about Stay Aligned,
		 * and the wrong one to arrive at by accident, which is why the README
		 * spells it out.
		 */
		private fun parseStayAligned(name: String, fields: Map<String, String>): StayAlignedConfig? {
			if (fields.isEmpty()) return null

			val config = StayAlignedConfig()
			config.enabled = true

			for ((key, value) in fields) {
				val dot = key.indexOf('.')
				require(dot > 0) {
					"$name.meta: 'stay_aligned.$key' should be " +
						"stay_aligned.<standing|sitting|flat>.<field>"
				}
				val poseName = key.substring(0, dot)
				val field = key.substring(dot + 1)

				val pose = when (poseName) {
					"standing" -> config.standingRelaxedPose

					"sitting" -> config.sittingRelaxedPose

					"flat" -> config.flatRelaxedPose

					else -> throw IllegalArgumentException(
						"$name.meta: '$poseName' is not a relaxed pose. " +
							"Known: standing, sitting, flat",
					)
				}

				when (field) {
					"enabled" -> pose.enabled = value.toBooleanStrictOrNull()
						?: throw IllegalArgumentException(
							"$name.meta: stay_aligned.$key must be true or false, got '$value'",
						)

					"upper_leg_deg" -> pose.upperLegAngleInDeg = requireFloat(name, key, value)

					"lower_leg_deg" -> pose.lowerLegAngleInDeg = requireFloat(name, key, value)

					"foot_deg" -> pose.footAngleInDeg = requireFloat(name, key, value)

					else -> throw IllegalArgumentException(
						"$name.meta: '$field' is not a relaxed pose field. " +
							"Known: enabled, upper_leg_deg, lower_leg_deg, foot_deg",
					)
				}
			}

			return config
		}

		private fun requireFloat(name: String, key: String, value: String): Float = value.toFloatOrNull()
			?: throw IllegalArgumentException(
				"$name.meta: stay_aligned.$key is not a number: '$value'",
			)

		/**
		 * Parses a sidecar. `key = value`, `#` comments, blank lines ignored --
		 * the same plain-text-over-JSON reasoning as `replay-baseline.txt`: the
		 * file exists to be read in a pull request.
		 */
		fun parse(name: String, file: File, text: String): CorpusRecording {
			val fields = linkedMapOf<String, String>()
			val offsets = linkedMapOf<SkeletonConfigOffsets, Float>()
			val stayAlignedFields = linkedMapOf<String, String>()

			for ((lineNo, raw) in text.lines().withIndex()) {
				val line = raw.substringBefore('#').trim()
				if (line.isEmpty()) continue

				val eq = line.indexOf('=')
				require(eq > 0) {
					"$name.meta:${lineNo + 1}: expected 'key = value', got '$line'"
				}
				val key = line.substring(0, eq).trim()
				val value = line.substring(eq + 1).trim()

				if (key.startsWith(STAY_ALIGNED_PREFIX)) {
					require(stayAlignedFields.put(key.removePrefix(STAY_ALIGNED_PREFIX), value) == null) {
						"$name.meta:${lineNo + 1}: duplicate key '$key'"
					}
				} else if (key.startsWith("offset.")) {
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

			return CorpusRecording(
				name = name,
				file = file,
				fields = fields,
				offsets = offsets,
				imuType = parseImuType(name, fields),
				stayAligned = parseStayAligned(name, stayAlignedFields),
			)
		}
	}
}
