package dev.slimevr.poseframeformat

import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.udp.IMUType

/**
 * The `.meta` sidecar that accompanies every committed `.pfr` recording:
 * the capture metadata without which the recording cannot be interpreted.
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
 * ## Why this lives in main rather than beside the replay suite
 *
 * [CorpusCapture] writes these files and `dev.slimevr.replay.CorpusRecording`
 * reads them. A schema defined twice is a schema that drifts, and the direction
 * it would drift in is the damaging one: a writer that omits a field the reader
 * treats as optional produces a recording that replays *successfully* with a
 * correction silently switched off. So the schema is defined once, here, and
 * the capture path validates its own output by parsing it back through
 * [parse] -- the same call the suite will make, while the wearer is still
 * wearing the trackers.
 *
 * See `test/resources/corpus/README.md` for the capture protocol.
 */
class CorpusMetadata(
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
	val description: String get() = fields.getValue(DESCRIPTION)

	companion object {
		const val RATE_HZ = "rate_hz"
		const val DESCRIPTION = "description"
		const val CAPTURED = "captured"
		const val CAPTURER = "capturer"
		const val CONSENT = "consent"
		const val TRACKERS = "trackers"
		const val FIRMWARE = "firmware"
		const val NOTES = "notes"
		const val IMU_TYPE = "imu_type"

		/** Sidecar prefix for the relaxed poses, e.g. `stay_aligned.standing.enabled`. */
		const val STAY_ALIGNED_PREFIX = "stay_aligned."

		/** Sidecar prefix for the skeleton proportions, e.g. `offset.UPPER_LEG`. */
		const val OFFSET_PREFIX = "offset."

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
			DESCRIPTION,
			CAPTURED,
			CAPTURER,
			CONSENT,
			TRACKERS,
		)

		/**
		 * Values that look like an unfilled template. Rejected explicitly: a
		 * sidecar copied from the example and left with its placeholders intact
		 * is worse than no sidecar, because it reads as provenance.
		 */
		val PLACEHOLDERS = setOf("todo", "tbd", "unknown", "?", "xxx", "fixme")

		/** Relaxed poses addressable as `stay_aligned.<pose>.<field>`. */
		val POSES = listOf("standing", "sitting", "flat")

		/** Fields addressable within a relaxed pose. */
		val POSE_FIELDS = listOf("enabled", "upper_leg_deg", "lower_leg_deg", "foot_deg")

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
							"Known: ${POSES.joinToString()}",
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
							"Known: ${POSE_FIELDS.joinToString()}",
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
		fun parse(name: String, text: String): CorpusMetadata {
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
				} else if (key.startsWith(OFFSET_PREFIX)) {
					val offsetName = key.removePrefix(OFFSET_PREFIX)
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

			return CorpusMetadata(
				fields = fields,
				offsets = offsets,
				imuType = parseImuType(name, fields),
				stayAligned = parseStayAligned(name, stayAlignedFields),
			)
		}
	}
}
