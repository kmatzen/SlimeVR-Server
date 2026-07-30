package dev.slimevr.poseframeformat

import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.CAPTURED
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.CAPTURER
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.CONSENT
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.DESCRIPTION
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.FIRMWARE
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.IMU_TYPE
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.NOTES
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.OFFSET_PREFIX
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.RATE_HZ
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.STAY_ALIGNED_PREFIX
import dev.slimevr.poseframeformat.CorpusMetadata.Companion.TRACKERS
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.udp.IMUType

/**
 * Renders a `.meta` sidecar.
 *
 * Separate from [CorpusMetadata] only to keep the parser -- which every replay
 * depends on -- free of the formatting concerns that exist purely so the file
 * reads well in a pull request diff.
 *
 * ## Why the writer exists at all
 *
 * Issue #15 and its follow-up both make the same point about the fields that
 * decide whether a correction runs: they are *cheap at capture time and
 * impossible to recover afterwards*. `imu_type` and `stay_aligned.*` are the
 * sharp cases -- absent, the replay completes, every metric is produced, and
 * yaw correction never ran.
 *
 * A human transcribing those by hand after a capture session is exactly the
 * failure mode that warning describes. Every field here that *can* be read off
 * the running server is read off the running server, so the only fields anyone
 * types are the ones no machine can know: what the recording is for, who wore
 * the trackers, and that they agreed to its redistribution.
 */
object CorpusMetadataWriter {
	/** Column the `=` is padded to, so a sidecar reads as a table. */
	private const val KEY_WIDTH = 11

	/**
	 * The values a capture cannot derive and a person must supply.
	 *
	 * [notes] is optional; the other three are required by the schema and are
	 * checked before a recording starts rather than after, so a capture session
	 * is never spent on a recording that will be rejected when it is committed.
	 */
	class Attestation(
		val description: String,
		val capturer: String,
		val consent: String,
		val notes: String? = null,
	) {
		/**
		 * Applies the schema's own rules to the typed fields up front.
		 *
		 * The same checks [CorpusMetadata.parse] makes, run before the wearer
		 * stands still for five minutes rather than after. Deliberately reuses
		 * [CorpusMetadata.PLACEHOLDERS] instead of restating it: a value this
		 * accepts and the parser later rejects would be a capture lost to a
		 * disagreement between two copies of one rule.
		 */
		fun validate() {
			val typed = mapOf(
				DESCRIPTION to description,
				CAPTURER to capturer,
				CONSENT to consent,
			)
			for ((key, value) in typed) {
				require(value.isNotBlank()) {
					"$key must be given: it is required by the sidecar schema and " +
						"cannot be derived from the server"
				}
				require(value.lowercase() !in CorpusMetadata.PLACEHOLDERS) {
					"$key is left at a template placeholder ('$value'). An unfilled " +
						"field reads as provenance without being any"
				}
				require('\n' !in value) {
					"$key must be a single line: the sidecar is one key per line"
				}
			}
		}
	}

	/**
	 * Everything the capture read off the running server.
	 *
	 * A plain snapshot rather than live references, so what is written describes
	 * the state the recording was actually made under even if the server moves
	 * on while the file is being written.
	 */
	class Derived(
		val rateHz: Float,
		val captured: String,
		val trackers: String,
		val firmware: String?,
		val imuType: IMUType?,
		/** All IMU types seen, when the trackers did not agree on one. */
		val imuTypeBreakdown: String?,
		val offsets: Map<SkeletonConfigOffsets, Float>,
		/**
		 * Null when Stay Aligned was switched off at capture.
		 *
		 * Written as nothing rather than as disabled poses, because
		 * [CorpusMetadata.parse] turns Stay Aligned *on* for the replay as soon as
		 * any `stay_aligned.*` key is present. Emitting the poses of a wearer who
		 * had the feature off would make the replay run a correction the recording
		 * never had.
		 */
		val stayAligned: StayAlignedConfig?,
	)

	fun render(name: String, attestation: Attestation, derived: Derived): String {
		attestation.validate()

		val out = StringBuilder()
		fun field(key: String, value: String) {
			out.append(key.padEnd(KEY_WIDTH)).append(" = ").append(value).append('\n')
		}
		fun comment(text: String) = out.append("# ").append(text).append('\n')

		comment("$name.meta -- written by 'record-corpus'; see corpus/README.md")
		comment("Derived fields come from the running server. The three typed ones")
		comment("(description, capturer, consent) are the ones no machine can know.")
		out.append('\n')

		field(RATE_HZ, formatFloat(derived.rateHz))
		field(DESCRIPTION, attestation.description)
		field(CAPTURED, derived.captured)
		field(CAPTURER, attestation.capturer)
		field(CONSENT, attestation.consent)
		field(TRACKERS, derived.trackers)
		derived.firmware?.let { field(FIRMWARE, it) }
		attestation.notes?.takeIf { it.isNotBlank() }?.let { field(NOTES, it) }

		out.append('\n')
		if (derived.imuType != null) {
			derived.imuTypeBreakdown?.let {
				comment("Trackers reported more than one IMU: $it.")
				comment("The most common is recorded; the schema holds a single type.")
			}
			field(IMU_TYPE, derived.imuType.name)
		} else {
			comment("No tracker reported an IMU type, so imu_type is omitted.")
			comment("Stay Aligned will be INERT for the whole replay: Tracker.isImu()")
			comment("is false without it and AdjustTrackerYaw returns before acting.")
		}

		out.append('\n')
		if (derived.stayAligned != null) {
			comment("Stay Aligned relaxed poses in force at capture.")
			renderStayAligned(derived.stayAligned, ::field)
		} else {
			comment("Stay Aligned was disabled at capture, so no stay_aligned.* keys.")
			comment("Adding any of them would switch the correction ON for the replay,")
			comment("which this recording was not made under.")
		}

		if (derived.offsets.isNotEmpty()) {
			out.append('\n')
			comment("Skeleton proportions in force at capture. Applied before replay,")
			comment("so this recording is not solved against default proportions.")
			for ((offset, value) in derived.offsets) {
				field(OFFSET_PREFIX + offset.name, formatFloat(value))
			}
		}

		return out.toString()
	}

	private fun renderStayAligned(config: StayAlignedConfig, field: (String, String) -> Unit) {
		val poses = listOf(
			"standing" to config.standingRelaxedPose,
			"sitting" to config.sittingRelaxedPose,
			"flat" to config.flatRelaxedPose,
		)
		for ((poseName, pose) in poses) {
			val prefix = "$STAY_ALIGNED_PREFIX$poseName."
			field(prefix + "enabled", pose.enabled.toString())
			field(prefix + "upper_leg_deg", formatFloat(pose.upperLegAngleInDeg))
			field(prefix + "lower_leg_deg", formatFloat(pose.lowerLegAngleInDeg))
			field(prefix + "foot_deg", formatFloat(pose.footAngleInDeg))
		}
	}

	/**
	 * Fixed notation, trailing zeros trimmed.
	 *
	 * Avoids `1.0E-4` for a small offset, which parses back correctly but reads
	 * as noise in the diff the file exists to be read in.
	 */
	private fun formatFloat(value: Float): String {
		val text = "%.6f".format(value)
		return text.trimEnd('0').trimEnd('.').ifEmpty { "0" }
	}
}
