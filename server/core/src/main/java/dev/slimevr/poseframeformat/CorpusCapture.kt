package dev.slimevr.poseframeformat

import dev.slimevr.VRServer
import dev.slimevr.rawsamples.ImuLogWriter
import dev.slimevr.rawsamples.RawSampleCapture
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.Tracker
import io.eiren.util.logging.LogManager
import java.io.File
import java.time.LocalDate
import java.util.concurrent.TimeUnit

/**
 * Captures a corpus-ready recording: a `.pfr` and the `.meta` sidecar that
 * makes it interpretable, written together, from a live server.
 *
 * ## The gap this closes
 *
 * Issue #15 asks for real recordings, and the corpus README describes the two
 * files a recording consists of. What did not exist was a way to *produce* the
 * pair. The only recording path in the server is the AutoBone flow, which
 * writes a `.pfs` into AutoBone's own directory and no sidecar at all -- so
 * capturing for the corpus meant renaming a file into a different container's
 * extension and hand-writing the metadata from memory afterwards.
 *
 * Both halves of that are the failure the issue warns about. The rename is
 * silent: [dev.slimevr.replay.CorpusRecording.discover] matches `*.pfr` and
 * ignores everything else, so a `.pfs` dropped into the corpus directory is not
 * an error, it simply is not there. And hand-writing the metadata is how
 * `imu_type` and `stay_aligned.*` go missing, which does not fail either -- the
 * replay completes with yaw correction switched off for its entire length.
 *
 * So this reads every field it can off the running server, and verifies its own
 * output by parsing it back through the same [CorpusMetadata.parse] the replay
 * suite will call. A capture that could not be replayed fails here, while the
 * wearer is still wearing the trackers, rather than in a pull request weeks
 * later.
 *
 * ## What it deliberately does not do
 *
 * It does not invent recordings. Everything here runs against live trackers and
 * a real wearer; there is no synthetic path, because a corpus exists to supply
 * the sensor noise, drift and mounting error synthetic motion cannot, and the
 * `capturer` and `consent` fields would be a fabricated attestation about a
 * real person.
 */
class CorpusCapture(private val server: VRServer) {
	/**
	 * One recorder for the lifetime of this object, not one per capture.
	 *
	 * [PoseRecorder] registers a tick callback in its constructor and
	 * [VRServer.addOnTick] has no counterpart to remove it, so a recorder built
	 * per capture would leave one behind on every recording.
	 */
	private val recorder by lazy { PoseRecorder(server) }

	/**
	 * A capture that has been written to disk, with what was derived for it.
	 *
	 * Returned rather than logged so a caller can report the sidecar back to the
	 * operator: the derived fields are the ones nobody typed, and the moment to
	 * notice that Stay Aligned was off at capture is while the session is still
	 * running.
	 */
	class Result(
		val name: String,
		val pfr: File,
		val meta: File,
		val frames: Int,
		val trackers: Int,
		val derived: CorpusMetadataWriter.Derived,
		/**
		 * Raw sample sidecars, one per sensor that produced any.
		 *
		 * Empty when no tracker streamed -- which is the state for any firmware
		 * without kmatzen/SlimeVR-Tracker-ESP#23, and leaves the recording
		 * fused-only and therefore pinned to the firmware that made it.
		 */
		val raw: List<File>,
		val rawSamples: Int,
		/**
		 * Conditions worth the operator's attention that are not errors -- a
		 * recording with Stay Aligned off, or one whose trackers reported no IMU.
		 * Both produce a valid recording that answers fewer questions than the
		 * session was probably held to answer.
		 */
		val warnings: List<String>,
	)

	/**
	 * Records for [seconds] at [rateHz] and writes both files into [outputDir].
	 *
	 * Blocks until the recording completes. [onProgress] is called with the
	 * frame count and total as it runs.
	 */
	@JvmOverloads
	fun capture(
		name: String,
		seconds: Float,
		rateHz: Float,
		outputDir: File,
		attestation: CorpusMetadataWriter.Attestation,
		onProgress: ((Int, Int) -> Unit)? = null,
	): Result {
		// Everything that can be rejected without a wearer standing still is
		// rejected first. A capture session is expensive and a rejected sidecar
		// is cheap to avoid.
		requireValidName(name)
		require(seconds > 0f && seconds.isFinite()) { "seconds must be positive, got $seconds" }
		require(rateHz > 0f && rateHz.isFinite()) { "rate must be positive, got $rateHz" }
		attestation.validate()

		val recordable = server.allTrackers.filter { !it.isInternal }
		require(recordable.isNotEmpty()) {
			"no trackers to record: connect trackers before capturing"
		}

		require(outputDir.isDirectory || outputDir.mkdirs()) {
			"could not create output directory ${outputDir.absolutePath}"
		}
		val pfr = File(outputDir, "$name.pfr")
		val meta = File(outputDir, "$name.meta")
		require(!pfr.exists() && !meta.exists()) {
			"${pfr.name} or ${meta.name} already exists in ${outputDir.absolutePath}. " +
				"Refusing to overwrite a capture: pick another name, or move the " +
				"existing pair out of the way"
		}

		// Snapshot before recording, so the sidecar describes the state the
		// recording was made under rather than whatever the server holds by the
		// time the file is written.
		val derived = derive(recordable, rateHz)

		val interval = 1f / rateHz
		val numFrames = Math.round(seconds * rateHz)
		require(numFrames >= 1) {
			"$seconds s at $rateHz Hz rounds to $numFrames frames; record for longer"
		}

		LogManager.info(
			"[CorpusCapture] Recording '$name': $numFrames frames at $rateHz Hz " +
				"(${"%.1f".format(seconds)} s) from ${recordable.size} trackers",
		)

		// Raw capture runs for exactly the recorded interval. Started before the
		// recorder so the first fused frame already has raw samples beside it,
		// and stopped in a finally so a failed capture does not leave every
		// tracker streaming into a collector nobody will read.
		val collector = server.trackersServer.rawSampleCollector
		server.trackersServer.setRawSampleStreaming(true)

		val future = recorder.startFrameRecording(numFrames, interval, recordable) { progress ->
			onProgress?.invoke(progress.frame, progress.totalFrames)
		}

		// Generous relative to the recording itself: the bound exists so a
		// server that stops ticking does not hang the console thread forever,
		// not to police timing.
		val frames = try {
			future.get((seconds * 2f).toLong() + 30L, TimeUnit.SECONDS)
		} finally {
			server.trackersServer.setRawSampleStreaming(false)
		}

		PfrIO.writeToFile(pfr, frames)
		meta.writeText(CorpusMetadataWriter.render(name, attestation, derived))

		// One .imu per sensor, since each has its own nominal timeline and its
		// own loss accounting. Merging them would make both uninterpretable.
		val rawFiles = mutableListOf<File>()
		for ((key, capture) in collector.results()) {
			if (capture.sampleCount == 0) continue
			val suffix = if (collector.results().size == 1) "" else ".${key.deviceId}-${key.sensorId}"
			val imu = File(outputDir, "$name$suffix.imu")
			ImuLogWriter.write(imu, capture)
			rawFiles += imu
		}

		verify(name, pfr, meta, frames.maxFrameCount, frames.frameHolders.size)

		return Result(
			name = name,
			pfr = pfr,
			meta = meta,
			frames = frames.maxFrameCount,
			trackers = frames.frameHolders.size,
			derived = derived,
			raw = rawFiles,
			rawSamples = collector.sampleCount,
			warnings = warningsFor(derived) +
				timestampWarnings(frames) +
				rawWarnings(collector.results().values, collector.unscalableBatches),
		)
	}

	/**
	 * Reads the pair back and parses it the way the replay suite will.
	 *
	 * The point of doing this here rather than trusting the write: every failure
	 * this catches is one that would otherwise surface long after the wearer has
	 * taken the trackers off, at which point the recording cannot be fixed, only
	 * discarded.
	 */
	private fun verify(name: String, pfr: File, meta: File, frames: Int, trackers: Int) {
		val reread = try {
			PfrIO.readFromFile(pfr)
		} catch (e: Exception) {
			throw IllegalStateException(
				"wrote ${pfr.name} but could not read it back: ${e.message}",
				e,
			)
		}
		check(reread.frameHolders.size == trackers && reread.maxFrameCount == frames) {
			"${pfr.name} did not survive the round trip: recorded $trackers trackers " +
				"x $frames frames, read back ${reread.frameHolders.size} x ${reread.maxFrameCount}"
		}

		val parsed = try {
			CorpusMetadata.parse(name, meta.readText())
		} catch (e: Exception) {
			throw IllegalStateException(
				"wrote ${meta.name} but the corpus loader rejects it: ${e.message}",
				e,
			)
		}
		check(parsed.rateHz > 0f) { "${meta.name} parsed with a non-positive rate" }
	}

	/**
	 * Reads the capture conditions off the running server.
	 *
	 * Each field here is one a person would otherwise transcribe by hand, and
	 * the two that matter most -- the IMU type and the relaxed poses -- are the
	 * two whose absence fails silently at replay.
	 */
	private fun derive(trackers: List<Tracker>, rateHz: Float): CorpusMetadataWriter.Derived {
		val imuTypes = trackers.mapNotNull { it.imuType }
		val byCount = imuTypes.groupingBy { it }.eachCount()
		val dominant = byCount.maxByOrNull { it.value }?.key
		val breakdown = if (byCount.size > 1) {
			byCount.entries.sortedByDescending { it.value }
				.joinToString { "${it.key.name} x${it.value}" }
		} else {
			null
		}

		val placement = trackers
			.map { it.trackerPosition?.name ?: it.name }
			.sorted()
			.joinToString()
		val firmware = trackers.mapNotNull { it.device?.firmwareVersion }
			.distinct()
			.sorted()
			.takeIf { it.isNotEmpty() }
			?.joinToString()

		val hpm = server.humanPoseManager
		val offsets = SkeletonConfigOffsets.values.associateWith { hpm.getOffset(it) }

		// Written only when the wearer actually had it on: any stay_aligned.* key
		// switches the correction on for the replay, so emitting the poses of a
		// disabled config would describe a recording that was never made.
		val stayAlignedConfig = server.configManager.vrConfig.stayAlignedConfig
		val stayAligned = stayAlignedConfig.takeIf { it.enabled }

		return CorpusMetadataWriter.Derived(
			rateHz = rateHz,
			captured = LocalDate.now().toString(),
			trackers = "${trackers.size} -- $placement",
			firmware = firmware,
			imuType = dominant,
			imuTypeBreakdown = breakdown,
			offsets = offsets,
			stayAligned = stayAligned,
		)
	}

	/**
	 * Whether the recording carries the sample timestamps time alignment needs.
	 *
	 * The same class of gap as a missing `imu_type`, and it fails the same way:
	 * a recording with no timestamps replays with every tracker apparently
	 * sampled at the same instant, `TimeAlignment` finds fewer than two
	 * participants, and the pass returns having touched nothing. Every metric is
	 * still produced.
	 *
	 * Unlike `imu_type` this cannot be repaired by editing the sidecar
	 * afterwards, because the data was never written down -- which is why it is
	 * reported while the session is still running.
	 */
	private fun timestampWarnings(frames: PoseFrames): List<String> {
		val timestamped = frames.frameHolders.count { holder ->
			holder.frames.any { it?.tryGetSampleServerMicros() != null }
		}
		if (timestamped == frames.frameHolders.size) return emptyList()

		val none = timestamped == 0
		return listOf(
			if (none) {
				"No tracker reported sample timestamps, so this recording carries " +
					"none. Time alignment will be inert for the whole replay. The " +
					"firmware must advertise PROTOCOL_SAMPLE_TIMESTAMPS -- this is " +
					"not recoverable after the session."
			} else {
				"$timestamped of ${frames.frameHolders.size} trackers reported sample " +
					"timestamps. The rest replay as untimestamped and take no part in " +
					"time alignment, exactly as they would live."
			},
		)
	}

	/**
	 * What the raw capture did or did not manage.
	 *
	 * The absent case is the important one, and it is not an error: a recording
	 * with no raw samples is a perfectly good regression baseline for the
	 * server. What it cannot do is survive a firmware change -- the fused output
	 * it holds was produced by one VQF configuration, one error model and one
	 * set of rest-detection thresholds, and nothing in the file says what they
	 * were. That is worth saying out loud while the session is still running,
	 * because it is the difference between recording once and re-shooting.
	 */
	private fun rawWarnings(
		captures: Collection<RawSampleCapture>,
		unscalableBatches: Long,
	): List<String> {
		val warnings = mutableListOf<String>()

		if (captures.isEmpty()) {
			warnings += "No tracker streamed raw samples, so this recording is " +
				"fused-only and stays tied to the firmware that made it. Needs " +
				"firmware with raw sample streaming " +
				"(kmatzen/SlimeVR-Tracker-ESP#23) -- not recoverable afterwards."
			return warnings
		}

		if (unscalableBatches > 0) {
			warnings += "$unscalableBatches raw batches arrived before the stream " +
				"metadata that scales them and were discarded. Expected only at the " +
				"very start of a capture; a large count means the metadata packet " +
				"is not getting through."
		}

		for (capture in captures) {
			if (capture.isComplete) continue
			// Named rather than summarised, because the two causes have
			// different fixes: the tracker could not send fast enough, or the
			// network dropped it.
			warnings += "Raw capture for ${capture.sensorName} has gaps -- " +
				"${capture.accel.summary()}; ${capture.gyro.summary()}. The .imu " +
				"file marks them, so it is still usable, but a re-fusion run " +
				"cannot cross a gap."

			// Called out separately because it is the one cause the wearer can
			// do nothing about and the one that means the tracker itself could
			// not keep up. It is also the only cause that would have been
			// invisible before kmatzen/SlimeVR-Tracker-ESP#26.
			val fifo = capture.accel.droppedInFifo + capture.gyro.droppedInFifo
			if (fifo > 0) {
				warnings += "$fifo of those were discarded by the sensor's own FIFO " +
					"before the firmware saw them, which means the tracker's drain " +
					"loop could not keep up. Not a network problem, and not fixable " +
					"by recapturing on a quieter WiFi channel."
			}
		}

		return warnings
	}

	/**
	 * Conditions that make a recording answer less than it looks like it does.
	 *
	 * Not errors: a recording with no IMU type is still a valid recording of leg
	 * behaviour. They are surfaced because the alternative is discovering in a
	 * pull request that a session held to settle issue #3 replayed with yaw
	 * correction switched off throughout.
	 */
	private fun warningsFor(derived: CorpusMetadataWriter.Derived): List<String> {
		val warnings = mutableListOf<String>()
		if (derived.imuType == null) {
			warnings += "No tracker reported an IMU type, so imu_type is not recorded. " +
				"Stay Aligned will be inert for the whole replay -- this recording " +
				"cannot say anything about yaw correction."
		}

		val stayAligned = derived.stayAligned
		if (stayAligned == null) {
			warnings += "Stay Aligned was disabled at capture, so no relaxed poses are " +
				"recorded. Fine for a leg-correction recording; useless for issue #3."
		} else if (!stayAligned.standingRelaxedPose.enabled &&
			!stayAligned.sittingRelaxedPose.enabled &&
			!stayAligned.flatRelaxedPose.enabled
		) {
			warnings += "Stay Aligned is on but every relaxed pose is disabled. " +
				"RelaxedPose.forPose returns null for all of them, so no centring " +
				"force is applied at replay."
		}
		return warnings
	}

	private fun requireValidName(name: String) {
		require(name.isNotBlank()) { "recording name must not be blank" }
		require(name.matches(NAME_PATTERN)) {
			"recording name '$name' must be lowercase letters, digits and dashes: " +
				"it becomes a filename and a baseline key prefix"
		}
	}

	companion object {
		private val NAME_PATTERN = Regex("[a-z0-9]+(-[a-z0-9]+)*")

		/** Sample rate used when the operator does not name one. */
		const val DEFAULT_RATE_HZ = 100f
	}
}
