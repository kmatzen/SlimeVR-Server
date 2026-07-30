package dev.slimevr.replay

import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.poseframeformat.PfrIO
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.poseframeformat.trackerdata.TrackerFrame
import dev.slimevr.poseframeformat.trackerdata.TrackerFrames
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.stayaligned.StayAligned
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import dev.slimevr.tracking.trackers.udp.IMUType
import io.eiren.util.collections.FastList
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertAll
import org.junit.jupiter.api.io.TempDir
import java.io.File
import java.nio.file.Path
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Replays the committed `.pfr` corpus through the pipeline and gates it on the
 * same baseline machinery as the synthetic suite.
 *
 * ## The corpus is currently empty, and these tests still do work
 *
 * There are no recordings in the repository yet -- capturing them needs
 * hardware and a wearer, which is the part of issue #15 that cannot be written.
 * Everything around them can be, and is: discovery, metadata validation, the
 * replay driver, the baseline keys, the CI table.
 *
 * Wiring that nothing exercises is wiring that rots. [pfrRoundTripReproducesTheSyntheticBaseline]
 * is what keeps this path honest in the meantime -- it pushes a synthetic
 * sequence out through [PfrIO], reads it back, replays it through the corpus
 * driver, and requires the result to match the *committed synthetic baseline*
 * for that same sequence. That exercises the serialiser, the player, the
 * corpus driver and the metric reduction end to end, and it will fail if any of
 * them breaks before the first real recording lands.
 *
 * It is not a substitute for the corpus. It cannot be: it is the same clean
 * synthetic motion, so it has none of the sensor noise, drift, mounting error
 * or dropout that recordings exist to supply. It proves the path works, not
 * that the pipeline handles reality.
 */
class CorpusReplayTest {

	/**
	 * Recordings are only usable if their sidecars parse and carry the required
	 * provenance. [CorpusRecording.discover] throws on a bad one; this makes
	 * that a named failure rather than an error inside an unrelated test, and
	 * prints the inventory so a reviewer can see what the corpus contains.
	 */
	@Test
	fun everyRecordingDeclaresItsCaptureMetadata() {
		val dir = CorpusRecording.directory()
		val recordings = CorpusRecording.discover()

		println("corpus directory: ${dir?.path ?: "(not on the test classpath)"}")
		if (recordings.isEmpty()) {
			println(
				"corpus is empty -- no .pfr recordings committed yet. The replay " +
					"path is covered by pfrRoundTripReproducesTheSyntheticBaseline " +
					"until captures land. See corpus/README.md to add one.",
			)
			return
		}

		println("corpus recordings (${recordings.size}):")
		for (recording in recordings) {
			// Prefixed so one `^corpus` pattern picks up the inventory, the
			// metric table and the empty-corpus notice in the CI summary.
			println("corpus: ${recording.summary()}")
		}
	}

	/**
	 * The property every baseline depends on, for the corpus path.
	 *
	 * Vacuous while the corpus is empty, and deliberately kept anyway: the day
	 * a recording is added, this is the test that says whether replaying it
	 * twice gives the same answer, and nobody should have to remember to write
	 * it then.
	 */
	@Test
	fun corpusReplayIsDeterministic() {
		for (recording in CorpusRecording.discover()) {
			val frames = recording.load()
			for ((suffix, legTweaks) in CorpusReplay.configurations) {
				val a = CorpusReplay.replay(
					frames,
					recording.rateHz,
					legTweaks,
					recording.offsets,
					recording.imuType,
					recording.stayAligned,
				)
				val b = CorpusReplay.replay(
					frames,
					recording.rateHz,
					legTweaks,
					recording.offsets,
					recording.imuType,
					recording.stayAligned,
				)
				assertEquals(
					a.toMap(),
					b.toMap(),
					"replay of corpus recording '${recording.name}$suffix' is not " +
						"reproducible; its baseline is meaningless until it is",
				)
			}
		}
	}

	/**
	 * End-to-end proof of the corpus path, using the committed synthetic
	 * baseline as ground truth.
	 *
	 * `squat` is driven into live trackers exactly as [SkeletonReplayTest]
	 * drives it, recorded frame by frame into a [PoseFrames], written to a real
	 * `.pfr` file, read back, and replayed through [CorpusReplay]. The metrics
	 * must land on the `squat/...` and `squat+legtweaks/...` entries already in
	 * `replay-baseline.txt`.
	 *
	 * Two different things have to hold for that to pass, which is why it is
	 * worth asserting against the committed file rather than against a second
	 * in-process run:
	 *
	 * - **The format is lossless for what the pipeline consumes.** Rotations
	 *   and positions are written as IEEE floats and read back bit-identically,
	 *   so a round trip may not perturb a single metric.
	 * - **The corpus driver is equivalent to the synthetic driver.** Trackers
	 *   reconstructed by `TrackerFrames.toTracker()` are not configured
	 *   identically to the ones [SkeletonReplayTest] builds by hand -- notably
	 *   the reconstructed head tracker is not flagged `isHmd`. If any of those
	 *   differences reached the solved pose, this fails, and a corpus metric
	 *   would not be comparable to a synthetic one.
	 */
	@Test
	fun pfrRoundTripReproducesTheSyntheticBaseline(@TempDir tmp: Path) {
		val motion = "squat"
		val rateHz = 100f
		val frameCount = 400

		val baseline = ReplayBaseline.load()
		assertTrue(
			baseline.keys.any { it.startsWith("$motion/") },
			"no committed baseline for '$motion'; this test uses it as ground truth",
		)

		val file = File(tmp.toFile(), "$motion.pfr")
		PfrIO.writeToFile(file, recordSynthetic(motion, frameCount, rateHz))
		val readBack = PfrIO.readFromFile(file)

		assertEquals(
			frameCount,
			readBack.maxFrameCount,
			"round trip lost frames",
		)

		val failures = mutableListOf<String>()
		for ((suffix, legTweaks) in CorpusReplay.configurations) {
			val metrics = CorpusReplay.replay(readBack, rateHz, legTweaks)
			for ((metric, value) in metrics.toMap()) {
				val entry = baseline["$motion$suffix/$metric"] ?: continue
				val delta = abs(value - entry.value)
				println(
					"%-40s baseline %12.6f  via .pfr %12.6f  delta %12.6f".format(
						"$motion$suffix/$metric",
						entry.value,
						value,
						value - entry.value,
					),
				)
				if (delta > entry.tolerance) {
					failures.add(
						"$motion$suffix/$metric: synthetic baseline ${entry.value}, " +
							"same motion via .pfr $value (tolerance ${entry.tolerance})",
					)
				}
			}
		}

		assertAll(failures.map { { throw AssertionError(it) } })
	}

	/**
	 * A replayed recording can actually run Stay Aligned.
	 *
	 * This is the property the corpus is being captured for on issue #3 -- the
	 * open question there is Stay Aligned against the kinematic heading solve on
	 * the same session -- and until now the corpus path could not have answered
	 * it. `TrackerFrames.toTracker()` built trackers with no IMU type, so
	 * `Tracker.isImu()` was false and `AdjustTrackerYaw.adjust` returned before
	 * touching anything. Every recording would have replayed with yaw correction
	 * silently switched off, produced a full set of metrics, and looked fine.
	 *
	 * Checked by difference rather than by inspecting internals: replay the same
	 * frames with the correction configured and not, and require the pose to move.
	 * If it does not, Stay Aligned is not running, whatever the configuration
	 * says.
	 */
	@Test
	fun aRecordingCanDriveStayAligned(@TempDir tmp: Path) {
		val rateHz = 100f
		val frameCount = 400
		val file = File(tmp.toFile(), "walk.pfr")
		PfrIO.writeToFile(file, recordSynthetic("walk-in-place", frameCount, rateHz))
		val frames = PfrIO.readFromFile(file)

		val relaxedPose = StayAlignedConfig().apply {
			enabled = true
			standingRelaxedPose.enabled = true
		}

		val inert = CorpusReplay.replay(frames, rateHz, false)

		val corrected = CorpusReplay.replay(
			frames,
			rateHz,
			false,
			emptyMap(),
			IMUType.BMI270,
			relaxedPose,
		)

		println("stay aligned off: ${inert.toMap()}")
		println("stay aligned on:  ${corrected.toMap()}")

		assertTrue(
			inert.toMap() != corrected.toMap(),
			"declaring an IMU type and a relaxed pose changed nothing, so Stay " +
				"Aligned is still not running on replayed recordings. The corpus " +
				"cannot answer issue #3's question in that state.",
		)
	}

	@Test
	fun corpusMetricsMatchBaseline() {
		val recordings = CorpusRecording.discover()
		val baseline = ReplayBaseline.load()
		val failures = mutableListOf<String>()
		val report = StringBuilder()
		val measured = linkedMapOf<String, Float>()

		for (recording in recordings) {
			val frames = recording.load()
			for ((suffix, legTweaks) in CorpusReplay.configurations) {
				val metrics = CorpusReplay.replay(
					frames,
					recording.rateHz,
					legTweaks,
					recording.offsets,
					recording.imuType,
					recording.stayAligned,
				)
				for ((metric, value) in metrics.toMap()) {
					val key = CorpusReplay.key(recording.name, suffix, metric)
					measured[key] = value
					val entry = baseline[key]
					if (entry == null) {
						report.append("%-44s %12s %12.6f  (new)\n".format(key, "-", value))
						continue
					}
					val delta = value - entry.value
					val bad = abs(delta) > entry.tolerance
					report.append(
						"%-44s %12.6f %12.6f %12.6f %s\n".format(
							key,
							entry.value,
							value,
							delta,
							if (bad) "REGRESSION" else "",
						),
					)
					if (bad) {
						failures.add("$key: baseline ${entry.value}, got $value (tolerance ${entry.tolerance})")
					}
				}
			}
		}

		if (recordings.isNotEmpty()) {
			println("%-44s %12s %12s %12s".format("metric", "baseline", "current", "delta"))
			println(report)
		}

		// Same regeneration switch as SkeletonReplayTest. Corpus keys are
		// appended to the same baseline file; see corpus/README.md for the
		// per-recording block policy.
		if (System.getProperty("replay.writeBaseline") == "true" && measured.isNotEmpty()) {
			println("--- BEGIN replay-baseline.txt (corpus) ---")
			println(ReplayBaseline.format(measured))
			println("--- END replay-baseline.txt (corpus) ---")
		}

		assertAll(failures.map { { throw AssertionError(it) } })
	}

	/**
	 * A baseline key naming a recording that is no longer committed is a
	 * metric silently no longer being checked. Catching it here means removing
	 * a recording forces the same deliberate baseline edit that adding one
	 * does.
	 */
	@Test
	fun everyCorpusBaselineKeyHasARecording() {
		val present = CorpusRecording.discover().map { it.name }.toSet()
		val orphaned = ReplayBaseline.load().keys
			.filter { it.startsWith("corpus:") }
			.map { it.removePrefix("corpus:").substringBefore('/').removeSuffix("+legtweaks") }
			.distinct()
			.filterNot { it in present }

		assertTrue(
			orphaned.isEmpty(),
			"replay-baseline.txt has corpus entries for recording(s) ${orphaned.joinToString()} " +
				"that are not in the corpus. Either the recording was removed without " +
				"dropping its baseline block, or its .pfr failed to load.",
		)
	}

	/**
	 * Drives a synthetic sequence into live trackers and captures each frame,
	 * mirroring the tracker set and drive loop in [SkeletonReplayTest].
	 *
	 * This is how a real capture is produced too, minus the transport: the
	 * server's `PoseRecorder` calls the same `TrackerFrames.addFrameFromTracker`
	 * against whatever the trackers last reported.
	 */
	private fun recordSynthetic(motion: String, frameCount: Int, rateHz: Float): PoseFrames {
		val trackers = listOf(
			mkTracker(0, TrackerPosition.HEAD, isHmd = true),
			mkTracker(1, TrackerPosition.CHEST),
			mkTracker(2, TrackerPosition.HIP),
			mkTracker(3, TrackerPosition.LEFT_UPPER_LEG),
			mkTracker(4, TrackerPosition.LEFT_LOWER_LEG),
			mkTracker(5, TrackerPosition.RIGHT_UPPER_LEG),
			mkTracker(6, TrackerPosition.RIGHT_LOWER_LEG),
		)
		val hmd = trackers[0]
		val chest = trackers[1]
		val hip = trackers[2]
		val leftThigh = trackers[3]
		val leftCalf = trackers[4]
		val rightThigh = trackers[5]
		val rightCalf = trackers[6]

		// The head height is a fraction of user height, and user height comes
		// from the default skeleton offsets -- so it has to be read from a
		// manager configured the same way the replay will be.
		val height = HumanPoseManager(trackers).userHeightFromConfig

		val holders = trackers.map { TrackerFrames(it, FastList<TrackerFrame?>(frameCount)) }

		for (frame in SyntheticMotion.sequence(motion, frameCount, rateHz)) {
			hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(frame.leftCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			for ((i, tracker) in trackers.withIndex()) {
				holders[i].addFrameFromTracker(tracker)
			}
		}

		return PoseFrames(FastList(holders))
	}

	private fun mkTracker(
		id: Int,
		position: TrackerPosition,
		isHmd: Boolean = false,
	): Tracker {
		val tracker = Tracker(
			device = null,
			id = id,
			name = position.name,
			trackerPosition = position,
			trackerNum = 0,
			hasPosition = isHmd,
			hasRotation = true,
			isComputed = isHmd,
			imuType = null,
			allowReset = !isHmd,
			allowMounting = !isHmd,
			isHmd = isHmd,
			trackRotDirection = false,
		)
		tracker.status = TrackerStatus.OK
		return tracker
	}
}
