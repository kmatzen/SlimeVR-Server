package dev.slimevr.replay

import dev.slimevr.metrics.PoseMetrics
import dev.slimevr.metrics.PoseMetricsAccumulator
import dev.slimevr.metrics.SegmentConsistency
import dev.slimevr.metrics.SegmentConsistencyAccumulator
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerRole
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.assertAll
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Dataset-driven regression tests for the skeleton pipeline.
 *
 * A synthetic motion sequence is replayed through a real [HumanPoseManager] and
 * reduced to pose-quality metrics, which are then compared against committed
 * baselines. This is the piece that lets a change to the solver or to
 * `LegTweaks` be argued about with numbers instead of impressions.
 *
 * Both configurations are covered: the plain solver, and the solver with the
 * leg corrections engaged. The latter used to be reportable but not gateable,
 * because `LegTweaksBuffer` stamped every frame with `System.nanoTime()` and so
 * derived velocities from real elapsed time. It now takes its time from an
 * injected [dev.slimevr.tracking.processor.skeleton.FrameClock], which replay
 * drives from the sequence's own timestep -- see [FixedStepClock].
 *
 * See [ReplayBaseline] for the baseline file and how to regenerate it.
 */
class SkeletonReplayTest {

	private val rateHz = 100f
	private val frames = 400

	/**
	 * The configurations each baselined motion is replayed under.
	 *
	 * The suffix is part of the baseline key, so the two configurations produce
	 * separate, individually attributable lines in `replay-baseline.txt`.
	 */
	private val configurations = listOf(
		"" to false,
		"+legtweaks" to true,
	)

	/**
	 * The core property a regression suite depends on: the same input must
	 * produce the same output. If this fails, every baseline below is noise.
	 */
	@Test
	fun replayIsDeterministic() {
		for (motion in SyntheticMotion.names) {
			val a = replay(motion)
			val b = replay(motion)
			assertEquals(
				a.toMap(),
				b.toMap(),
				"replay of '$motion' is not reproducible; baselines are meaningless until it is",
			)
		}
	}

	/**
	 * The same property for the leg corrections, which is the case that used to
	 * fail.
	 *
	 * `LegTweaksBuffer` derives foot velocities from the interval between
	 * consecutive frames. While that interval came from `System.nanoTime()`, two
	 * replays of byte-identical input did not agree -- measured drift was on the
	 * order of 1e-4 m/s of foot slide on the machine that first recorded it, and
	 * a few times 1e-6 on others. The magnitude was never the point; it was set
	 * by machine speed and load rather than by anything about the code under
	 * test, which put a floor under how tight any baseline on these metrics
	 * could be.
	 *
	 * With the clock injected, the interval is the sequence's timestep and the
	 * spread is exactly zero, so this is an equality assertion and the corrected
	 * metrics are in the committed baseline alongside the uncorrected ones.
	 */
	@Test
	fun replayIsDeterministicWithLegTweaks() {
		val a = replay("squat", enableSkatingCorrection = true)
		val b = replay("squat", enableSkatingCorrection = true)

		val aMap = a.toMap()
		val bMap = b.toMap()
		val drift = aMap.keys.associateWith { key ->
			abs((aMap[key] ?: 0f) - (bMap[key] ?: 0f))
		}

		assertEquals(
			aMap,
			bMap,
			"replay with the leg corrections engaged is not reproducible. This is " +
				"what the injected FrameClock exists to prevent -- check that " +
				"nothing in the corrected path has gone back to reading the " +
				"system clock. Drift: $drift",
		)
	}

	/**
	 * Confirms the skating correction is actually engaged, so that the test
	 * above is not vacuously passing on a disabled code path.
	 *
	 * The `squat` sequence lowers the headset while bending the knees, which
	 * drives the ankles through the floor when nothing corrects for it. If
	 * enabling the correction does not change the result, it is not running and
	 * every conclusion drawn from it is worthless.
	 */
	@Test
	fun skatingCorrectionChangesTheResult() {
		val off = replay("squat", enableSkatingCorrection = false)
		val on = replay("squat", enableSkatingCorrection = true)

		println("squat floor clip: off=${off.floorClipMaxM} m, on=${on.floorClipMaxM} m")
		println("squat foot slide: off=${off.footSlideMPerSec} m/s, on=${on.footSlideMPerSec} m/s")

		assertTrue(
			off.toMap() != on.toMap(),
			"enabling the skating correction changed nothing -- it is not running, " +
				"so any test that exercises it is vacuous",
		)
	}

	/**
	 * A frame that is not separated in time from its parent carries no velocity
	 * information. `getTimeDelta()` returns 1/dt, so a zero interval would put an
	 * infinity into every threshold comparison in `LegTweaksBuffer` and force the
	 * feet permanently unlocked.
	 *
	 * `System.nanoTime()` made that all but unreachable; an injected clock makes
	 * it reachable, so it is handled rather than assumed away. This pins the
	 * handling.
	 */
	@Test
	fun aStalledClockDoesNotProduceInfiniteVelocities() {
		val stalled = FixedStepClock(0L)
		val metrics = replay("squat", enableSkatingCorrection = true, clock = stalled)

		for ((name, value) in metrics.toMap()) {
			assertTrue(
				value.isFinite(),
				"metric '$name' is $value with a stalled clock; a zero frame " +
					"interval has leaked an infinity into the corrections",
			)
		}
	}

	@Test
	fun metricsMatchBaseline() {
		val baseline = ReplayBaseline.load()
		val failures = mutableListOf<String>()
		val report = StringBuilder()
		val measured = linkedMapOf<String, Float>()

		for (motion in SyntheticMotion.names) {
			for ((suffix, legTweaks) in configurations) {
				val metrics = replay(motion, enableSkatingCorrection = legTweaks)
				for ((metric, value) in metrics.toMap()) {
					val key = "$motion$suffix/$metric"
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

		println("%-44s %12s %12s %12s".format("metric", "baseline", "current", "delta"))
		println(report)

		// Documented in ReplayBaseline: run with -Dreplay.writeBaseline=true and
		// copy the emitted block over the resource file.
		if (System.getProperty("replay.writeBaseline") == "true") {
			println("--- BEGIN replay-baseline.txt ---")
			println(ReplayBaseline.format(measured))
			println("--- END replay-baseline.txt ---")
		}

		assertAll(failures.map { { throw AssertionError(it) } })
	}

	/**
	 * Sanity checks that hold regardless of the baseline, so that an
	 * accidentally regenerated baseline cannot quietly bless nonsense.
	 */
	@Test
	fun standingPoseIsPhysicallyPlausible() {
		val metrics = replay("stand")

		assertTrue(
			metrics.footSlideMPerSec < 0.01f,
			"a motionless skeleton slid its feet at ${metrics.footSlideMPerSec} m/s",
		)
		assertTrue(
			metrics.floorClipMaxM < 0.05f,
			"a standing skeleton clipped ${metrics.floorClipMaxM} m through the floor",
		)
	}

	/**
	 * The `lean` sequence rotates the chest and hip while the headset stays
	 * pinned, so the feet necessarily swing -- the skeleton hangs from the head
	 * and the legs are the far end of the chain. That is correct kinematics for
	 * this input, not a defect, and it is why there is no upper bound asserted
	 * on foot slide here.
	 *
	 * It is kept as a baseline case precisely because it is sensitive: any
	 * change to how upper-body rotation propagates down the chain moves this
	 * number.
	 */
	@Test
	fun leanPropagatesThroughTheChain() {
		val metrics = replay("lean")
		assertTrue(
			metrics.footSlideMPerSec > 0f,
			"upper-body rotation with a pinned headset should move the feet",
		)
	}

	/**
	 * One replay's numbers: where the output is, and whether it is a body.
	 *
	 * [SegmentConsistency] is folded in here rather than left to
	 * [SegmentConsistencyTest] so that it reaches the committed baseline, which
	 * is issue #4's first suggested step. It matters more than one more metric
	 * usually would.
	 *
	 * Almost every `+legtweaks` line in `replay-baseline.txt` is `0.000000`,
	 * because the corrections fully absorb clean synthetic input. Those lines
	 * gate *"the corrections still work at all"* and nothing finer -- there is no
	 * headroom left to express degradation, which is the argument issue #15
	 * makes for needing recordings.
	 *
	 * Deformation is the exception, and it is the exception for a structural
	 * reason rather than a lucky one: the corrections drive slide and clip to
	 * zero *by* moving joints independently, so the very frames where those read
	 * zero are the frames where this reads largest -- up to 0.16 m on `squat`,
	 * 11% of a segment. It is the one channel in the suite where a change that
	 * made the leg path meaningfully worse can move a number today, without a
	 * corpus.
	 */
	private class ReplayResult(
		val pose: PoseMetrics,
		val consistency: SegmentConsistency,
	) {
		val footSlideMPerSec: Float get() = pose.footSlideMPerSec
		val floorClipMaxM: Float get() = pose.floorClipMaxM

		fun toMap(): Map<String, Float> = pose.toMap() +
			linkedMapOf(
				"segment_deformation_mean_m" to consistency.meanViolationM,
				"segment_deformation_max_m" to consistency.maxViolationM,
				"segment_deformation_fraction" to consistency.meanViolationFraction,
			)
	}

	private fun replay(
		motion: String,
		enableSkatingCorrection: Boolean = false,
		clock: FixedStepClock = FixedStepClock(1f / rateHz),
	): ReplayResult {
		val hmd = mkTracker(0, TrackerPosition.HEAD, isHmd = true)
		val chest = mkTracker(1, TrackerPosition.CHEST)
		val hip = mkTracker(2, TrackerPosition.HIP)
		val leftThigh = mkTracker(3, TrackerPosition.LEFT_UPPER_LEG)
		val leftCalf = mkTracker(4, TrackerPosition.LEFT_LOWER_LEG)
		val rightThigh = mkTracker(5, TrackerPosition.RIGHT_UPPER_LEG)
		val rightCalf = mkTracker(6, TrackerPosition.RIGHT_LOWER_LEG)

		val trackers = listOf(hmd, chest, hip, leftThigh, leftCalf, rightThigh, rightCalf)
		val hpm = HumanPoseManager(trackers)
		val height = hpm.userHeightFromConfig
		hpm.skeleton.hasKneeTrackers = true

		// setLegTweaksEnabled is only the master switch; the individual
		// corrections are separate toggles and default to off. Setting just the
		// master switch changes nothing, which is how this test was vacuous
		// before -- see skatingCorrectionChangesTheResult.
		hpm.setLegTweaksEnabled(enableSkatingCorrection)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, enableSkatingCorrection)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, enableSkatingCorrection)

		// After the toggles: each of them resets the frame buffer, and so does
		// assigning the clock. Installing it last guarantees no frame stamped by
		// the system clock survives into the replay.
		hpm.skeleton.legTweaks.clock = clock.clock

		// The heading shadow reports on a wall-clock window, which under replay
		// would make what it logs depend on how fast the host ran. It corrects
		// nothing, so this cannot move a baseline -- but the whole point of the
		// injected clock is that a replay says the same thing every time.
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val accumulator = PoseMetricsAccumulator()
		val consistency = SegmentConsistencyAccumulator()
		val dt = 1f / rateHz

		for (frame in SyntheticMotion.sequence(motion, frames, rateHz)) {
			// Advance before the update so that the frame this update produces is
			// exactly one timestep after the previous one.
			clock.advance()

			hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(frame.leftCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			hpm.update()

			consistency.observe(hpm.skeleton)

			// Measure the computed foot trackers rather than the skeleton's own
			// bones: that is the pipeline's actual output, and it is the only
			// place the LegTweaks corrections are visible.
			accumulator.observeAnkles(
				hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position,
				hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position,
				dt,
			)
		}

		return ReplayResult(accumulator.result(height), consistency.result())
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
