package dev.slimevr.replay

import dev.slimevr.metrics.PoseMetrics
import dev.slimevr.metrics.PoseMetricsAccumulator
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
 * See [ReplayBaseline] for the baseline file and how to regenerate it.
 */
class SkeletonReplayTest {

	private val rateHz = 100f
	private val frames = 400

	/**
	 * The core property a regression suite depends on: the same input must
	 * produce the same output. If this fails, every baseline below is noise.
	 *
	 * It currently passes only because the default toggles leave the
	 * wall-clock-dependent stages off. `LegTweaksBuffer.kt:180` stamps every
	 * frame with `System.nanoTime()` and `HumanPoseManager.kt:513` reads
	 * `System.currentTimeMillis()`, so those stages compute velocities from
	 * *real* elapsed time and cannot be replayed reproducibly. Extending the
	 * suite to cover them needs an injectable clock -- see
	 * [replayIsNotYetDeterministicWithLegTweaks].
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
	 * Quantifies the wall-clock dependence of the leg-correction stages.
	 *
	 * `LegTweaksBuffer.kt:180` stamps every frame with `System.nanoTime()` and
	 * `HumanPoseManager.kt:513` reads `System.currentTimeMillis()`, so the
	 * skating correction derives velocities from *real* elapsed time rather
	 * than from the frame's simulated timestep. Two replays of byte-identical
	 * input therefore need not agree, and in practice they do not: measured
	 * drift is on the order of 1e-4 m/s of foot slide, roughly 0.3% of the
	 * signal.
	 *
	 * Small, but not zero, and that is the whole problem -- it puts a floor
	 * under how tight any baseline on these metrics can be, and the floor is
	 * set by machine speed and load rather than by anything about the code
	 * under test. This is why [metricsMatchBaseline] covers only the
	 * deterministic configuration.
	 *
	 * The bound asserted here is a guard against the nondeterminism getting
	 * worse. When an injectable clock lands it should become an equality
	 * assertion and the skating-correction metrics should join the baseline.
	 */
	@Test
	fun clockDependentStagesAreOnlyApproximatelyReproducible() {
		val a = replay("squat", enableSkatingCorrection = true)
		val b = replay("squat", enableSkatingCorrection = true)

		val aMap = a.toMap()
		val bMap = b.toMap()
		val drift = aMap.keys.associateWith { key ->
			abs((aMap[key] ?: 0f) - (bMap[key] ?: 0f))
		}
		println("skating-correction replay drift between identical runs: $drift")

		val worst = drift.values.maxOrNull() ?: 0f
		assertTrue(
			worst < 0.01f,
			"replay nondeterminism has grown to $worst; the clock dependence in " +
				"LegTweaksBuffer is now large enough to matter. Drift: $drift",
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

	@Test
	fun metricsMatchBaseline() {
		val baseline = ReplayBaseline.load()
		val failures = mutableListOf<String>()
		val report = StringBuilder()

		for (motion in SyntheticMotion.names) {
			val metrics = replay(motion)
			for ((metric, value) in metrics.toMap()) {
				val key = "$motion/$metric"
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

		println("%-44s %12s %12s %12s".format("metric", "baseline", "current", "delta"))
		println(report)

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

	private fun replay(
		motion: String,
		enableSkatingCorrection: Boolean = false,
	): PoseMetrics {
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

		val accumulator = PoseMetricsAccumulator()
		val dt = 1f / rateHz

		for (frame in SyntheticMotion.sequence(motion, frames, rateHz)) {
			hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(frame.leftCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			hpm.update()

			// Measure the computed foot trackers rather than the skeleton's own
			// bones: that is the pipeline's actual output, and it is the only
			// place the LegTweaks corrections are visible.
			accumulator.observeAnkles(
				hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position,
				hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position,
				dt,
			)
		}

		return accumulator.result(height)
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
