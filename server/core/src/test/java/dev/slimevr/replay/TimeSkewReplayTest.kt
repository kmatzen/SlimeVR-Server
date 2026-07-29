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
import kotlin.math.abs
import kotlin.test.assertTrue

/**
 * What per-tracker clock skew costs, and how much of it interpolation recovers.
 *
 * Issue #2 asserted that fusing trackers sampled at different instants degrades
 * the pose, and that interpolating them to a common tick is "the step that
 * actually buys the accuracy". Both claims were plausible and neither had a
 * number attached. The issue's own instruction was to get one *before* writing
 * the correction, on the grounds that a small number is also worth knowing.
 *
 * ## The three runs
 *
 * Each configuration replays the same motion; only *when* each tracker sampled
 * it differs.
 *
 * - **reference** -- every tracker samples at `t - maxDelay`. A common-mode
 *   delay, which is input lag and nothing else: no two trackers disagree, so
 *   this is the pose the pipeline should produce. It is the target, not the
 *   unskewed run at `t`, because a correction that resolves everything to the
 *   laggiest tracker's instant cannot do better than this and should not be
 *   scored against something it is not trying to achieve.
 * - **skewed** -- tracker *k* samples at `t - delay(k)`, and the server is told
 *   nothing about it. This is what the server did before this change, and
 *   still does for firmware that cannot report sample timestamps.
 * - **aligned** -- the same skewed samples, but each carrying its instant, so
 *   `TimeAlignment` resolves them all back to a common tick.
 *
 * The gap between *skewed* and *reference* is what the skew costs. The gap
 * between *aligned* and *reference* is what is left after correcting it. The
 * second should be a small fraction of the first, and the residual is not
 * expected to be zero: slerp between two real samples is not the true motion
 * between them, so a sample interval's worth of interpolation error survives.
 *
 * ## Why the delays are not multiples of the frame interval
 *
 * If they were, every alignment lookup would land exactly on a stored sample
 * and the interpolation would never run. The delays below are deliberately
 * off-grid, which is also the realistic case -- nothing synchronises a
 * tracker's sampling to the server's solve.
 *
 * ## Why the headset is excluded
 *
 * The headset is a position source with no rotation history, so there is
 * nothing to interpolate it from; in production its samples arrive over an
 * entirely different transport and carry no tracker timestamp. Asserting on
 * motions with a static head height ([SyntheticMotion.staticHeadHeight])
 * removes that term rather than hiding it. It is a real limitation: an
 * un-timestamped headset leading the IMUs leaves a head-to-body offset that
 * alignment cannot correct, and the `squat` case below reports it instead of
 * asserting on it.
 */
class TimeSkewReplayTest {

	private val rateHz = 100f
	private val frames = 400

	/** 200 ms, enough for every tracker's history to bracket the reference. */
	private val warmupFrames = 20

	/**
	 * Per-tracker delay in milliseconds. Single-digit milliseconds of spread is
	 * the WiFi behaviour issue #2 describes; the values differ per tracker
	 * because the whole point is that the delays are independent.
	 */
	private val delaysMs = mapOf(
		TrackerPosition.CHEST to 1.3f,
		TrackerPosition.HIP to 3.7f,
		TrackerPosition.LEFT_UPPER_LEG to 6.1f,
		TrackerPosition.LEFT_LOWER_LEG to 11.4f,
		TrackerPosition.RIGHT_UPPER_LEG to 8.2f,
		TrackerPosition.RIGHT_LOWER_LEG to 14.9f,
	)

	private val maxDelayMs = delaysMs.values.max()

	/** Every tracker equally late: the pose alignment is trying to recover. */
	private val commonModeDelaysMs = delaysMs.mapValues { maxDelayMs }

	/**
	 * The configurations each motion is measured under, matching
	 * [SkeletonReplayTest].
	 *
	 * Both are needed and the plain solver is the one that carries the result.
	 * With the leg corrections engaged, synthetic input drives almost every
	 * metric to exactly zero -- the heuristics fully absorb motion this clean,
	 * which leaves nothing for skew to degrade and nothing for alignment to
	 * recover. That is a property of the input, not evidence that skew is
	 * harmless: the corrected column here measures how much damage survives
	 * the heuristics, and the uncorrected column measures how much there was.
	 */
	private val configurations = listOf(
		"" to false,
		"+legtweaks" to true,
	)

	/**
	 * The headline measurement. Reports every motion and configuration; asserts
	 * on the motions whose head height is static, for the reason in the class
	 * doc.
	 */
	@Test
	fun interpolationRecoversMostOfWhatSkewCosts() {
		// The SKEW prefix keeps these rows out of the replay-metrics workflow's
		// baseline table, which greps for lines starting with a motion name and
		// has different columns.
		println(
			"SKEW delays (ms): " + delaysMs.entries.joinToString { "${it.key.name}=${it.value}" },
		)
		println(
			"SKEW %-28s %-28s %12s %12s %12s %10s".format(
				"motion",
				"metric",
				"reference",
				"skewed",
				"aligned",
				"recovered",
			),
		)

		val failures = mutableListOf<String>()
		var judged = 0

		for (motion in SyntheticMotion.names) {
			for ((suffix, legTweaks) in configurations) {
				val ref = replay(motion, commonModeDelaysMs, false, legTweaks).toMap()
				val skew = replay(motion, delaysMs, false, legTweaks).toMap()
				val algn = replay(motion, delaysMs, true, legTweaks).toMap()

				for (metric in ref.keys) {
					if (metric == "height_m") continue
					val cost = abs(skew.getValue(metric) - ref.getValue(metric))
					val residual = abs(algn.getValue(metric) - ref.getValue(metric))
					val recovered = if (cost > 0f) 1f - residual / cost else Float.NaN

					println(
						"SKEW %-28s %-28s %12.6f %12.6f %12.6f %9.1f%%".format(
							"$motion$suffix",
							metric,
							ref.getValue(metric),
							skew.getValue(metric),
							algn.getValue(metric),
							recovered * 100f,
						),
					)

					if (motion !in SyntheticMotion.staticHeadHeight) continue
					// Only judge metrics where the skew did enough damage for
					// the ratio to mean anything. Below this the difference
					// being divided by is itself near zero, and a percentage of
					// it is noise dressed up as a result.
					if (cost < MEANINGFUL_COST_M) continue
					judged++
					if (residual > cost * MAX_RESIDUAL_FRACTION) {
						failures.add(
							"$motion$suffix/$metric: skew cost $cost, alignment left " +
								"$residual (more than ${MAX_RESIDUAL_FRACTION * 100}% of it)",
						)
					}
				}
			}
		}

		// A silent zero here would make this test pass by measuring nothing.
		assertTrue(
			judged > 0,
			"no metric moved by more than $MEANINGFUL_COST_M under ${maxDelayMs}ms of " +
				"skew, so nothing above was judged. Either the skew is too small to " +
				"matter or the metrics cannot see it; both are results, and neither " +
				"is a passing test",
		)
		assertTrue(failures.isEmpty(), failures.joinToString("\n"))
	}

	/**
	 * The measurement above is only meaningful if the skew is doing visible
	 * damage in the first place. Foot slide on `walk-in-place` with the plain
	 * solver is the most skew-sensitive combination available: the legs are the
	 * far end of the chain and they are moving fastest.
	 *
	 * If this fails, the delays are too small or the metrics cannot see skew,
	 * and the recovery percentages above are dividing by noise.
	 */
	@Test
	fun skewDegradesTheWalkPose() {
		val reference = replay("walk-in-place", commonModeDelaysMs, false, legTweaks = false)
		val skewed = replay("walk-in-place", delaysMs, false, legTweaks = false)

		val cost = abs(skewed.footSlideMPerSec - reference.footSlideMPerSec)
		println(
			"SKEW walk-in-place foot slide: reference=${reference.footSlideMPerSec} m/s, " +
				"skewed=${skewed.footSlideMPerSec} m/s, cost=$cost m/s",
		)

		assertTrue(
			cost > MEANINGFUL_COST_M,
			"${maxDelayMs}ms of tracker skew changed foot slide by only $cost m/s. " +
				"Time alignment is correcting something too small to measure here, " +
				"which is a result about the correction's value, not a passing test",
		)
	}

	/**
	 * A tracker that stops reporting must not drag the reference into the past
	 * with it. `TimeAlignment.DEFAULT_MAX_SKEW_MICROS` bounds how far the
	 * laggiest participant can pull the instant everything else is solved for;
	 * without that bound one dead tracker holds the whole pose still.
	 */
	@Test
	fun aStalledTrackerDoesNotFreezeThePose() {
		val setup = buildSkeleton()
		val hpm = setup.hpm
		val height = hpm.userHeightFromConfig
		val clock = FixedStepClock(1f / rateHz)
		hpm.skeleton.legTweaks.clock = clock.clock

		val stalled = setup.byPosition.getValue(TrackerPosition.RIGHT_LOWER_LEG)
		var lastAnkle = Vector3.NULL
		var moved = 0

		for (i in 0 until frames) {
			clock.advance()
			val t = i / rateHz
			val frame = SyntheticMotion.at("walk-in-place", t)
			setup.hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			setup.hmd.setRotation(Quaternion.IDENTITY)

			for ((position, tracker) in setup.byPosition) {
				// The stalled tracker stops sending after a quarter of the run.
				if (tracker === stalled && i > frames / 4) continue
				tracker.setTimestampedRotation(
					rotationFor(position, frame),
					EPOCH_MICROS + (t * 1e6f).toLong(),
				)
			}

			hpm.update()

			val ankle = hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position
			if (i > frames / 2 && (ankle - lastAnkle).len() > 1e-4f) moved++
			lastAnkle = ankle
		}

		println(
			"SKEW with one stalled tracker: spread=${hpm.skeleton.timeAlignment.spreadMicros}us, " +
				"stragglerPasses=${hpm.skeleton.timeAlignment.stragglerPasses}, " +
				"moving frames after the stall=$moved",
		)

		assertTrue(
			moved > 0,
			"the pose stopped moving after one tracker stalled -- the reference " +
				"followed the dead tracker into the past instead of being bounded",
		)
	}

	private class Setup(
		val hpm: HumanPoseManager,
		val hmd: Tracker,
		val byPosition: Map<TrackerPosition, Tracker>,
	)

	private fun buildSkeleton(): Setup {
		val hmd = mkTracker(0, TrackerPosition.HEAD, isHmd = true)
		val byPosition = linkedMapOf<TrackerPosition, Tracker>()
		var id = 1
		for (position in delaysMs.keys) {
			byPosition[position] = mkTracker(id++, position)
		}

		val hpm = HumanPoseManager(listOf(hmd) + byPosition.values)
		hpm.skeleton.hasKneeTrackers = true
		return Setup(hpm, hmd, byPosition)
	}

	/**
	 * @param timestamped when false the trackers report rotations with no
	 * instant attached, which is exactly the pre-alignment code path: with no
	 * sample history `TimeAlignment` finds no participants and touches nothing.
	 * This flag is therefore a straight toggle between old and new behaviour,
	 * not a separate test-only pipeline.
	 */
	private fun replay(
		motion: String,
		delaysMs: Map<TrackerPosition, Float>,
		timestamped: Boolean,
		legTweaks: Boolean,
	): PoseMetrics {
		val setup = buildSkeleton()
		val hpm = setup.hpm
		val height = hpm.userHeightFromConfig

		// As in SkeletonReplayTest, the master switch alone changes nothing --
		// the individual corrections are separate toggles that default to off.
		hpm.setLegTweaksEnabled(legTweaks)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, legTweaks)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, legTweaks)

		val clock = FixedStepClock(1f / rateHz)
		hpm.skeleton.legTweaks.clock = clock.clock

		val accumulator = PoseMetricsAccumulator()
		val dt = 1f / rateHz

		for (i in 0 until frames) {
			clock.advance()
			val t = i / rateHz

			// The headset is un-delayed and un-timestamped; see the class doc.
			val headFrame = SyntheticMotion.at(motion, t)
			setup.hmd.position = Vector3(0f, height * headFrame.headHeightFraction, 0f)
			setup.hmd.setRotation(Quaternion.IDENTITY)

			for ((position, tracker) in setup.byPosition) {
				// Each tracker samples the same motion at its own instant. Note
				// this is the true motion at that instant, not a resampled
				// frame -- the skew being simulated is in when the measurement
				// was taken, not in what was measured.
				val sampleT = t - delaysMs.getValue(position) / 1000f
				val rotation = rotationFor(position, SyntheticMotion.at(motion, sampleT))
				if (timestamped) {
					tracker.setTimestampedRotation(
						rotation,
						EPOCH_MICROS + (sampleT * 1e6f).toLong(),
					)
				} else {
					tracker.setRotation(rotation)
				}
			}

			hpm.update()

			// Skip the warm-up. For the first few frames a tracker's history
			// does not yet bracket the reference, so alignment clamps instead
			// of interpolating, and the leg corrections are still filling their
			// own buffers. That transient is real but it is a startup cost, not
			// a property of the correction, and on the low-signal motions it is
			// larger than the skew being measured.
			if (i < warmupFrames) continue

			accumulator.observeAnkles(
				hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position,
				hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position,
				dt,
			)
		}

		return accumulator.result(height)
	}

	private fun rotationFor(position: TrackerPosition, frame: SyntheticMotion.Frame): Quaternion = when (position) {
		TrackerPosition.CHEST -> frame.chest
		TrackerPosition.HIP -> frame.hip
		TrackerPosition.LEFT_UPPER_LEG -> frame.leftThigh
		TrackerPosition.LEFT_LOWER_LEG -> frame.leftCalf
		TrackerPosition.RIGHT_UPPER_LEG -> frame.rightThigh
		TrackerPosition.RIGHT_LOWER_LEG -> frame.rightCalf
		else -> throw IllegalArgumentException("no synthetic motion for $position")
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

	companion object {
		/**
		 * Sample times are offset from zero so that a delayed tracker's first
		 * few samples are still positive -- `TimeAlignment` treats a zero
		 * timestamp as "this tracker does not report one".
		 */
		private const val EPOCH_MICROS = 1_000_000_000L

		/**
		 * Below this the skew has not moved the metric far enough for a
		 * recovery percentage to be anything but a ratio of two small numbers.
		 */
		private const val MEANINGFUL_COST_M = 1e-3f

		/** Alignment must remove at least this much of the damage it can see. */
		private const val MAX_RESIDUAL_FRACTION = 0.25f
	}
}
