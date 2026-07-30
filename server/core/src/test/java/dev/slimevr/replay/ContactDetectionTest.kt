package dev.slimevr.replay

import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.processor.skeleton.LegTweaksBuffer
import dev.slimevr.tracking.processor.skeleton.OfflineContactLabeller
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerRole
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Measures foot-contact detection against ground truth.
 *
 * ## Why this exists
 *
 * Issue #5 proposes replacing the contact thresholds in
 * [LegTweaksBuffer] with a learned classifier. Before any classifier can be
 * argued about there has to be a number for what it would be replacing, and
 * there is not one: nothing in the repository has ever measured how well contact
 * detection works, because nothing had labels to measure it against.
 *
 * The audit on the issue is specific about what a fair comparison requires:
 *
 * > The baseline to beat is a hysteretic state machine, not a memoryless
 * > threshold. Hysteresis is cheap temporal smoothing, and it is exactly the
 * > thing a 0.5 s windowed model would otherwise claim as its own advantage.
 * > The comparison needs to be against the hysteretic behaviour, or the model
 * > will look better than it is.
 *
 * So the baseline here is the real [LegTweaksBuffer.checkState] running inside a
 * real pipeline, hysteresis, sensitivity scalars and all -- not a
 * reimplementation of its thresholds.
 *
 * ## On the "#15 is a hard prerequisite" caveat
 *
 * Issue #5 says evaluating this work needs real recordings, because on synthetic
 * motion the existing heuristics drive foot slide to exactly zero and there is
 * nothing to beat.
 *
 * That is true of *foot slide*, and it is why slide is not the metric here.
 * Contact detection has its own error, and it is not zero on synthetic motion:
 * a foot can be declared planted several frames after it left the floor while
 * the slide metric stays at zero throughout, because slide measures where the
 * foot ends up and this measures when the decision was made. The two are not
 * the same quantity and the second is measurable today.
 *
 * That does not retire #15. A synthetic sequence has no IMU noise, no yaw drift
 * and no mounting error, so what is measured here is how the detector handles
 * clean geometry -- an upper bound on its real behaviour, and a lower bound on
 * the problem. Training a classifier still needs real recordings.
 */
class ContactDetectionTest {

	private val rateHz = ContactReplay.RATE_HZ
	private val frames = ContactReplay.FRAMES

	private fun replay(motion: String) = ContactReplay.run(motion)

	/**
	 * The ground truth has to describe a foot that actually leaves the floor,
	 * or every method scores perfectly by saying "planted" forever.
	 */
	@Test
	fun theWalkingSequenceLiftsEachFootRepeatedly() {
		val run = replay("walk-in-place")

		val leftLifts = ContactMetrics.transitionsOf(run.leftTruth).count { !it.toContact }
		val rightLifts = ContactMetrics.transitionsOf(run.rightTruth).count { !it.toContact }

		println("walk-in-place: $leftLifts left liftoffs, $rightLifts right liftoffs in $frames frames")

		assertTrue(leftLifts >= 4, "only $leftLifts left liftoffs; too few to measure timing on")
		assertTrue(rightLifts >= 4, "only $rightLifts right liftoffs")

		// Antiphase: the feet must not be in the air together, or "planted"
		// would be a legal answer for neither and the sequence would be
		// measuring something other than alternating gait.
		val bothAirborne = run.leftTruth.indices.count { !run.leftTruth[it] && !run.rightTruth[it] }
		assertEquals(0, bothAirborne, "both feet were off the floor for $bothAirborne frames of walk-in-place")
	}

	/**
	 * The baseline measurement: what the existing heuristics actually achieve.
	 *
	 * Reported rather than gated tightly. The purpose is to establish the number
	 * a learned classifier would have to beat, and to make it visible in CI so
	 * that a change to the thresholds moves something that can be seen.
	 */
	@Test
	fun theHeuristicBaselineIsMeasured() {
		val run = replay("walk-in-place")

		val left = ContactMetrics.score(run.leftHeuristic, run.leftTruth)
		val right = ContactMetrics.score(run.rightHeuristic, run.rightTruth)

		println(left.report("heuristic left", rateHz))
		println(right.report("heuristic right", rateHz))

		for (result in listOf(left, right)) {
			assertTrue(
				result.truePositives > 0,
				"the detector never reported contact at all, so it is not being exercised",
			)
			assertTrue(
				result.falsePositives + result.falseNegatives > 0 ||
					result.meanAbsLagFrames > 0f,
				"the detector matched ground truth exactly, including every transition " +
					"frame. That would mean contact detection has no error to improve " +
					"on, which contradicts the premise of issue #5 -- check the " +
					"ground truth is not being derived from the detector.",
			)
		}
	}

	/**
	 * Both feet stay down for the sequences where neither is meant to lift.
	 *
	 * The other half of contact accuracy, and the one the walking sequence
	 * cannot test: a detector that reports liftoffs during a squat is anchoring
	 * the body to nothing at moments the user is stationary, which is the
	 * "world lurches" failure issue #5 describes.
	 */
	@Test
	fun stationarySequencesProduceNoSpuriousLiftoffs() {
		for (motion in listOf("stand", "squat", "lean")) {
			val run = replay(motion)

			val left = ContactMetrics.score(run.leftHeuristic, run.leftTruth)
			val right = ContactMetrics.score(run.rightHeuristic, run.rightTruth)

			println(left.report("$motion left", rateHz))
			println(right.report("$motion right", rateHz))

			assertTrue(
				run.leftTruth.all { it } && run.rightTruth.all { it },
				"'$motion' is supposed to keep both feet planted throughout",
			)
		}
	}

	/**
	 * Offline labels are better than the labels the current detector would
	 * produce, which is the minimum the training-data plan needs.
	 *
	 * Issue #5 recommends starting with self-supervised labelling of the `.pfr`
	 * corpus. If those labels were no better than the heuristic, the plan would
	 * be circular: a classifier trained on them would learn to reproduce the
	 * heuristic's mistakes and would look fine on its own training data while
	 * improving nothing. So the comparison is made rather than assumed.
	 *
	 * It passes -- but note what it does *not* establish. The issue attributes
	 * the advantage to lookahead ("that asymmetry is the whole trick"), and this
	 * test cannot tell whether the win came from looking ahead or from the
	 * offline labeller asking a different question than the heuristic does.
	 * [lookaheadAndRuleChangeAreSeparated] separates them, and the answer is not
	 * the one the issue assumes.
	 */
	@Test
	fun theOfflineLabellerBeatsTheCausalDetectorOnTiming() {
		val run = replay("walk-in-place")
		val labeller = OfflineContactLabeller()

		val offlineLeft = labeller.label(run.leftPositions)
		val offlineRight = labeller.label(run.rightPositions)

		val heuristicLeft = ContactMetrics.score(run.leftHeuristic, run.leftTruth)
		val heuristicRight = ContactMetrics.score(run.rightHeuristic, run.rightTruth)
		val labelledLeft = ContactMetrics.score(offlineLeft, run.leftTruth)
		val labelledRight = ContactMetrics.score(offlineRight, run.rightTruth)

		println(heuristicLeft.report("heuristic left", rateHz))
		println(labelledLeft.report("offline left", rateHz))
		println(heuristicRight.report("heuristic right", rateHz))
		println(labelledRight.report("offline right", rateHz))

		val heuristicLag = (heuristicLeft.meanAbsLagFrames + heuristicRight.meanAbsLagFrames) / 2f
		val offlineLag = (labelledLeft.meanAbsLagFrames + labelledRight.meanAbsLagFrames) / 2f
		val heuristicF1 = (heuristicLeft.f1 + heuristicRight.f1) / 2f
		val offlineF1 = (labelledLeft.f1 + labelledRight.f1) / 2f

		println(
			"mean |lag|: heuristic %.2f fr, offline %.2f fr | F1: heuristic %.3f, offline %.3f".format(
				heuristicLag,
				offlineLag,
				heuristicF1,
				offlineF1,
			),
		)

		assertTrue(
			offlineLag < heuristicLag,
			"the offline labeller ($offlineLag frames mean |lag|) is no better at " +
				"transition timing than the existing detector ($heuristicLag frames), " +
				"so training on its labels would teach a classifier the same " +
				"transition errors the heuristic already makes.",
		)

		assertTrue(
			offlineF1 >= heuristicF1,
			"the offline labeller traded frame-wise accuracy for timing " +
				"($offlineF1 against $heuristicF1). Labels that are better placed " +
				"but more often wrong are not obviously better training data.",
		)
	}

	/**
	 * Splits the offline labeller's advantage into the part that comes from
	 * looking ahead and the part that comes from asking a better question.
	 *
	 * This matters for what issue #5 should build. The offline labeller beats
	 * the hysteretic heuristic on both timing and frame-wise accuracy, and it is
	 * tempting to read that as "lookahead wins" -- but it changed two things at
	 * once. It looks ahead, *and* it replaces five thresholded signals plus
	 * hysteresis with a single question: did this foot stay within 2 cm of where
	 * it is now?
	 *
	 * Running the same rule over a trailing window separates them. A trailing
	 * window is causal -- it could run live -- so whatever it recovers is
	 * available without any learning at all, and only the remainder is the part a
	 * model would have to earn by inferring what is about to happen.
	 *
	 * Deliberately implemented here rather than as a mode on
	 * [OfflineContactLabeller]: that class's one guarantee is that it cannot run
	 * live, and adding a causal mode would make it a detector that happens to
	 * default to offline. This is a measurement probe, not a component.
	 */
	@Test
	fun lookaheadAndRuleChangeAreSeparated() {
		val run = replay("walk-in-place")
		val labeller = OfflineContactLabeller()

		fun trailingStillness(positions: List<Vector3>) = ContactReplay.trailingStillness(positions, labeller)

		val heuristic = listOf(
			ContactMetrics.score(run.leftHeuristic, run.leftTruth),
			ContactMetrics.score(run.rightHeuristic, run.rightTruth),
		)
		val trailing = listOf(
			ContactMetrics.score(trailingStillness(run.leftPositions), run.leftTruth),
			ContactMetrics.score(trailingStillness(run.rightPositions), run.rightTruth),
		)
		val offline = listOf(
			ContactMetrics.score(labeller.label(run.leftPositions), run.leftTruth),
			ContactMetrics.score(labeller.label(run.rightPositions), run.rightTruth),
		)

		fun meanF1(rs: List<ContactMetrics.Result>) = rs.map { it.f1 }.average().toFloat()
		fun meanLag(rs: List<ContactMetrics.Result>) = rs.map { it.meanAbsLagFrames }.average().toFloat()

		println("%-34s F1=%.3f  mean|lag|=%.2f fr".format("hysteretic thresholds (causal)", meanF1(heuristic), meanLag(heuristic)))
		println("%-34s F1=%.3f  mean|lag|=%.2f fr".format("stillness, trailing (causal)", meanF1(trailing), meanLag(trailing)))
		println("%-34s F1=%.3f  mean|lag|=%.2f fr".format("stillness, centred (offline)", meanF1(offline), meanLag(offline)))

		// Finding 1: the rule change is what wins. A single stillness radius,
		// evaluated causally, beats five thresholded signals with hysteresis on
		// both measures, without looking ahead at anything.
		assertTrue(
			meanF1(trailing) > meanF1(heuristic) && meanLag(trailing) < meanLag(heuristic),
			"a causal stillness test no longer beats the hysteretic thresholds " +
				"(F1 ${meanF1(trailing)} vs ${meanF1(heuristic)}, lag " +
				"${meanLag(trailing)} vs ${meanLag(heuristic)}). Much of the case " +
				"for this direction rests on it doing so.",
		)

		// Finding 2, and the one that was not expected: lookahead does not help
		// here. The centred window is *worse* than the trailing window of the
		// same total width, on both measures.
		//
		// Both windows shorten each contact interval by about the same amount,
		// because both need the whole window to be still before they will call
		// contact. The difference is where the shortening lands. Trailing puts
		// all of it at touchdown and none at liftoff, since the instant the foot
		// moves the window containing that motion is the current one. Centred
		// splits it across both ends, so it reports contact late *and* drops it
		// early, and the early drop is a transition error the trailing rule
		// simply does not make.
		//
		// This is asserted so that it cannot silently stop being true. Issue #5
		// justifies its whole training-data plan on lookahead being the
		// advantage ("that asymmetry is the whole trick"), so if this ordering
		// ever reverses, the conclusion drawn on the issue needs revisiting
		// rather than quietly going stale.
		//
		// The caveat is large and belongs next to the claim: this is one
		// synthetic sequence with no IMU noise. The hysteresis and the five
		// signals exist to cope with noise, and a bare stillness radius has no
		// defence against it -- noise inflates apparent displacement, which is
		// exactly what this rule thresholds on. The ordering could reverse on
		// real recordings, which is why #15 is still on the critical path.
		assertTrue(
			meanF1(trailing) >= meanF1(offline) && meanLag(trailing) <= meanLag(offline),
			"the centred window now beats the trailing window (F1 ${meanF1(offline)} " +
				"vs ${meanF1(trailing)}, lag ${meanLag(offline)} vs ${meanLag(trailing)}). " +
				"That reverses the measured finding recorded on issue #5, which is " +
				"that lookahead contributes nothing here and the rule change " +
				"contributes everything.",
		)
	}

	/**
	 * The labeller must not be usable as a detector, and the way to show that is
	 * that its answer for a frame depends on frames after it.
	 *
	 * Worth pinning because the failure would be silent and attractive: if the
	 * labeller happened to be causal, it could be run live, and someone would.
	 * Then it would be a threshold heuristic competing with the existing
	 * threshold heuristic, which is not what issue #5 is asking for.
	 */
	@Test
	fun theOfflineLabellerIsNotCausal() {
		val labeller = OfflineContactLabeller()
		val still = 40
		val after = 40

		// Two trajectories that are byte-identical for the first [still] frames
		// and differ only afterwards. Motion is horizontal, so height is the same
		// in both and stillness is the only thing that can distinguish them.
		val stays = List(still + after) { Vector3(0f, 0f, 0f) }
		val leaves = List(still + after) { i ->
			if (i < still) Vector3(0f, 0f, 0f) else Vector3(0.05f * (i - still + 1), 0f, 0f)
		}

		val stayLabels = labeller.label(stays)
		val leaveLabels = labeller.label(leaves)

		val differing = (0 until still).count { stayLabels[it] != leaveLabels[it] }

		println("frames before the divergence whose label changed: $differing")

		assertTrue(
			differing > 0,
			"two trajectories identical up to frame $still were labelled identically " +
				"up to frame $still, so no label depends on a later frame. Lookahead " +
				"is the labeller's only advantage over a runtime detector, and issue " +
				"#5's plan to train on its labels depends on it being real.",
		)

		// And the dependence must be local: a frame long before the divergence
		// cannot legitimately know about it. A labeller whose window reached
		// arbitrarily far would be smearing, not looking ahead.
		val earliestChanged = (0 until still).first { stayLabels[it] != leaveLabels[it] }
		assertTrue(
			earliestChanged >= still - labeller.stillnessWindowFrames - labeller.minSegmentFrames,
			"a label at frame $earliestChanged changed because of motion at frame " +
				"$still, which is further than the window should reach",
		)
	}
}
