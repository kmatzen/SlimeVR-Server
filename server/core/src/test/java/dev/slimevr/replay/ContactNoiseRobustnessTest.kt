package dev.slimevr.replay

import dev.slimevr.tracking.processor.skeleton.OfflineContactLabeller
import org.junit.jupiter.api.Test
import kotlin.test.assertTrue

/**
 * Does the stillness rule survive sensor noise, or do the thresholds earn their
 * keep?
 *
 * Issue #5 measured three contact detectors on clean synthetic motion and found
 * a causal trailing-stillness rule beat the shipped hysteretic thresholds --
 * F1 0.799 → 0.935, mean transition error 125 ms → 36 ms -- and beat the offline
 * labeller too. It recorded a caveat next to the result naming exactly how the
 * finding might be wrong:
 *
 * > a bare stillness radius has no defence against noise -- noise inflates
 * > apparent displacement, which is exactly what this rule thresholds on. The
 * > ordering could reverse on real recordings, and what looks like ~50 redundant
 * > constants may be ~50 constants earning their keep.
 *
 * That is a named mechanism, and a named mechanism can be probed without waiting
 * for hardware. [SensorNoise] injects the three components of IMU error that
 * model faithfully; this sweeps them.
 *
 * ## The answer, in one line
 *
 * **The mechanism is real and the crossover is far outside the range the
 * ordering was in doubt over.** Stillness is *flat* -- F1 unchanged in the third
 * decimal -- through a degree of per-frame white orientation error, then falls
 * off a cliff. The thresholds overtake it at σ ≈ 3° white, or σ ≈ 8° when the
 * noise is correlated rather than white.
 *
 * ## What this can and cannot settle
 *
 * It settles **shape**: the stillness rule does not degrade gently, it holds and
 * then collapses, and the collapse is nowhere near where the caveat feared. It
 * also settles the sub-question the caveat implies -- whether the ~50 constants
 * are buying noise immunity. Up to the crossover they buy none: the thresholds
 * score 0.799 with or without noise, so robustness is not what distinguishes
 * them.
 *
 * It cannot settle **magnitude**: nothing here says what σ a real tracker
 * delivers, so it cannot say which side of the crossover reality sits on. That
 * still needs #15 -- but it narrows the ask from *"re-run the comparison on a
 * recording"* to *"measure one number and read it off this table"*.
 *
 * Two limits bound the claim, both from [SensorNoise]: `.pfr` stores fused
 * output, so real error is smoother than the white column and the correlated
 * column is the better guide; and noise is injected at the tracker rather than
 * the sensor, so no fusion filter gets to recover from it.
 */
class ContactNoiseRobustnessTest {

	private val motion = "walk-in-place"

	/** Orientation error in degrees, spanning far below to far above plausible. */
	private val sweepDeg = listOf(0f, 0.1f, 0.25f, 0.5f, 1f, 2f, 3f, 5f, 8f, 12f, 20f)

	/**
	 * Frames of exponential smoothing for the correlated sweep.
	 *
	 * 10 frames at 100 Hz, so the error wanders over a tenth of a second rather
	 * than being resampled every frame. Closer to what survives a fusion filter.
	 */
	private val smoothingFrames = 10

	/** Below this, the ordering measured on issue #5 is expected to hold. */
	private val orderingHoldsBelowDeg = 1f

	private class Row(
		val label: String,
		val noiseDeg: Float,
		val heuristicF1: Float,
		val heuristicLag: Float,
		val trailingF1: Float,
		val trailingLag: Float,
		val offlineF1: Float,
		val offlineLag: Float,
		/**
		 * Spurious transitions the thresholds invented, summed over both feet.
		 *
		 * Reported because it is what makes the lag column interpretable. A
		 * detector that emits many extra transitions gives the nearest-match
		 * search more chances to land close to a true one, so its mean lag can
		 * fall while its decisions get worse. Without this column the thresholds
		 * appear to get *better* at timing as noise rises, which they do not.
		 */
		val heuristicSpurious: Int,
	) {
		val trailingWins: Boolean get() = trailingF1 > heuristicF1

		fun report(): String = "%-24s  %.3f / %5.2f / %4d   %.3f / %5.2f   %.3f / %5.2f  %s".format(
			label,
			heuristicF1, heuristicLag, heuristicSpurious,
			trailingF1, trailingLag,
			offlineF1, offlineLag,
			if (trailingWins) "stillness" else "THRESHOLDS",
		)
	}

	private fun measure(label: String, noiseDeg: Float, noise: SensorNoise?): Row {
		val run = ContactReplay.run(motion, noise)
		val labeller = OfflineContactLabeller()

		fun mean(scores: List<ContactMetrics.Result>, of: (ContactMetrics.Result) -> Float) = scores.map(of).average().toFloat()

		val heuristic = listOf(
			ContactMetrics.score(run.leftHeuristic, run.leftTruth),
			ContactMetrics.score(run.rightHeuristic, run.rightTruth),
		)
		val trailing = listOf(
			ContactMetrics.score(ContactReplay.trailingStillness(run.leftPositions, labeller), run.leftTruth),
			ContactMetrics.score(ContactReplay.trailingStillness(run.rightPositions, labeller), run.rightTruth),
		)
		val offline = listOf(
			ContactMetrics.score(labeller.label(run.leftPositions), run.leftTruth),
			ContactMetrics.score(labeller.label(run.rightPositions), run.rightTruth),
		)

		return Row(
			label = label,
			noiseDeg = noiseDeg,
			heuristicF1 = mean(heuristic) { it.f1 },
			heuristicLag = mean(heuristic) { it.meanAbsLagFrames },
			trailingF1 = mean(trailing) { it.f1 },
			trailingLag = mean(trailing) { it.meanAbsLagFrames },
			offlineF1 = mean(offline) { it.f1 },
			offlineLag = mean(offline) { it.meanAbsLagFrames },
			heuristicSpurious = heuristic.sumOf { it.spuriousTransitions },
		)
	}

	private fun header(title: String) {
		println()
		println(title)
		println("%-24s  %-21s %-14s %-14s %s".format("noise", "thresholds", "trailing", "offline", "better"))
		println("%-24s  %-21s %-14s %-14s %s".format("", "F1 / |lag| / spur", "F1 / |lag|fr", "F1 / |lag|fr", ""))
	}

	private fun sweep(title: String, smoothing: Int): List<Row> {
		header(title)
		return sweepDeg.map { deg ->
			val row = measure(
				if (deg == 0f) "noiseless" else "σ=%.2f°".format(deg),
				deg,
				if (deg == 0f) null else SensorNoise(whiteNoiseDeg = deg, smoothingFrames = smoothing),
			)
			println(row.report())
			row
		}
	}

	/** First σ at which the thresholds overtake stillness, or null if never. */
	private fun crossover(rows: List<Row>): Float? = rows.firstOrNull { !it.trailingWins }?.noiseDeg

	/**
	 * The headline sweep: per-frame white orientation error.
	 *
	 * This is the worst case for the stillness rule, since white error is
	 * uncorrelated frame to frame and therefore turns most directly into
	 * apparent displacement.
	 */
	@Test
	fun stillnessSurvivesFarMoreNoiseThanTheCaveatFeared() {
		val rows = sweep("white orientation noise", smoothing = 0)
		val crossoverDeg = crossover(rows)
		println()
		println(
			crossoverDeg?.let { "crossover: thresholds overtake stillness at σ=$it°" }
				?: "stillness still ahead at σ=${sweepDeg.last()}°",
		)

		// The finding recorded on issue #5 must hold everywhere the ordering was
		// not in doubt. If it stops holding at a tenth of a degree, the finding
		// was an artefact of noiseless input and the issue needs revisiting.
		val quiet = rows.filter { it.noiseDeg <= orderingHoldsBelowDeg }
		for (row in quiet) {
			assertTrue(
				row.trailingWins,
				"at ${row.label} the hysteretic thresholds already beat trailing " +
					"stillness (F1 ${row.heuristicF1} vs ${row.trailingF1}). Issue #5's " +
					"measured finding would hold only on noiseless input, and its " +
					"suggestion to try the rule change first would not survive.",
			)
		}

		// And the rule must be genuinely flat there, not merely ahead. A rule
		// already degrading at a tenth of a degree would be one whose crossover
		// happens to sit just above the sweep's first steps.
		val clean = rows.first().trailingF1
		for (row in quiet.drop(1)) {
			assertTrue(
				row.trailingF1 >= clean - 0.02f,
				"trailing stillness fell from $clean to ${row.trailingF1} by ${row.label}, " +
					"so it is degrading within the range the ordering was never in " +
					"doubt over rather than holding flat to a distant cliff.",
			)
		}

		// The caveat's mechanism is real: enough noise does reverse the ordering.
		// Pinned so that "noise never matters" cannot be read off this file.
		assertTrue(
			crossoverDeg != null,
			"the thresholds never overtook stillness even at σ=${sweepDeg.last()}°, so " +
				"either the noise is not reaching the detector or the sweep no longer " +
				"spans the crossover. Both make this test vacuous.",
		)
		assertTrue(
			crossoverDeg!! > orderingHoldsBelowDeg,
			"the crossover moved to σ=$crossoverDeg°, at or below the range where issue " +
				"#5's finding was taken to be safe. The case for trying the stillness " +
				"rule rests on that crossover being distant.",
		)
	}

	/**
	 * Correlated error, which is what a fusion filter actually leaves behind.
	 *
	 * Reported separately rather than folded into the sweep above because it is
	 * the more realistic column and the less conservative one: it should be read
	 * as the likely case and the white sweep as the bound.
	 */
	@Test
	fun correlatedNoiseIsMoreForgivingThanWhite() {
		val white = sweep("white orientation noise", smoothing = 0)
		val smooth = sweep("correlated orientation noise ($smoothingFrames-frame)", smoothing = smoothingFrames)

		val whiteCrossover = crossover(white)
		val smoothCrossover = crossover(smooth)
		println()
		println("crossover: white σ=$whiteCrossover°, correlated σ=$smoothCrossover°")

		// Same standard deviation, spread over time instead of resampled every
		// frame. Neighbouring frames then move together, so a rule that
		// thresholds on displacement *between* frames sees less of it.
		//
		// This matters for reading the whole file: `.pfr` stores fused output,
		// so the realistic column is this one, and it is the forgiving one.
		assertTrue(
			smoothCrossover != null && whiteCrossover != null && smoothCrossover > whiteCrossover,
			"correlated noise ($smoothCrossover°) no longer tolerates more than white " +
				"($whiteCrossover°). The argument that real post-fusion error is the " +
				"easier case depends on this ordering.",
		)
	}

	/**
	 * A control: bias is not variance, and only variance hurts.
	 *
	 * Mounting error puts the whole body somewhere slightly wrong and holds it
	 * there. If the detectors degraded under it too, the sweep above would be
	 * measuring "the pose is wrong" rather than "the pose is jittery", and the
	 * conclusion about a displacement threshold would not follow.
	 */
	@Test
	fun steadyBiasDoesNotDegradeEitherDetector() {
		header("constant mounting error (control)")
		val clean = measure("none", 0f, null)
		println(clean.report())

		for (deg in listOf(2f, 5f, 10f)) {
			val row = measure("mount=%.0f°".format(deg), deg, SensorNoise(mountingErrorDeg = deg))
			println(row.report())

			assertTrue(
				row.trailingF1 >= clean.trailingF1 - 0.02f,
				"a constant $deg° mounting error moved trailing stillness from " +
					"${clean.trailingF1} to ${row.trailingF1}. It should be nearly " +
					"invariant to a steady bias, since it thresholds on displacement " +
					"between frames rather than on absolute position -- if it is not, " +
					"the noise sweep is not isolating what it claims to.",
			)
		}
	}

	/**
	 * The thresholds' timing appears to *improve* under noise. It does not.
	 *
	 * Worth pinning explicitly because the number is genuinely there in the
	 * table and reads as a result: mean transition error falls from 12.5 frames
	 * to under 6 as σ passes 2°. What is actually happening is that the detector
	 * starts emitting tens of transitions that correspond to nothing, and the
	 * nearest-match search finds one of them close to each true event.
	 *
	 * A future reader comparing detectors on lag alone would conclude the
	 * thresholds get better in noise, and act on it.
	 */
	@Test
	fun apparentTimingGainUnderNoiseIsSpuriousTransitions() {
		val rows = sweep("white orientation noise", smoothing = 0)

		val clean = rows.first()
		val improved = rows.filter { it.heuristicLag < clean.heuristicLag - 1f }
		println()
		println("rows where the thresholds' lag 'improved': ${improved.map { it.label }}")

		assertTrue(
			improved.isNotEmpty(),
			"the thresholds' apparent timing no longer improves under noise, so this " +
				"test no longer guards anything -- check whether the sweep still " +
				"reaches the noise levels where it did.",
		)

		// Every row where the lag improves must have got worse in a way the lag
		// column cannot show. There are two such ways and the sweep reaches both:
		//
		//  - Spurious transitions explode (σ 2-12°). Extra transitions give the
		//    matcher more chances to land near a true event.
		//  - The detector collapses entirely (σ 20°, F1 0.016). It barely calls
		//    contact at all, so almost nothing is matched and the mean is taken
		//    over a handful of events.
		//
		// Asserting the disjunction rather than the first alone, because the
		// claim being pinned is neither mechanism specifically -- it is that a
		// falling lag never means better decisions.
		for (row in improved) {
			val spuriousExploded = row.heuristicSpurious > clean.heuristicSpurious * 5
			val detectorCollapsed = row.heuristicF1 < clean.heuristicF1 * 0.5f
			assertTrue(
				spuriousExploded || detectorCollapsed,
				"at ${row.label} the thresholds' mean lag improved to ${row.heuristicLag} " +
					"frames while their decisions did not get worse -- spurious " +
					"transitions ${row.heuristicSpurious} vs ${clean.heuristicSpurious} " +
					"clean, F1 ${row.heuristicF1} vs ${clean.heuristicF1}. If that is " +
					"real rather than an artefact of the matcher, the timing comparison " +
					"on issue #5 needs restating.",
			)
		}
	}

	/**
	 * Injected noise must be reproducible, or every number above is unstable and
	 * this file undermines the baseline machinery (#14, #16) rather than adding
	 * to it.
	 */
	@Test
	fun noiseIsReproducible() {
		val a = ContactReplay.run(motion, SensorNoise(whiteNoiseDeg = 0.5f, smoothingFrames = 4))
		val b = ContactReplay.run(motion, SensorNoise(whiteNoiseDeg = 0.5f, smoothingFrames = 4))

		val differing = a.leftPositions.indices.count { (a.leftPositions[it] - b.leftPositions[it]).len() != 0f }
		println("frames differing between two replays of one noise configuration: $differing")

		assertTrue(differing == 0, "injected noise is not reproducible: $differing frames differ")
	}
}
