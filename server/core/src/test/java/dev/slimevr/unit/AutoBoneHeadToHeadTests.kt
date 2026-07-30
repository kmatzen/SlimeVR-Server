package dev.slimevr.unit

import dev.slimevr.autobone.AutoBone
import dev.slimevr.autobone.AutoBoneStep
import dev.slimevr.autobone.PoseFrameStep
import dev.slimevr.autobone.leastsquares.AutoBoneErrorSet
import dev.slimevr.autobone.leastsquares.AutoBoneObjective
import dev.slimevr.config.AutoBoneConfig
import dev.slimevr.config.ConfigManager
import dev.slimevr.config.SkeletonConfig
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.poseframeformat.player.TrackerFramesPlayer
import dev.slimevr.poseframeformat.trackerdata.TrackerFrame
import dev.slimevr.poseframeformat.trackerdata.TrackerFrames
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerRole
import dev.slimevr.tracking.trackers.TrackerStatus
import io.eiren.util.collections.FastList
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import java.util.EnumMap
import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.test.assertTrue

/**
 * The head-to-head issue #7 step 4 is gated on: the Levenberg-Marquardt path
 * against the greedy coordinate search that ships, both run end to end through
 * [AutoBone.processFrames] exactly as a user gets them.
 *
 * ## Why this is not the comparison `AutoBoneLeastSquaresTests` already makes
 *
 * That file compares LM against *first-order descent on the LM objective*. That
 * is the claim issue #7 makes about iteration counts, and it is a fair test of
 * it, but it is not a test of the thing that ships. What ships is
 * [AutoBone.step]: a greedy coordinate search that proposes a length change per
 * bone from [dev.slimevr.autobone.BoneContribution] and keeps it only if the
 * error drops. It descends no global objective, so it cannot be run through
 * [dev.slimevr.autobone.leastsquares.AutoBoneObjective], and it lives inside an
 * [AutoBone] instance. Until that class could be built without a `VRServer`, no
 * test could reach it.
 *
 * ## What this can and cannot settle
 *
 * The greedy path's justification is empirical robustness on real recordings,
 * and the repository has none (#15). But "robustness" is robustness to
 * something nameable, and the three things it is robustness to are modelled
 * here: white sensor noise, a constant per-tracker mounting misalignment, and
 * per-tracker yaw drift. Sweeping them and watching where each solver breaks is
 * a weaker claim than a real capture, and a much stronger one than "we have not
 * looked".
 *
 * What it still cannot settle is model mismatch: a real body is not this
 * kinematic chain, and no noise model makes it one. That is left on #7 and #15.
 *
 * ## Why the answer is knowable at all
 *
 * The same construction `AutoBoneLeastSquaresTests` documents: generate the
 * motion from a skeleton with chosen bone lengths, then translate the headset
 * each frame so the ankles land on a fixed point. The chain hangs rigidly from
 * the head, so one translation plants both feet exactly, and the recording has
 * zero foot slide when replayed with the lengths that made it.
 *
 * Two constraints keep that exact here. Every rotation is a pitch about X, so
 * the constant ±X offset between the two hip joints never rotates and planting
 * the ankle midpoint plants each ankle. And both legs carry the same rotation,
 * so the ankles stay a fixed vector apart.
 *
 * ## What is perturbed, and why that particular perturbation
 *
 * The recording is generated from stock proportions except for the thigh/shin
 * split: 6 cm is moved from the shin into the thigh. That is deliberate on two
 * counts. It is height-neutral, so the target height both solvers normalise to
 * is exactly right and neither is scored on a scale error neither can see. And
 * it is the direction the knee bend makes identifiable, which
 * `AutoBoneLeastSquaresTests` already pins as recoverable — so a solver that
 * misses it is missing something the recording genuinely contains.
 *
 * The other seven adjusted bones start at the values that generated the
 * recording. They are scored too, and it is not a free pass: a solver chasing
 * noise moves them away from an answer that was already correct.
 */
class AutoBoneHeadToHeadTests {

	private val frameCount = 120

	/** 50 Hz, matching `AutoBoneConfig.sampleRateMs`. */
	private val frameSeconds = 0.02f

	/**
	 * Moved out of the shin and into the thigh, so the total is unchanged.
	 *
	 * Both solvers start from stock proportions, so this is also how far each
	 * has to travel: 6 cm, in opposite directions, on two bones.
	 */
	private val splitShift = 0.06f

	private val trueLengths = linkedMapOf(
		SkeletonConfigOffsets.UPPER_LEG to
			SkeletonConfigOffsets.UPPER_LEG.defaultValue + splitShift,
		SkeletonConfigOffsets.LOWER_LEG to
			SkeletonConfigOffsets.LOWER_LEG.defaultValue - splitShift,
	)

	/** The bones [AutoBone] adjusts, which is what gets scored. */
	private val scoredOffsets = listOf(
		SkeletonConfigOffsets.HEAD,
		SkeletonConfigOffsets.NECK,
		SkeletonConfigOffsets.UPPER_CHEST,
		SkeletonConfigOffsets.CHEST,
		SkeletonConfigOffsets.WAIST,
		SkeletonConfigOffsets.HIP,
		SkeletonConfigOffsets.HIPS_WIDTH,
		SkeletonConfigOffsets.UPPER_LEG,
		SkeletonConfigOffsets.LOWER_LEG,
	)

	/**
	 * What the recording was generated from: stock everywhere except the two
	 * bones above.
	 */
	private fun truth(offset: SkeletonConfigOffsets): Float = trueLengths[offset] ?: offset.defaultValue

	// #region The experiment

	/** Roughly what a well-mounted set reports: 0.6° jitter, 1° mounting, slow drift. */
	private val TYPICAL = SensorError(whiteNoiseRad = 0.010f, mountingErrorRad = 0.017f, yawDriftRadPerSec = 0.0009f)

	/** A bad session: 2° jitter, 5° mounting, drift fast enough to see over a minute. */
	private val POOR = SensorError(whiteNoiseRad = 0.035f, mountingErrorRad = 0.087f, yawDriftRadPerSec = 0.0044f)

	/**
	 * Everything below is scored against [trueLengths], so this has to hold
	 * first: the recording really is one that the true lengths explain exactly.
	 *
	 * If planting were only approximate -- which is what would happen if any
	 * rotation at or below the hip left the X axis, see [pose] -- the true
	 * lengths would not be the slide optimum, and every comparison after this
	 * would be scoring solvers against a target that is not the answer.
	 */
	@Test
	fun theTrueLengthsExplainTheRecordingExactly() {
		val frames = record(SensorError())

		val atTruth = measureSlide(frames, scoredOffsets.associateWith { truth(it) })
		val atStock = measureSlide(frames, scoredOffsets.associateWith { it.defaultValue })

		println("mean foot slide at the true lengths:  ${fmt(atTruth)} m")
		println("mean foot slide at stock proportions: ${fmt(atStock)} m")

		assertTrue(
			atTruth < 1e-5f,
			"the recording was generated from these lengths, so replaying it with them should " +
				"plant the feet, but the mean slide is ${atTruth}m -- the planting construction " +
				"is not exact and nothing below this is measuring what it claims to",
		)
		assertTrue(
			atStock > 100f * atTruth,
			"stock proportions leave ${atStock}m of slide against ${atTruth}m at the truth, " +
				"which is not enough of a gap for a solver to have anything to find",
		)
	}

	/**
	 * ## The blocker on issue #7 step 4
	 *
	 * `bodyProportionErrorFactor` was tuned for the greedy path, whose scalar is
	 * a weighted sum of error *magnitudes*. The least-squares objective is a sum
	 * of *squares*, and a weight does not carry across that change: squaring
	 * rewards whichever term is already larger and suppresses whichever is
	 * already smaller.
	 *
	 * Here the proportion term is roughly ten times the slide term before
	 * squaring, and it comes out about a thousand times larger after. The
	 * consequence is not a matter of degree. At the shipped weights the
	 * least-squares objective **ranks stock proportions above the lengths the
	 * recording was generated from**, so the answer is not merely hard for the
	 * solver to reach: it is not the optimum.
	 *
	 * This is the thing that has to be settled before the greedy path's
	 * constants can be retired, and it cannot be settled here. Choosing a new
	 * weight to make this fixture come out right would be picking a constant to
	 * satisfy one synthetic recording, which is the practice issue #7 objects to
	 * in the first place. Re-deriving the weights for the squared objective
	 * needs recordings -- #15.
	 */
	@Test
	fun theProportionTermDominatesTheSquaredObjectiveAndPrefersStockProportions() {
		val frames = record(SensorError())

		val withPrior = costAt(frames, bodyProportionFactor = 0.05f)
		val slideOnly = costAt(frames, bodyProportionFactor = 0f)

		println("least-squares cost, stock weights (slide + proportion):")
		println("  at stock proportions: ${sci(withPrior.startCost)}  (slide ${sci(withPrior.startSlide)}, proportion ${sci(withPrior.startOther)})")
		println("  at the true lengths:  ${sci(withPrior.truthCost)}  (slide ${sci(withPrior.truthSlide)}, proportion ${sci(withPrior.truthOther)})")
		println("least-squares cost, slide only:")
		println("  at stock proportions: ${sci(slideOnly.startCost)}")
		println("  at the true lengths:  ${sci(slideOnly.truthCost)}")

		assertTrue(
			withPrior.startOther > 100.0 * withPrior.startSlide,
			"the proportion term was expected to swamp the slide term after squaring, but it is " +
				"only ${sci(withPrior.startOther)} against ${sci(withPrior.startSlide)}",
		)
		assertTrue(
			withPrior.truthCost > withPrior.startCost,
			"at the shipped weights the squared objective was expected to prefer stock " +
				"proportions to the truth -- that is why the least-squares path cannot be the " +
				"default -- but the truth scored better (${sci(withPrior.truthCost)} vs " +
				"${sci(withPrior.startCost)}). If this has stopped being true the weights have " +
				"been re-derived and issue #7 step 4 can be reconsidered.",
		)
		assertTrue(
			slideOnly.truthCost < slideOnly.startCost,
			"with the proportion term off, the truth has to be the better point or the objective " +
				"is not measuring the recording at all",
		)
	}

	/**
	 * The other defect, and an independent one: with the objective well posed,
	 * the *solver* was still failing.
	 *
	 * `AutoBone.adjustOffsets` is never fully identifiable -- see
	 * [dev.slimevr.autobone.leastsquares.AutoBoneLevenbergMarquardt] -- and
	 * handed the raw problem, LM spends its first step in the null space, sees
	 * no cost reduction, and reports convergence. The unconstrained bones end up
	 * somewhere arbitrary and the constrained ones never move.
	 *
	 * Pinned on the seven bones that were already correct when the solve
	 * started, because that is where the damage shows up most plainly: any
	 * movement there is pure fabrication.
	 */
	@Test
	fun regularisationStopsTheSolverScatteringBonesTheRecordingCannotSee() {
		val frames = record(SensorError())

		val regularised = solve(frames, useLevenbergMarquardt = true, bodyProportionFactor = 0f)
		val raw = solve(frames, useLevenbergMarquardt = true, bodyProportionFactor = 0f, regularisation = 0f)

		println(header())
		println(regularised.row("LM"))
		println(raw.row("LM/noReg"))

		assertTrue(
			raw.otherError > 10f * regularised.otherError,
			"unregularised LM was expected to scatter the already-correct bones and regularised " +
				"LM to leave them alone, but they moved ${fmt(raw.otherError)}m and " +
				"${fmt(regularised.otherError)}m respectively",
		)
		assertTrue(
			regularised.splitError < raw.splitError,
			"regularised LM recovered the split no better than unregularised LM " +
				"(${fmt(regularised.splitError)}m vs ${fmt(raw.splitError)}m), so the null-space " +
				"step was not what was stopping it",
		)
	}

	/**
	 * The result issue #7 predicted, once the objective is well posed and the
	 * solver is not lost in the null space: LM wins, on both the bones the
	 * recording determines and the bones it does not.
	 *
	 * Run with the proportion term off, which is the only configuration in which
	 * the two solvers are being asked the same question -- see
	 * [theProportionTermDominatesTheSquaredObjectiveAndPrefersStockProportions].
	 */
	@Test
	fun onASlideOnlyObjectiveLevenbergMarquardtBeatsTheGreedySearch() {
		val frames = record(SensorError())

		val lm = solve(frames, useLevenbergMarquardt = true, bodyProportionFactor = 0f)
		val greedy = solve(frames, useLevenbergMarquardt = false, bodyProportionFactor = 0f)

		println(header())
		println(lm.row("LM"))
		println(greedy.row("greedy"))
		println("true: ${fmt(truth(SkeletonConfigOffsets.UPPER_LEG))} / ${fmt(truth(SkeletonConfigOffsets.LOWER_LEG))}")

		assertTrue(
			lm.splitError < greedy.splitError,
			"LM landed ${fmt(lm.splitError)}m from the true thigh/shin split and the greedy " +
				"search ${fmt(greedy.splitError)}m",
		)
		assertTrue(
			lm.otherError < greedy.otherError,
			"LM moved the already-correct bones ${fmt(lm.otherError)}m and the greedy search " +
				"${fmt(greedy.otherError)}m",
		)
	}

	/**
	 * At the sensor error a well-mounted set actually reports, the lead holds.
	 */
	@Test
	fun levenbergMarquardtKeepsThatLeadAtTypicalSensorError() {
		val frames = record(TYPICAL)

		val lm = solve(frames, useLevenbergMarquardt = true, bodyProportionFactor = 0f)
		val greedy = solve(frames, useLevenbergMarquardt = false, bodyProportionFactor = 0f)

		println(header())
		println(lm.row("LM"))
		println(greedy.row("greedy"))

		assertTrue(
			lm.splitError <= greedy.splitError + TIE_TOLERANCE,
			"under typical sensor error the greedy search recovered the thigh/shin split better " +
				"than LM: greedy ${fmt(greedy.splitError)}m vs LM ${fmt(lm.splitError)}m",
		)
	}

	/**
	 * ## The greedy path's justification, measured
	 *
	 * The reason the greedy search is still the default is a belief that it
	 * holds up on imperfect data. That belief turns out to be correct, and the
	 * margin is not small.
	 *
	 * Pushed to a bad session -- 2° of jitter, 5° of mounting error, drift --
	 * the slide-only least-squares objective stops having a sane optimum. Slide
	 * cannot be driven to zero, because no set of bone lengths explains
	 * corrupted rotations, and the cheapest remaining way to make the feet move
	 * less is to make the legs shorter. The height constraint permits it as long
	 * as the torso grows to compensate, so the solve walks off to a body that is
	 * nearly half legs by construction. LM finds that point accurately; it is a
	 * genuine optimum of what it was asked to minimise.
	 *
	 * The greedy search does not go there, and not by luck. It only ever accepts
	 * a per-bone change that reduces the error, one bone at a time, so it cannot
	 * take the coordinated leg-down/torso-up trade that the collapse requires --
	 * no single step of it is an improvement.
	 *
	 * This is what a proportion prior is for, and it is why the answer is not
	 * "turn the prior off": with the prior on, the same objective refuses to
	 * move at all (see
	 * [theProportionTermDominatesTheSquaredObjectiveAndPrefersStockProportions]).
	 * Both configurations fail, in opposite directions, and the weight that
	 * would sit between them has to be re-derived for a sum of squares against
	 * data that shows what real corruption looks like -- issue #15.
	 */
	@Test
	fun atPoorSensorErrorTheLeastSquaresPathCollapsesToADegenerateBody() {
		val frames = record(POOR)

		val lm = solve(frames, useLevenbergMarquardt = true, bodyProportionFactor = 0f)
		val greedy = solve(frames, useLevenbergMarquardt = false, bodyProportionFactor = 0f)

		println(header())
		println(lm.row("LM"))
		println(greedy.row("greedy"))
		println("true: ${fmt(truth(SkeletonConfigOffsets.UPPER_LEG))} / ${fmt(truth(SkeletonConfigOffsets.LOWER_LEG))}")

		val lmLegs = lm.lengths.getValue(SkeletonConfigOffsets.UPPER_LEG) +
			lm.lengths.getValue(SkeletonConfigOffsets.LOWER_LEG)
		val greedyLegs = greedy.lengths.getValue(SkeletonConfigOffsets.UPPER_LEG) +
			greedy.lengths.getValue(SkeletonConfigOffsets.LOWER_LEG)
		val trueLegs = truth(SkeletonConfigOffsets.UPPER_LEG) + truth(SkeletonConfigOffsets.LOWER_LEG)
		println("total leg length -- true ${fmt(trueLegs)}, LM ${fmt(lmLegs)}, greedy ${fmt(greedyLegs)}")

		assertTrue(
			lmLegs < 0.75f * trueLegs,
			"LM was expected to shrink the legs away under this much sensor error, which is the " +
				"failure the greedy path avoids, but its total leg length came out ${fmt(lmLegs)}m " +
				"against a true ${fmt(trueLegs)}m. If this has stopped happening the slide-only " +
				"objective has become usable on corrupted data and issue #7 step 4 can be revisited.",
		)
		assertTrue(
			greedyLegs > 0.9f * trueLegs,
			"the greedy search was expected to hold the legs near their true total, but it came " +
				"out ${fmt(greedyLegs)}m against ${fmt(trueLegs)}m",
		)
		assertTrue(
			greedy.splitError < lm.splitError,
			"the greedy search was expected to be the more robust of the two here -- that is the " +
				"argument for keeping it -- but LM scored ${fmt(lm.splitError)}m against " +
				"${fmt(greedy.splitError)}m",
		)
	}

	/**
	 * The verdict on step 4, stated as a test so it cannot quietly stop being
	 * true.
	 *
	 * At stock settings -- which is what a user gets -- the greedy search is
	 * still the better of the two, for the reason
	 * [theProportionTermDominatesTheSquaredObjectiveAndPrefersStockProportions]
	 * measures. `initialAdjustRate` and `adjustRateDecay` therefore stay, and
	 * `useLevenbergMarquardt` stays off by default.
	 *
	 * When this test starts failing, that is the signal to revisit: it means the
	 * error weights have been re-derived for the squared objective and the
	 * least-squares path has caught up at stock settings.
	 */
	@Test
	fun atStockSettingsTheGreedySearchIsStillTheBetterDefault() {
		val frames = record(SensorError())

		val lm = solve(frames, useLevenbergMarquardt = true)
		val greedy = solve(frames, useLevenbergMarquardt = false)

		println(header())
		println(lm.row("LM"))
		println(greedy.row("greedy"))
		println("true: ${fmt(truth(SkeletonConfigOffsets.UPPER_LEG))} / ${fmt(truth(SkeletonConfigOffsets.LOWER_LEG))}")

		assertTrue(
			greedy.splitError < lm.splitError,
			"at stock settings LM now recovers the thigh/shin split better than the greedy search " +
				"(${fmt(lm.splitError)}m vs ${fmt(greedy.splitError)}m). That is the gate on issue " +
				"#7 step 4: re-check the proportion weights, then retire initialAdjustRate and " +
				"adjustRateDecay and default useLevenbergMarquardt on.",
		)
		assertTrue(
			AutoBoneConfig().useLevenbergMarquardt.not(),
			"useLevenbergMarquardt is on by default, but the comparison above says the greedy " +
				"search is still the better of the two at stock settings",
		)
	}

	/**
	 * Slide is what AutoBone exists to minimise, so it is worth checking that
	 * the accuracy verdict is not an artefact of scoring against truth: a solver
	 * could land closer to the true lengths while leaving more slide, which
	 * would mean the objective and the goal disagree.
	 *
	 * Measured independently of both solvers -- replay the recording, set the
	 * solved lengths, watch the feet -- so it is not either one's own report of
	 * how it did.
	 */
	@Test
	fun theSolverThatLandsCloserToTruthAlsoLeavesLessSlide() {
		val frames = record(TYPICAL)

		val lm = solve(frames, useLevenbergMarquardt = true, bodyProportionFactor = 0f)
		val greedy = solve(frames, useLevenbergMarquardt = false, bodyProportionFactor = 0f)

		val lmSlide = measureSlide(frames, lm.lengths)
		val greedySlide = measureSlide(frames, greedy.lengths)
		val startSlide = measureSlide(frames, scoredOffsets.associateWith { it.defaultValue })

		println("mean foot slide per frame pair:")
		println("  start (stock proportions): ${fmt(startSlide)} m")
		println("  LM:                        ${fmt(lmSlide)} m  (split error ${fmt(lm.splitError)} m)")
		println("  greedy:                    ${fmt(greedySlide)} m  (split error ${fmt(greedy.splitError)} m)")

		assertTrue(
			lmSlide <= greedySlide,
			"LM landed closer to the true lengths but left more slide (${fmt(lmSlide)}m vs " +
				"${fmt(greedySlide)}m), so the objective and the goal disagree on this recording",
		)
	}

	// #endregion

	// #region Running a solver

	/** Tolerance for "no worse than", as a length in metres. Half a millimetre. */
	private val TIE_TOLERANCE = 5e-4f

	private class Outcome(
		val lengths: Map<SkeletonConfigOffsets, Float>,
		val splitError: Float,
		val otherError: Float,
		val height: Float,
		val seconds: Double,
	)

	private fun Outcome.row(label: String): String = "%-14s split %8s   other %8s   height %6s   %5.2fs   thigh %s shin %s".format(
		label,
		fmt(splitError),
		fmt(otherError),
		fmt(height),
		seconds,
		fmt(lengths.getValue(SkeletonConfigOffsets.UPPER_LEG)),
		fmt(lengths.getValue(SkeletonConfigOffsets.LOWER_LEG)),
	)

	private fun header(): String = "%-14s %-14s %-14s %-13s %-7s %s".format(
		"",
		"|split err|",
		"|other err|",
		"height",
		"time",
		"solved lengths",
	)

	private fun fmt(v: Float) = "%.4f".format(v)

	/**
	 * One full [AutoBone.processFrames] run, at stock settings apart from the
	 * flag under test.
	 */
	private fun solve(
		frames: PoseFrames,
		useLevenbergMarquardt: Boolean,
		bodyProportionFactor: Float = AutoBoneConfig().bodyProportionErrorFactor,
		regularisation: Float = AutoBoneConfig().lmRegularisation,
	): Outcome {
		val configManager = ConfigManager(
			// Never written: nothing here calls applyAndSaveConfig. Reading a
			// path that does not exist is how ConfigManager yields defaults.
			"build/tmp/autobone-head-to-head-nonexistent.yml",
		)
		configManager.loadConfig()

		// No live skeleton, so AutoBone starts from stock proportions -- which
		// is the honest starting point for a user who has not calibrated, and
		// is the same for both solvers.
		val autoBone = AutoBone(configManager) { null }

		val config = AutoBoneConfig().apply {
			this.useLevenbergMarquardt = useLevenbergMarquardt
			this.bodyProportionErrorFactor = bodyProportionFactor
			this.lmRegularisation = regularisation
			// This gate exists to stop a bad solve reaching a user's skeleton.
			// Here a bad solve is a result, not a failure, and throwing would
			// replace the number being compared with a stack trace.
			maxFinalError = Float.MAX_VALUE
		}

		val skeletonConfig = SkeletonConfig().apply {
			// Give both solvers the true height rather than letting it be
			// inferred from the recording's headset track, which the planting
			// translation moves around. The perturbation is height-neutral, so
			// this is exactly right and neither solver is scored on a scale
			// error it was never shown.
			hmdHeight = trueUserHeight
			floorHeight = 0f
		}

		val startedAt = System.nanoTime()
		val results = autoBone.processFrames(frames, config, skeletonConfig)
		val seconds = (System.nanoTime() - startedAt) / 1e9

		val lengths = EnumMap(results.configValues)
		return Outcome(
			lengths = lengths,
			splitError = rms(lengths, trueLengths.keys),
			otherError = rms(lengths, scoredOffsets - trueLengths.keys),
			height = results.finalHeight,
			seconds = seconds,
		)
	}

	private fun rms(
		lengths: Map<SkeletonConfigOffsets, Float>,
		over: Collection<SkeletonConfigOffsets>,
	): Float {
		if (over.isEmpty()) return 0f
		var sum = 0f
		for (offset in over) {
			val d = (lengths[offset] ?: truth(offset)) - truth(offset)
			sum += d * d
		}
		return sqrt(sum / over.size)
	}

	// #endregion

	// #region Direct cost measurement

	private class Costs(
		val startCost: Double,
		val startSlide: Double,
		val startOther: Double,
		val truthCost: Double,
		val truthSlide: Double,
		val truthOther: Double,
	)

	private fun sci(v: Double) = "%.3e".format(v)

	/**
	 * The least-squares cost at stock proportions and at the true lengths,
	 * split by term.
	 *
	 * Builds the objective the way `AutoBone.solveLeastSquares` does rather than
	 * going through a solver, because the question here is about the objective
	 * and not about anything an optimiser did to it. Normalised units
	 * throughout, matching `AutoBone`: the whole skeleton is scaled so the user
	 * is one unit tall and the recording is scaled to meet it.
	 */
	private fun costAt(frames: PoseFrames, bodyProportionFactor: Float): Costs {
		val config = AutoBoneConfig().apply { bodyProportionErrorFactor = bodyProportionFactor }
		val step = PoseFrameStep(
			config = config,
			serverConfig = null,
			frames = frames,
			onStep = {},
			data = AutoBoneStep(targetHmdHeight = trueUserHeight),
		)
		step.skeleton1.setLegTweaksEnabled(false)
		step.skeleton2.setLegTweaksEnabled(false)
		// Every bone, not just the adjusted ones: leaving the rest at metres
		// while the recording is in normalised units would put a scale error
		// into both measurements.
		for (skeleton in listOf(step.skeleton1, step.skeleton2)) {
			for (offset in SkeletonConfigOffsets.values) {
				skeleton.setOffset(offset, offset.defaultValue / trueUserHeight)
			}
		}
		step.framePlayer1.setScales(1f / trueUserHeight)
		step.framePlayer2.setScales(1f / trueUserHeight)

		val objective = AutoBoneObjective(
			step = step,
			adjustOffsets = scoredOffsets,
			normalizedHeight = 1f,
			framePairs = AutoBoneObjective.sampleFramePairs(frames.maxFrameCount, config, config.lmMaxFramePairs),
			terms = AutoBoneObjective.enabledTerms(config, AutoBoneErrorSet()),
			heightConstraintWeight = config.lmHeightConstraintWeight,
			estimateHeight = false,
			fixedHeight = trueUserHeight,
		)

		val start = objective.toParameters(scoredOffsets.associateWith { it.defaultValue / trueUserHeight })
		val atTruth = objective.toParameters(scoredOffsets.associateWith { truth(it) / trueUserHeight })

		val (startSlide, startOther) = splitCost(objective, start)
		val (truthSlide, truthOther) = splitCost(objective, atTruth)
		return Costs(
			startCost = startSlide + startOther,
			startSlide = startSlide,
			startOther = startOther,
			truthCost = truthSlide + truthOther,
			truthSlide = truthSlide,
			truthOther = truthOther,
		)
	}

	/** Sum of squared residuals, split into the slide term and everything else. */
	private fun splitCost(objective: AutoBoneObjective, parameters: DoubleArray): kotlin.Pair<Double, Double> {
		val residuals = objective.residuals(parameters)
		val names = objective.termNames
		var slide = 0.0
		var other = 0.0
		// Residuals run [term0, term1, ...] per frame pair, with the single
		// height-constraint row appended last.
		for (i in 0 until residuals.size - 1) {
			val square = residuals[i] * residuals[i]
			if (names[i % names.size] == "slide") slide += square else other += square
		}
		other += residuals.last() * residuals.last()
		return slide to other
	}

	// #endregion

	// #region Independent slide measurement

	/**
	 * Mean distance a planted foot moves between frames one apart, replaying
	 * the recording with [lengths].
	 *
	 * Deliberately does not go through either solver's error terms: this is the
	 * physical quantity both are proxies for.
	 */
	private fun measureSlide(
		frames: PoseFrames,
		lengths: Map<SkeletonConfigOffsets, Float>,
	): Float {
		val player = TrackerFramesPlayer(frames)
		val hpm = HumanPoseManager(player.trackers.toList())
		hpm.setLegTweaksEnabled(false)
		for ((offset, length) in lengths) hpm.setOffset(offset, length)

		val left = ArrayList<Vector3>(frameCount)
		val right = ArrayList<Vector3>(frameCount)
		for (frame in 0 until frameCount) {
			player.setCursors(frame)
			hpm.update()
			left.add(hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position)
			right.add(hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position)
		}

		var sum = 0f
		var count = 0
		for (i in 0 until frameCount - 1) {
			sum += (left[i + 1] - left[i]).len()
			sum += (right[i + 1] - right[i]).len()
			count += 2
		}
		return sum / count
	}

	// #endregion

	// #region Recording generation

	/**
	 * The three things the greedy path's "robust on real data" claim is a claim
	 * about.
	 *
	 * @param whiteNoiseRad per-frame independent rotation jitter.
	 * @param mountingErrorRad a fixed rotation per tracker, unknown to the
	 * solver. This is the one that does not average out, and the one a longer
	 * recording does not help with.
	 * @param yawDriftRadPerSec a per-tracker yaw ramp. Real drift is a random
	 * walk; a linear ramp with a random per-tracker rate is the part of it that
	 * matters over a recording this short.
	 */
	private class SensorError(
		val whiteNoiseRad: Float = 0f,
		val mountingErrorRad: Float = 0f,
		val yawDriftRadPerSec: Float = 0f,
	)

	private class Pose(
		val headYaw: Float,
		val chestPitch: Float,
		val chestYaw: Float,
		val hipPitch: Float,
		val thigh: Float,
		val calf: Float,
	)

	/**
	 * Torso and legs both move, so torso bone lengths are not trivially
	 * unobservable -- a length is only visible in foot slide if the segment
	 * above it rotates between the two frames being compared.
	 *
	 * Incommensurate frequencies, so the pose does not repeat within the
	 * recording and the frame pairs are not all near-duplicates of each other.
	 *
	 * ## Where yaw is allowed, and why only there
	 *
	 * Planting stays exact as long as the two ankles are a constant vector
	 * apart. That vector is `hipsWidth` along the hip's own X axis, and the leg
	 * chain below it, so it is constant iff the *hip* and *leg* rotations keep X
	 * fixed -- pitch only. Nothing above the hip is subject to that: the head
	 * and chest trackers can yaw freely, and they need to. Without them the
	 * head, upper chest, chest and hips-width columns are exactly rank
	 * deficient, and scoring a solver on a recording that cannot see four of the
	 * nine bones it adjusts measures the fixture rather than the solver.
	 */
	private fun pose(frame: Int): Pose {
		val t = frame.toFloat() / frameCount
		val tau = 2f * PI.toFloat()
		return Pose(
			headYaw = 0.55f * sin(tau * 1.3f * t + 0.2f),
			chestPitch = 0.16f * sin(tau * 1.0f * t),
			chestYaw = 0.40f * sin(tau * 0.7f * t + 1.4f),
			hipPitch = 0.12f * sin(tau * 1.7f * t + 0.6f),
			thigh = 0.44f * sin(tau * 2.3f * t),
			calf = 0.35f * sin(tau * 3.1f * t + 1.0f),
		)
	}

	private val nominalHeight = 1.6f

	/** The user height of the skeleton the recording is generated from. */
	private var trueUserHeight = 0f

	private fun record(error: SensorError): PoseFrames {
		// Seeded, so an assertion is not a coin flip. The same draws are used
		// at every noise level, so a level differs from another only in scale.
		val random = Random(11)

		val trackers = mkTrackers()
		val hmd = trackers[0]
		val chest = trackers[1]
		val hip = trackers[2]
		val legTrackers = trackers.drop(3)

		val hpm = HumanPoseManager(trackers)
		hpm.setLegTweaksEnabled(false)
		for ((offset, length) in trueLengths) hpm.setOffset(offset, length)
		trueUserHeight = hpm.userHeightFromConfig

		// Drawn once per tracker, then held for the whole recording -- that is
		// what makes these different from the per-frame jitter.
		val mounting = trackers.associate { it.id to randomRotation(error.mountingErrorRad, random) }
		val driftRate = trackers.associate { it.id to (random.nextGaussian().toFloat() * error.yawDriftRadPerSec) }

		val holders = trackers.map { TrackerFrames(it, FastList<TrackerFrame?>(frameCount)) }

		for (frame in 0 until frameCount) {
			val p = pose(frame)
			val truth = mapOf(
				hmd.id to yaw(p.headYaw),
				chest.id to (yaw(p.chestYaw) * pitch(p.chestPitch)),
				hip.id to pitch(p.hipPitch),
			) +
				legTrackers.associate {
					it.id to pitch(if (it.trackerPosition!!.name.contains("UPPER")) p.thigh else p.calf)
				}

			for (tracker in trackers) tracker.setRotation(truth.getValue(tracker.id))
			hmd.position = Vector3(0f, nominalHeight, 0f)
			hpm.update()

			// Plant: put the ankle midpoint on a fixed point. Both ankles are a
			// constant vector apart, so this plants each of them.
			val mid = (
				hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position +
					hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position
				) *
				0.5f
			hmd.position = Vector3(0f, nominalHeight, 0f) - mid
			hpm.update()

			// Corruption is applied only to what gets written down. The body
			// really was planted; the trackers report it imperfectly. Order
			// matters and is the physical one: drift and mounting rotate the
			// sensor's idea of the world frame, jitter is measurement noise on
			// top.
			for (tracker in trackers) {
				if (tracker.isHmd) continue
				val drift = yaw(driftRate.getValue(tracker.id) * frame * frameSeconds)
				var reported = drift * mounting.getValue(tracker.id) * truth.getValue(tracker.id)
				if (error.whiteNoiseRad > 0f) {
					reported *= randomRotation(error.whiteNoiseRad, random)
				}
				tracker.setRotation(reported)
			}

			for ((i, tracker) in trackers.withIndex()) {
				holders[i].addFrameFromTracker(tracker)
			}
		}

		return PoseFrames(FastList(holders))
	}

	/** A small rotation about a random axis, of scale [sigmaRad]. */
	private fun randomRotation(sigmaRad: Float, random: Random): Quaternion {
		if (sigmaRad <= 0f) return Quaternion.IDENTITY
		val half = sigmaRad / 2f
		return Quaternion(
			1f,
			(random.nextGaussian() * half).toFloat(),
			(random.nextGaussian() * half).toFloat(),
			(random.nextGaussian() * half).toFloat(),
		).unit()
	}

	/** Rotation about X, built directly to avoid any Euler-order dependence. */
	private fun pitch(angleRad: Float): Quaternion {
		val h = angleRad / 2f
		return Quaternion(cos(h), sin(h), 0f, 0f)
	}

	/** Rotation about Y, likewise. */
	private fun yaw(angleRad: Float): Quaternion {
		if (angleRad == 0f) return Quaternion.IDENTITY
		val h = angleRad / 2f
		return Quaternion(cos(h), 0f, sin(h), 0f)
	}

	private fun mkTrackers(): List<Tracker> = listOf(
		mkTracker(0, TrackerPosition.HEAD, isHmd = true),
		mkTracker(1, TrackerPosition.CHEST),
		mkTracker(2, TrackerPosition.HIP),
		mkTracker(3, TrackerPosition.LEFT_UPPER_LEG),
		mkTracker(4, TrackerPosition.LEFT_LOWER_LEG),
		mkTracker(5, TrackerPosition.RIGHT_UPPER_LEG),
		mkTracker(6, TrackerPosition.RIGHT_LOWER_LEG),
	)

	private fun mkTracker(id: Int, position: TrackerPosition, isHmd: Boolean = false): Tracker {
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

	// #endregion
}
