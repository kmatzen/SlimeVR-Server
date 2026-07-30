package dev.slimevr.unit

import dev.slimevr.autobone.AutoBoneStep
import dev.slimevr.autobone.PoseFrameStep
import dev.slimevr.autobone.leastsquares.AutoBoneErrorSet
import dev.slimevr.autobone.leastsquares.AutoBoneLevenbergMarquardt
import dev.slimevr.autobone.leastsquares.AutoBoneObjective
import dev.slimevr.config.AutoBoneConfig
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.poseframeformat.trackerdata.TrackerFrame
import dev.slimevr.poseframeformat.trackerdata.TrackerFrames
import dev.slimevr.tracking.processor.BoneType
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.eiren.util.collections.FastList
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.assertTrue

/**
 * Tests for the least-squares AutoBone path.
 *
 * ## How this is checkable without a recording
 *
 * AutoBone's whole premise is that wrong bone lengths make planted feet slide.
 * That gives a way to build a recording whose answer is known in advance:
 * generate the motion *from* a skeleton with chosen bone lengths, and after
 * solving each frame, translate the headset so the ankles land on the same
 * point every time. Translating the headset moves the whole chain rigidly, so
 * one correction plants the feet exactly.
 *
 * Replayed with the lengths it was generated from, that recording has zero
 * slide by construction. Replayed with any other lengths, the feet move. So the
 * optimum is known, and "did the solver find it" is a real question rather than
 * "did the number go down".
 *
 * The poses are not anatomically meaningful and are not trying to be. What
 * matters is which bone directions vary and how, because that is exactly what
 * determines whether a length is identifiable at all.
 *
 * ## What is not tested here
 *
 * A head-to-head against the existing greedy search on a real recording. That
 * needs `AutoBone`, which needs a `VRServer`, and more importantly it needs
 * captures the repository does not have (issue #15). The comparison here is
 * against first-order descent on the *same* objective, which is the specific
 * claim issue #7 makes about iteration counts.
 */
class AutoBoneLeastSquaresTests {

	private val frameCount = 60
	private val nominalHeight = 1.6f

	/** Bone lengths the recordings are generated from. */
	private val trueLengths = linkedMapOf(
		SkeletonConfigOffsets.UPPER_LEG to 0.42f,
		SkeletonConfigOffsets.LOWER_LEG to 0.46f,
	)

	private val solveOffsets = trueLengths.keys.toList()

	/**
	 * The knee bends, so the thigh and the shin point in different directions
	 * and those directions vary independently over the recording. Both lengths
	 * are identifiable.
	 */
	private fun bentKneePose(frame: Int): Pose {
		val t = frame.toFloat() / frameCount
		return Pose(
			thighPitch = 0.44f * sin(2f * PI.toFloat() * t),
			calfPitch = 0.35f * sin(4f * PI.toFloat() * t + 1f),
		)
	}

	/**
	 * The knee never bends: the thigh and shin rotate together, so the leg
	 * swings as one rigid segment.
	 *
	 * Its total length is observable; the split between thigh and shin is not,
	 * because no measurement in the recording distinguishes them. This is the
	 * "user never bent their knees" case issue #7 names, and the case a solver
	 * that reports only a number cannot tell you about.
	 */
	private fun straightKneePose(frame: Int): Pose {
		val t = frame.toFloat() / frameCount
		val angle = 0.44f * sin(2f * PI.toFloat() * t)
		return Pose(thighPitch = angle, calfPitch = angle)
	}

	private class Pose(val thighPitch: Float, val calfPitch: Float)

	@Test
	fun recoversKnownBoneLengthsFromABentKneeRecording() {
		val frames = record(::bentKneePose)
		val step = mkStep(frames)
		val objective = mkObjective(step, frames)

		// Start well away from the answer, and in opposite directions, so
		// getting back cannot be luck: the thigh starts 24% short and the shin
		// 22% long, which also swaps their order.
		val start = objective.toParameters(
			linkedMapOf(
				SkeletonConfigOffsets.UPPER_LEG to 0.32f,
				SkeletonConfigOffsets.LOWER_LEG to 0.56f,
			),
		)

		val startError = objective.meanStepError(start)
		val solution = AutoBoneLevenbergMarquardt.solve(objective, start, mkConfig())

		println(solution.report())
		println("mean step error: $startError -> ${objective.meanStepError(objective.toParameters(solution.lengths))}")

		for ((offset, expected) in trueLengths) {
			val got = solution.lengths.getValue(offset)
			assertTrue(
				abs(got - expected) < 0.01f,
				"$offset: expected ${expected}m, solved ${got}m from a start of " +
					"${if (offset == SkeletonConfigOffsets.UPPER_LEG) 0.32f else 0.56f}m",
			)
		}

		assertTrue(solution.converged, "did not converge: ${solution.message}")
	}

	/**
	 * The recovered lengths should carry small uncertainty when the recording
	 * determines them, which is the other half of the claim: a covariance that
	 * is large everywhere is as useless as no covariance at all.
	 */
	@Test
	fun aBentKneeRecordingReportsTightUncertainty() {
		val frames = record(::bentKneePose, sensorNoiseRad)
		val objective = mkObjective(mkStep(frames), frames)
		val solution = AutoBoneLevenbergMarquardt.solve(
			objective,
			objective.toParameters(linkedMapOf(SkeletonConfigOffsets.UPPER_LEG to 0.32f, SkeletonConfigOffsets.LOWER_LEG to 0.56f)),
			mkConfig(),
		)

		println(solution.report())

		assertTrue(
			solution.residualVariance > 0.0,
			"noise was applied but the fit came out exact, so the uncertainties below are " +
				"vacuously zero rather than small",
		)
		assertTrue(
			solution.poorlyDetermined(threshold = 0.05f).isEmpty(),
			"a recording that determines both lengths reported them as uncertain: " +
				solution.relativeSigma,
		)

		// An uncertainty is only worth reporting if it is calibrated. The true
		// lengths are known here, so the error bar can be checked against them
		// rather than merely being small: a σ that does not bracket the truth
		// is a confident wrong answer, which is the failure mode this whole
		// change exists to remove.
		for ((offset, expected) in trueLengths) {
			val got = solution.lengths.getValue(offset)
			val sd = solution.sigma.getValue(offset)
			assertTrue(
				abs(got - expected) < 2f * sd,
				"$offset came out ${got}m ± ${sd}m, which does not bracket the true ${expected}m " +
					"within 2σ; the reported uncertainty is not calibrated",
			)
		}
	}

	/**
	 * The point of the whole exercise.
	 *
	 * With a rigid leg the sum of thigh and shin is determined and the
	 * difference is not. A solver that returns only lengths reports this exactly
	 * as confidently as the bent-knee case; the covariance has to say otherwise,
	 * and it has to say it about the *combination*, since either length on its
	 * own is free to move as long as the other compensates.
	 */
	@Test
	fun aStraightKneeRecordingReportsTheSplitAsUndetermined() {
		val bent = solveFor(::bentKneePose)
		val straight = solveFor(::straightKneePose)

		println("bent knee:     " + bent.report())
		println("straight knee: " + straight.report())

		val bentWorst = bent.worstDirection!!
		val straightWorst = straight.worstDirection!!

		// The worst-determined direction must be the thigh/shin difference:
		// components of opposite sign and comparable size.
		val thigh = straightWorst.components.getValue(SkeletonConfigOffsets.UPPER_LEG.configKey)
		val shin = straightWorst.components.getValue(SkeletonConfigOffsets.LOWER_LEG.configKey)
		assertTrue(
			thigh * shin < 0.0,
			"the worst-determined direction should be thigh minus shin, got $thigh and $shin",
		)
		assertTrue(
			abs(abs(thigh) - abs(shin)) < 0.3,
			"the worst-determined direction should weight thigh and shin comparably, got $thigh and $shin",
		)

		// And it must be markedly worse than when the knee bends.
		assertTrue(
			straightWorst.sigma > bentWorst.sigma * 5.0,
			"a rigid leg left the split uncertainty at ${straightWorst.sigma}, barely worse than " +
				"the bent-knee recording's ${bentWorst.sigma}; the covariance is not detecting " +
				"non-identifiability",
		)
		assertTrue(
			straight.conditionNumber > bent.conditionNumber * 20.0,
			"condition number ${straight.conditionNumber} vs ${bent.conditionNumber}: a rigid leg " +
				"should be far closer to singular",
		)

		// The sharpest form of the claim, and the one that shows the covariance
		// is describing this recording rather than just being large: with a
		// rigid leg the *total* length is still pinned down, and only the split
		// between the two bones is lost. A solver reporting lengths alone
		// cannot distinguish that from having got both right.
		val trueSum = trueLengths.values.sum()
		val straightSum = straight.lengths.values.sum()
		val straightSplitError = abs(straight.lengths.getValue(SkeletonConfigOffsets.UPPER_LEG) - trueLengths.getValue(SkeletonConfigOffsets.UPPER_LEG))

		println("straight knee: sum ${straightSum}m (true ${trueSum}m), split off by ${straightSplitError}m")

		assertTrue(
			abs(straightSum - trueSum) < 0.01f,
			"the rigid-leg recording should still determine total leg length, got $straightSum vs $trueSum",
		)
		assertTrue(
			straightSplitError > 0.02f,
			"the rigid-leg recording was expected to lose the thigh/shin split, but it came back " +
				"within ${straightSplitError}m -- if the split is recoverable here, this test is " +
				"not demonstrating non-identifiability",
		)
	}

	/**
	 * Issue #7's headline claim, on the objective it is actually about.
	 *
	 * The baseline is fixed-schedule first-order descent using the same
	 * decaying step rule as `AutoBone.decayFunc`, on the same residual
	 * objective, with gradients from the same finite differences. That isolates
	 * the solver from everything else.
	 *
	 * Counted in *objective evaluations*, not iterations, so LM's finite
	 * difference Jacobian is charged for honestly rather than hidden inside an
	 * iteration count.
	 */
	@Test
	fun levenbergMarquardtBeatsFirstOrderDescentPerEvaluation() {
		val frames = record(::bentKneePose)
		val startLengths = linkedMapOf(
			SkeletonConfigOffsets.UPPER_LEG to 0.32f,
			SkeletonConfigOffsets.LOWER_LEG to 0.56f,
		)

		val lmObjective = mkObjective(mkStep(frames), frames)
		val lm = AutoBoneLevenbergMarquardt.solve(
			lmObjective,
			lmObjective.toParameters(startLengths),
			mkConfig(),
		)
		val lmEvaluations = lmObjective.evaluations
		val lmError = lmObjective.meanStepError(lmObjective.toParameters(lm.lengths))

		// Give descent its best shot rather than one arbitrary rate. A fixed
		// schedule that has to be swept to work is the complaint issue #7 makes
		// about it, so the fair comparison is against the swept best -- and
		// sweeping costs an evaluation budget LM never spends.
		var bestGdError = Double.MAX_VALUE
		var bestGdLengths = startLengths.toMap()
		var bestRate = 0.0
		for (rate in listOf(1.0, 10.0, 1e2, 1e3, 1e4, 1e5, 1e6)) {
			val gdObjective = mkObjective(mkStep(frames), frames)
			val gd = gradientDescent(
				gdObjective,
				gdObjective.toParameters(startLengths),
				lmEvaluations,
				initialRate = rate,
			)
			val error = gdObjective.meanStepError(gd)
			println("GD rate %-8s -> mean step error %.3e, lengths %s".format(rate, error, gdObjective.toLengths(gd)))
			if (error < bestGdError) {
				bestGdError = error
				bestGdLengths = gdObjective.toLengths(gd)
				bestRate = rate
			}
		}

		val gdError = bestGdError
		val gdLengths = bestGdLengths
		println("LM  : ${lm.lengths}, mean step error $lmError, $lmEvaluations evaluations")
		println("GD  : $gdLengths, mean step error $gdError, best of 7 swept rates (best rate $bestRate)")
		println("true: $trueLengths")

		assertTrue(
			lmError <= gdError,
			"LM reached $lmError, first-order descent reached $gdError with the same evaluation budget",
		)

		val lmDistance = distanceToTruth(lm.lengths)
		val gdDistance = distanceToTruth(gdLengths)
		assertTrue(
			lmDistance < gdDistance,
			"LM landed ${lmDistance}m from the true lengths, descent ${gdDistance}m",
		)
	}

	/**
	 * The residual vector and the scalar the existing optimiser reports have to
	 * agree about when the fit is perfect, or they are not measuring the same
	 * recording.
	 *
	 * They are deliberately *not* the same function elsewhere -- one is a sum of
	 * squares and the other the square of a sum -- so this pins the only place
	 * they must coincide.
	 */
	@Test
	fun theTrueLengthsMakeBothTheResidualsAndTheStepErrorVanish() {
		val frames = record(::bentKneePose)
		val objective = mkObjective(mkStep(frames), frames)
		val truth = objective.toParameters(trueLengths)

		val residuals = objective.residuals(truth)
		// The last residual is the height constraint, which is satisfied by
		// construction here; the rest are the slide terms.
		val slideResiduals = residuals.dropLast(1)
		val worst = slideResiduals.maxOf { abs(it) }

		println("worst slide residual at the true lengths: $worst")
		println("mean step error at the true lengths: ${objective.meanStepError(truth)}")

		assertTrue(
			worst < 1e-4,
			"the recording was generated from these lengths, so replaying it with them should " +
				"produce no slide, but the worst residual is $worst",
		)
		assertTrue(objective.meanStepError(truth) < 1e-4)
	}

	/**
	 * Height solved jointly with the bone lengths, rather than by the greedy
	 * path's separate one-dimensional line search.
	 *
	 * The recording is generated at scale 1, so the correct height parameter is
	 * exactly 1.0 by construction and starting elsewhere is a 15% error in how
	 * large the user is. Both the height and the two bone lengths have to come
	 * back, which is the part a separate line search cannot promise: it cannot
	 * trade one against the other within a step.
	 */
	@Test
	fun recoversHeightJointlyWithBoneLengths() {
		val frames = record(::bentKneePose)
		val objective = mkObjective(mkStep(frames), frames, estimateHeight = true)

		val start = objective.toParameters(
			linkedMapOf(
				SkeletonConfigOffsets.UPPER_LEG to 0.36f,
				SkeletonConfigOffsets.LOWER_LEG to 0.52f,
			),
			height = 1.15f,
		)

		val solution = AutoBoneLevenbergMarquardt.solve(objective, start, mkConfig())
		println(solution.report())

		val height = solution.height
		assertTrue(height != null, "height was solved for but not reported")
		assertTrue(
			abs(height!! - 1f) < 0.01f,
			"the recording was generated at scale 1, so height should return to 1.0; got $height from a start of 1.15",
		)
		for ((offset, expected) in trueLengths) {
			val got = solution.lengths.getValue(offset)
			assertTrue(
				abs(got - expected) < 0.01f,
				"$offset: expected ${expected}m, solved ${got}m with height free",
			)
		}
	}

	/**
	 * Freeing height must not quietly make the problem degenerate.
	 *
	 * A uniform scale-up of every bone paired with a matching change in height
	 * is the obvious candidate for a direction the data cannot see, and if it
	 * were one the covariance would be meaningless rather than merely wide.
	 * That it is not is a property of this objective worth pinning: the
	 * recording's tracker positions are scaled by `1/height` while the bones
	 * are not, so the two do not cancel.
	 */
	@Test
	fun freeingHeightDoesNotMakeTheProblemSingular() {
		val frames = record(::bentKneePose, sensorNoiseRad)
		val fixed = mkObjective(mkStep(frames), frames, estimateHeight = false)
		val free = mkObjective(mkStep(frames), frames, estimateHeight = true)

		val fixedSolution = AutoBoneLevenbergMarquardt.solve(fixed, fixed.toParameters(trueLengths), mkConfig())
		val freeSolution = AutoBoneLevenbergMarquardt.solve(free, free.toParameters(trueLengths, height = 1f), mkConfig())

		println("height fixed: " + fixedSolution.report())
		println("height free:  " + freeSolution.report())

		assertTrue(
			freeSolution.conditionNumber.isFinite(),
			"freeing height made JᵀJ singular (condition ${freeSolution.conditionNumber}); the " +
				"height parameter is redundant with the bone lengths",
		)
		// Wider than with height pinned -- an extra free parameter always costs
		// something -- but not by orders of magnitude, which is what a
		// near-degenerate direction would look like.
		assertTrue(
			freeSolution.conditionNumber < fixedSolution.conditionNumber * 100.0,
			"condition went from ${fixedSolution.conditionNumber} to ${freeSolution.conditionNumber} " +
				"when height was freed, which is the signature of a redundant parameter",
		)
	}

	// #region harness

	/** Per-tracker rotation noise for the covariance tests, about 0.6°. */
	private val sensorNoiseRad = 0.01f

	private fun solveFor(pose: (Int) -> Pose, noiseRad: Float = sensorNoiseRad) = record(pose, noiseRad).let { frames ->
		val objective = mkObjective(mkStep(frames), frames)
		AutoBoneLevenbergMarquardt.solve(objective, objective.toParameters(trueLengths), mkConfig())
	}

	private fun distanceToTruth(lengths: Map<SkeletonConfigOffsets, Float>): Float {
		var sum = 0f
		for ((offset, expected) in trueLengths) {
			val d = lengths.getValue(offset) - expected
			sum += d * d
		}
		return kotlin.math.sqrt(sum)
	}

	/**
	 * First-order descent on the same objective, with `AutoBone`'s decay rule.
	 *
	 * Given an evaluation budget rather than an iteration count so the
	 * comparison is like for like.
	 */
	private fun gradientDescent(
		objective: AutoBoneObjective,
		start: DoubleArray,
		evaluationBudget: Int,
		initialRate: Double,
		decay: Double = 1.0,
	): DoubleArray {
		val n = start.size
		val point = start.copyOf()
		val h = 5e-3
		var epoch = 0

		// Each iteration costs 2n evaluations for the central-difference
		// gradient, matching how LM's Jacobian is charged.
		while (objective.evaluations + 2 * n <= evaluationBudget) {
			val gradient = DoubleArray(n)
			val probe = point.copyOf()
			for (j in 0 until n) {
				probe[j] = point[j] + h
				val plus = objective.residuals(probe).sumOf { it * it }
				probe[j] = point[j] - h
				val minus = objective.residuals(probe).sumOf { it * it }
				probe[j] = point[j]
				gradient[j] = 0.5 * (plus - minus) / (2.0 * h)
			}
			val rate = initialRate / (1.0 + decay * epoch)
			for (j in 0 until n) point[j] -= rate * gradient[j]
			epoch++
		}
		return point
	}

	private fun mkConfig() = AutoBoneConfig().apply {
		useLevenbergMarquardt = true
		// Only slide, so identifiability is a property of the recording rather
		// than of a proportion prior quietly pinning the answer.
		slideErrorFactor = 1f
		bodyProportionErrorFactor = 0f
		minDataDistance = 1
		maxDataDistance = 8
		cursorIncrement = 4
		lmMaxFramePairs = 250
		lmMaxIterations = 60
		// Weak: the sum of the leg bones should be determined by the recording,
		// not handed to the solver. A strong constraint would make the
		// bent-knee recovery test a one-parameter problem.
		lmHeightConstraintWeight = 0.1f
	}

	private fun mkStep(frames: PoseFrames): PoseFrameStep<AutoBoneStep> = PoseFrameStep(
		config = mkConfig(),
		serverConfig = null,
		frames = frames,
		onStep = {},
		data = AutoBoneStep(targetHmdHeight = nominalHeight),
	)

	private fun mkObjective(
		step: PoseFrameStep<AutoBoneStep>,
		frames: PoseFrames,
		estimateHeight: Boolean = false,
	): AutoBoneObjective {
		val config = mkConfig()
		return AutoBoneObjective(
			step = step,
			adjustOffsets = solveOffsets,
			normalizedHeight = trueLengths.values.sum(),
			framePairs = AutoBoneObjective.sampleFramePairs(
				frameCount = frames.maxFrameCount,
				config = config,
				maxPairs = config.lmMaxFramePairs,
			),
			terms = AutoBoneObjective.enabledTerms(config, AutoBoneErrorSet()),
			heightConstraintWeight = config.lmHeightConstraintWeight,
			estimateHeight = estimateHeight,
			// The recording is generated unscaled, so 1.0 is "leave it alone"
			// and the true height parameter is exactly 1.0.
			fixedHeight = 1f,
		)
	}

	/**
	 * Generates a recording from [trueLengths] with both ankles planted at the
	 * origin on every frame.
	 *
	 * The planting is what makes the answer knowable: the headset position is
	 * chosen per frame so that the ankle midpoint lands on a fixed point, and
	 * since the chain hangs rigidly from the headset, one correction is exact.
	 */
	private fun record(pose: (Int) -> Pose, noiseRad: Float = 0f): PoseFrames {
		// Seeded, so a covariance assertion is not a coin flip.
		val random = java.util.Random(7)
		val trackers = mkTrackers()
		val hmd = trackers[0]
		val leftThigh = trackers[3]
		val leftCalf = trackers[4]
		val rightThigh = trackers[5]
		val rightCalf = trackers[6]

		val hpm = HumanPoseManager(trackers)
		hpm.setLegTweaksEnabled(false)
		for ((offset, length) in trueLengths) hpm.setOffset(offset, length)

		val holders = trackers.map { TrackerFrames(it, FastList<TrackerFrame?>(frameCount)) }

		for (frame in 0 until frameCount) {
			val p = pose(frame)
			val thigh = pitch(p.thighPitch)
			val calf = pitch(p.calfPitch)
			leftThigh.setRotation(thigh)
			rightThigh.setRotation(thigh)
			leftCalf.setRotation(calf)
			rightCalf.setRotation(calf)

			hmd.position = Vector3(0f, nominalHeight, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			hpm.update()

			// Ankle midpoint, measured exactly where SlideError measures it.
			val mid = (
				hpm.skeleton.getBone(BoneType.LEFT_LOWER_LEG).getTailPosition() +
					hpm.skeleton.getBone(BoneType.RIGHT_LOWER_LEG).getTailPosition()
				) *
				0.5f

			hmd.position = Vector3(0f, nominalHeight, 0f) - mid
			hpm.update()

			// Sensor noise is applied only to what gets written down: the body
			// really was planted, the trackers just report it imperfectly.
			// Without this the fit is exact, the residual variance is zero, and
			// every uncertainty comes out zero -- true, but useless as a test of
			// whether the covariance can tell determined from undetermined.
			if (noiseRad > 0f) {
				leftThigh.setRotation(jitter(thigh, noiseRad, random))
				rightThigh.setRotation(jitter(thigh, noiseRad, random))
				leftCalf.setRotation(jitter(calf, noiseRad, random))
				rightCalf.setRotation(jitter(calf, noiseRad, random))
			}

			for ((i, tracker) in trackers.withIndex()) {
				holders[i].addFrameFromTracker(tracker)
			}
		}

		return PoseFrames(FastList(holders))
	}

	/** [rotation] perturbed by a small random rotation of scale [sigmaRad]. */
	private fun jitter(rotation: Quaternion, sigmaRad: Float, random: java.util.Random): Quaternion {
		val half = sigmaRad / 2f
		val perturbation = Quaternion(
			1f,
			(random.nextGaussian() * half).toFloat(),
			(random.nextGaussian() * half).toFloat(),
			(random.nextGaussian() * half).toFloat(),
		).unit()
		return rotation * perturbation
	}

	/** Rotation about X, built directly to avoid any Euler-order dependence. */
	private fun pitch(angleRad: Float): Quaternion {
		val h = angleRad / 2f
		return Quaternion(cos(h), sin(h), 0f, 0f)
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
