package dev.slimevr.replay

import dev.slimevr.config.LocalizerConfig
import dev.slimevr.tracking.processor.HumanPoseManager
import dev.slimevr.tracking.processor.config.SkeletonConfigToggles
import dev.slimevr.tracking.processor.skeleton.MovementStates
import dev.slimevr.tracking.trackers.Tracker
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import org.junit.jupiter.api.Test
import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Replay tests for [dev.slimevr.tracking.processor.skeleton.Localizer], the
 * global-translation estimator used when nothing supplies an absolute position.
 *
 * ## Why this needs its own harness
 *
 * `Localizer.update()` opens with
 *
 * ```kotlin
 * if (skeleton.headTracker != null && skeleton.headTracker!!.hasPosition) return
 * ```
 *
 * and every existing replay harness gives its HMD a position. So none of them
 * execute a single line of this class. The head tracker here is rotation-only,
 * which is the configuration `Localizer` exists to serve: standalone or
 * HMD-less use, where translation has to be inferred from the body.
 *
 * That early return also bounds what these tests are worth. Nothing measured
 * here is in the path for a user wearing a positional headset.
 *
 * ## What is being measured
 *
 * The `jump` sequence, whose ground truth is exact --
 * [SyntheticMotion.jumpComHeight] is a closed-form height profile and the
 * acceleration fed to the torso tracker is its analytic second derivative. So
 * the estimator is handed a noise-free acceleration and asked to recover the
 * height it came from, and the residual is method error with nothing else mixed
 * in.
 *
 * The quantity compared is the mass-weighted centre of mass out of
 * `LegTweaksBuffer`, not the head bone. `Localizer` is a servo: it shifts the
 * skeleton root until the CoM sits on its predicted target, so the CoM is the
 * variable it actually controls and the root is only how it gets there. Reading
 * the root instead would charge the estimator for the legs tucking, which is
 * pose, not translation error.
 *
 * ## What is *not* being measured
 *
 * The audit on issue #6 notes that during a jump the torso's acceleration and
 * the whole-body CoM's acceleration are different things -- legs tuck, arms
 * swing -- and that the discrepancy lands precisely in this case. This harness
 * sets them equal on purpose, to isolate the inversion error. A sequence where
 * they differ is a separate measurement and wants the real recordings from #15.
 */
class LocalizerReplayTest {

	private val rateHz = 100f

	/**
	 * Standing time prepended before the jump sequence starts.
	 *
	 * Not cosmetic. `Localizer` suppresses all upward CoM acceleration until the
	 * feet have been the world reference for `WARMUP_FRAMES` consecutive frames
	 * -- 100, so a full second at this rate -- to stop the skeleton flying away.
	 * A jump consists almost entirely of upward acceleration, so how long the
	 * user stood still beforehand decides whether the push-off is integrated or
	 * discarded.
	 *
	 * The default gives the existing path its best case: long enough that the
	 * clamp is released before the crouch begins. Measuring the incumbent with
	 * the clamp engaged would flatter the replacement for a reason that has
	 * nothing to do with ballistics. [repeatedJumpsAreEstimatedNoBetter] covers
	 * the other case deliberately.
	 */
	private val defaultLeadInSec = 1.5f

	/** One jump, plus standing on each end. */
	private val jumpFrames = (SyntheticMotion.jumpDurationSec * rateHz).toInt()

	/**
	 * Per-frame record of one replay.
	 *
	 * @param comHeightM estimated CoM height, relative to the standing pose
	 * @param truthHeightM ground-truth CoM height, relative to the standing pose
	 * @param reference which source `Localizer` took translation from
	 */
	data class Sample(
		val tSec: Float,
		val comHeightM: Float,
		val truthHeightM: Float,
		val reference: MovementStates,
		val inFlight: Boolean,
		val horizontalDriftM: Float,
		/** Which jump of a repeated sequence this frame belongs to. */
		val jumpIndex: Int,
	) {
		val errorM: Float get() = comHeightM - truthHeightM
	}

	data class Run(val samples: List<Sample>) {
		val flightSamples = samples.filter { it.inFlight }

		val jumpCount = (samples.maxOfOrNull { it.jumpIndex } ?: -1) + 1

		/** The same measurements restricted to one jump of a repeated sequence. */
		fun forJump(index: Int) = Run(samples.filter { it.jumpIndex == index })

		/** Frames during flight where the estimator agreed the feet were free. */
		val flightDetected = flightSamples.count { it.reference == MovementStates.FOLLOW_COM }

		val maxFlightErrorM = flightSamples.maxOfOrNull { abs(it.errorM) } ?: 0f

		/** Estimated peak height, against the true apex. */
		val estimatedApexM = flightSamples.maxOfOrNull { it.comHeightM } ?: 0f

		val apexErrorM = estimatedApexM - SyntheticMotion.jumpApexHeightM

		/**
		 * Height error once the body is standing again. A jump starts and ends
		 * on the same floor, so this is measurable with no ground truth at all
		 * -- it is the vertical analogue of the walk-and-return-to-start drift
		 * metric issue #6 proposes.
		 */
		val settledErrorM = samples.lastOrNull()?.errorM ?: 0f

		/**
		 * Total horizontal travel of the estimate from its standing position.
		 *
		 * The jump's *trajectory* is purely vertical, but its *pose* is not: the
		 * legs bend in the sagittal plane, which moves the mass-weighted centre
		 * of mass fore and aft during push-off and tuck. So this is not a pure
		 * error measure the way the vertical channel is, and it is reported
		 * rather than asserted on. Use [horizontalDriftRateMPerSec] to compare
		 * two configurations.
		 */
		val maxHorizontalDriftM = samples.maxOfOrNull { it.horizontalDriftM } ?: 0f

		/**
		 * Mean horizontal speed of the estimate over the frames where no foot was
		 * anchoring it.
		 *
		 * This is the horizontal comparison that is fair between configurations.
		 * Total drift is not: a planted foot pins horizontal translation, so a
		 * configuration that wrongly believes the feet are down during flight
		 * reports less drift for that reason alone -- it bought the number with a
		 * detection error, not with a better estimate. Dividing by the number of
		 * unanchored frames removes that advantage and asks the actual question,
		 * which is how fast the estimate wanders when nothing is holding it.
		 */
		val horizontalDriftRateMPerSec: Float
			get() {
				val unanchored = samples.filter { it.reference == MovementStates.FOLLOW_COM }
				if (unanchored.size < 2) return 0f
				var path = 0f
				var previous: Sample? = null
				for (sample in unanchored) {
					val last = previous
					// Only consecutive frames; a gap means the feet came back
					// down in between and the travel is not unanchored travel.
					if (last != null && sample.tSec - last.tSec < 1.5f / 100f) {
						path += abs(sample.horizontalDriftM - last.horizontalDriftM)
					}
					previous = sample
				}
				return path / (unanchored.size / 100f)
			}

		/** The summary metrics, for comparing two configurations. */
		fun toMap(): Map<String, Float> = linkedMapOf(
			"flightDetected" to flightDetected.toFloat(),
			"maxFlightError" to maxFlightErrorM,
			"apexError" to apexErrorM,
			"settledError" to settledErrorM,
			"maxHorizontalDrift" to maxHorizontalDriftM,
			"horizontalDriftRate" to horizontalDriftRateMPerSec,
		)

		fun report(label: String): String = buildString {
			append("%-28s".format(label))
			append(" flight=%3d/%3d".format(flightDetected, flightSamples.size))
			append(" maxErr=%7.4f".format(maxFlightErrorM))
			append(" apexErr=%+7.4f".format(apexErrorM))
			append(" settled=%+7.4f".format(settledErrorM))
			append(" horiz=%7.4f".format(maxHorizontalDriftM))
			append(" horizRate=%7.4f".format(horizontalDriftRateMPerSec))
		}
	}

	/**
	 * The ground truth has to be right before anything is measured against it.
	 *
	 * [SyntheticMotion.jumpComAccel] is asserted to be the second derivative of
	 * [SyntheticMotion.jumpComHeight] by integrating it twice and checking the
	 * result reproduces the profile. If these two ever drift apart, every error
	 * below is measured against a trajectory the input does not describe, and
	 * the whole harness silently becomes meaningless.
	 */
	@Test
	fun theGroundTruthIsSelfConsistent() {
		// Second-order midpoint integration, in double precision, on a step far
		// finer than the replay's own.
		//
		// The fine step is the part that matters, and not for the reason one
		// would guess. Within a phase the acceleration is linear in time, so
		// midpoint integration is exact there and the step size is irrelevant.
		// The error all comes from the phase *boundaries*: acceleration jumps by
		// 35 m/s^2 at landing, and a step straddling that instant gets charged
		// one value for its whole width. That leaves a velocity error of order
		// `da * dt`, which then persists for the rest of the sequence as a
		// position error of order `da * dt * t_remaining` -- at a 1 ms step,
		// 9 mm, which is not small next to what is being measured. It shrinks
		// linearly with the step, so the fix is simply to take a small one.
		//
		// Worth recording because the same effect is not available to the code
		// under test: a real pipeline runs at 100 Hz and cannot refine its way
		// around a takeoff that happens between two frames. That is a genuine
		// error source for anything integrating acceleration through this
		// motion, and one of the reasons a ballistic fit anchored at the
		// boundary is better conditioned than integration across it.
		val dt = 1e-5
		var v = 0.0
		var y = 0.0
		var worst = 0.0
		var worstAt = 0.0

		var t = 0.0
		while (t < SyntheticMotion.jumpDurationSec) {
			val a = SyntheticMotion.jumpComAccel((t + dt / 2.0).toFloat()).toDouble()
			y += (v + a * dt / 2.0) * dt
			v += a * dt
			t += dt

			val err = abs(y - SyntheticMotion.jumpComHeight(t.toFloat()))
			if (err > worst) {
				worst = err
				worstAt = t
			}
		}

		println("ground-truth double integration: worst error %.6f m at t=%.3f s".format(worst, worstAt))
		assertTrue(
			worst < 5e-4,
			"jumpComAccel does not integrate to jumpComHeight (worst $worst m at $worstAt s); " +
				"the acceleration handed to the tracker and the trajectory " +
				"asserted as truth describe different motions",
		)
	}

	/** The profile has to actually leave the floor, or there is no flight to test. */
	@Test
	fun theJumpProfileIsAJump() {
		assertTrue(
			SyntheticMotion.jumpApexHeightM > 0.15f,
			"apex is only ${SyntheticMotion.jumpApexHeightM} m; too small to be distinguishable from noise",
		)
		assertTrue(
			SyntheticMotion.jumpFlightDurationSec > 0.3f,
			"flight lasts only ${SyntheticMotion.jumpFlightDurationSec} s",
		)

		// Continuity of height across every phase boundary. A step here would
		// be an impossible teleport that the estimator would be blamed for.
		for (boundary in listOf(0.30f, SyntheticMotion.jumpTakeoffSec, SyntheticMotion.jumpLandingSec)) {
			val before = SyntheticMotion.jumpComHeight(boundary - 1e-4f)
			val after = SyntheticMotion.jumpComHeight(boundary + 1e-4f)
			assertTrue(
				abs(after - before) < 1e-3f,
				"jump height steps by ${after - before} m at t=$boundary s",
			)
		}
	}

	@Test
	fun replayIsDeterministic() {
		val a = replay()
		val b = replay()
		assertEquals(
			a.samples.map { it.comHeightM },
			b.samples.map { it.comHeightM },
			"Localizer replay is not reproducible; nothing measured from it means anything",
		)
	}

	/**
	 * Confirms the flight phase is entered at all.
	 *
	 * This is the assertion that keeps every other flight metric honest. If
	 * contact detection never reports both feet off the floor, `flightSamples`
	 * is filtered against a reference that is never `FOLLOW_COM`, the maxima are
	 * taken over a set with nothing interesting in it, and the numbers look
	 * excellent for the wrong reason.
	 */
	@Test
	fun theFlightPhaseIsDetected() {
		val run = replay()
		println(run.report("current"))

		assertTrue(
			run.flightSamples.isNotEmpty(),
			"the jump sequence reports no flight frames; check jumpInFlight",
		)
		// A third, not a half. The frames at takeoff and landing read as stance
		// however good the estimate is -- the feet are near the floor there by
		// definition -- so requiring most of flight would be gating on contact
		// detection at the transitions it is worst at, which is #5's subject and
		// not this test's. A third is enough for the flight metrics to be
		// measuring something.
		assertTrue(
			run.flightDetected > run.flightSamples.size / 3,
			"Localizer only agreed the feet were off the floor for " +
				"${run.flightDetected} of ${run.flightSamples.size} flight frames. " +
				"Every flight metric in this class is measured over that subset, " +
				"so they are not meaningful until this holds.",
		)
	}

	/**
	 * The baseline this issue's proposal has to beat, recorded as an assertion
	 * rather than a printout so it cannot quietly change.
	 *
	 * The bound is deliberately loose: it is not a statement that this accuracy
	 * is acceptable, it is a statement of what the accelerometer-integration
	 * path currently achieves on noise-free input. Tightening it is the point of
	 * the work, and the ballistic path is measured against the same numbers in
	 * [ballisticFlightBeatsAccelerationIntegration].
	 */
	@Test
	fun currentFlightErrorIsBounded() {
		val run = replay()
		println(run.report("current"))

		for (sample in run.samples) {
			assertTrue(
				sample.comHeightM.isFinite(),
				"CoM height went non-finite at t=${sample.tSec}",
			)
		}

		assertTrue(
			run.maxFlightErrorM < 1.0f,
			"vertical CoM error during flight reached ${run.maxFlightErrorM} m, " +
				"which is larger than the body is tall -- the estimate has diverged, " +
				"not merely drifted",
		)
	}

	/**
	 * The comparison issue #6 asks for: ballistic arc against accelerometer
	 * integration, on input where the right answer is known exactly.
	 *
	 * Both configurations replay the identical sequence; the only difference is
	 * the flag. The claim being pinned is that the arc gets the height of the
	 * jump substantially right where integration does not.
	 */
	@Test
	fun ballisticFlightBeatsAccelerationIntegration() {
		val current = replay()
		val ballistic = replay(ballisticFlight = true)

		println(current.report("integration"))
		println(ballistic.report("ballistic"))
		println(
			"true apex %.4f m | integration apex %.4f m | ballistic apex %.4f m".format(
				SyntheticMotion.jumpApexHeightM,
				current.estimatedApexM,
				ballistic.estimatedApexM,
			),
		)

		assertTrue(
			ballistic.toMap() != current.toMap(),
			"the flag changed nothing, so the ballistic path is not running and " +
				"this comparison is vacuous",
		)

		assertTrue(
			abs(ballistic.apexErrorM) < abs(current.apexErrorM),
			"the ballistic arc did not improve peak height: error " +
				"${ballistic.apexErrorM} m against integration's ${current.apexErrorM} m",
		)

		assertTrue(
			ballistic.maxFlightErrorM < current.maxFlightErrorM,
			"the ballistic arc did not reduce vertical error through flight: " +
				"${ballistic.maxFlightErrorM} m against integration's ${current.maxFlightErrorM} m",
		)

		// The arc is vertical-only, so the horizontal channel should be
		// untouched. Total drift is nonetheless higher, and legitimately so:
		// correct flight detection means more frames with no foot anchoring
		// horizontal translation, and the incumbent's lower total came from
		// believing the feet were down for 18 of 39 flight frames. Comparing
		// drift *rate* over unanchored frames removes that and asks whether the
		// horizontal estimate itself got worse, which it must not have.
		println(
			"horizontal drift rate: integration %.4f m/s, ballistic %.4f m/s".format(
				current.horizontalDriftRateMPerSec,
				ballistic.horizontalDriftRateMPerSec,
			),
		)
		assertTrue(
			ballistic.horizontalDriftRateMPerSec <= current.horizontalDriftRateMPerSec * 1.05f,
			"the ballistic arc is vertical-only, so horizontal drift rate should be " +
				"unchanged, but it went from ${current.horizontalDriftRateMPerSec} to " +
				"${ballistic.horizontalDriftRateMPerSec} m/s",
		)
	}

	/**
	 * The arc has to launch at roughly the true takeoff speed, because that one
	 * measurement determines the entire trajectory.
	 *
	 * Separated from the trajectory assertions deliberately. If the arc is
	 * inaccurate, this says whether the cause is the launch measurement or the
	 * extrapolation from it, and those have completely different fixes -- the
	 * first wants a better contact classifier (#5), the second is just physics.
	 */
	@Test
	fun theArcLaunchesAtRoughlyTheTrueTakeoffSpeed() {
		val launches = mutableListOf<Pair<Float, Float>>()
		replay(
			ballisticFlight = true,
			onTakeoff = { tSec, velocity -> launches.add(tSec to velocity.y) },
		)

		for ((tSec, speed) in launches) {
			println("takeoff at t=%.3f s, launch speed %+.3f m/s".format(tSec, speed))
		}

		// The fastest upward launch, not the first. Contact detection produces
		// brief spurious no-foot frames during the crouch -- the legs bend, the
		// leg-state thresholds lose the lock for a frame or two, and an arc is
		// launched from a body that is still on the ground. Those are real and
		// worth knowing about, but they are not the jump, and taking the first
		// takeoff would measure one of them instead.
		val launchSpeed = launches.maxOfOrNull { it.second } ?: 0f

		println(
			"best launch speed %.3f m/s, true %.3f m/s (%d takeoffs)".format(
				launchSpeed,
				SyntheticMotion.jumpTakeoffSpeedMPerSec,
				launches.size,
			),
		)

		assertTrue(
			launchSpeed > 0f,
			"no upward launch velocity was ever measured, so the arc never described a jump",
		)

		// Loose, and deliberately so. The launch is measured one frame after the
		// feet leave the floor, over a one-frame window, from a CoM derived from
		// approximate body proportions -- several tens of a percent is the honest
		// expectation. The tight claim is the one above: whatever this is, it
		// beats integrating.
		val ratio = launchSpeed / SyntheticMotion.jumpTakeoffSpeedMPerSec
		assertTrue(
			ratio > 0.5f && ratio < 1.5f,
			"launch speed $launchSpeed m/s is not within 50% of the true " +
				"${SyntheticMotion.jumpTakeoffSpeedMPerSec} m/s",
		)
	}

	/**
	 * The guard against mistaking a lost foot lock for flight.
	 *
	 * With the maximum flight time set to nothing, every frame of real flight
	 * exceeds it, so the arc is abandoned on the frame it launches and every
	 * frame falls back to the old path. The result must therefore match the old
	 * path -- if it does not, the fallback is not a fallback.
	 */
	@Test
	fun anImplausiblyLongFlightFallsBackToTheOldPath() {
		val current = replay()
		val guarded = replay(ballisticFlight = true, maxFlightSec = 0f)

		println(current.report("integration"))
		println(guarded.report("ballistic maxFlight=0"))

		assertEquals(
			current.samples.map { it.comHeightM },
			guarded.samples.map { it.comHeightM },
			"with the flight-time guard tripping immediately, the ballistic path " +
				"must be indistinguishable from the path it falls back to",
		)
	}

	/**
	 * The arc reports its own error at landing, and that report is consistent with
	 * the launch velocity being under-measured.
	 *
	 * This is the only feedback the method has. Nothing observes the body during
	 * flight, so the arc cannot be checked while it is running -- but takeoff and
	 * landing happen on the same floor, so the gap between where the arc finished
	 * and where the body was seen to land measures how wrong the launch velocity
	 * was, with no ground truth involved. A deployment would log exactly this.
	 *
	 * The sign is the substance of the test. The launch is read a frame or two
	 * after the feet actually leave the floor, by which point gravity has already
	 * taken some of the launch speed away, so the measured speed is low, so the arc
	 * describes a shorter and lower flight than the real one -- and by the time the
	 * body is observed to land, the arc has already fallen past the height it
	 * started from. It must finish low. An arc finishing high would mean the launch
	 * was over-measured, which would point at a different defect entirely.
	 */
	@Test
	fun theArcReportsFinishingLowBecauseTheLaunchIsMeasuredLate() {
		val landings = mutableListOf<Triple<Float, Float, Float>>()
		replay(
			ballisticFlight = true,
			onLanding = { tSec, error, actualFlight, predictedFlight ->
				// Only arcs that described a real jump. Contact detection also
				// produces brief no-foot blips during the crouch and the landing
				// absorb, whose arcs launch downward and last a frame or two.
				if (predictedFlight > 0.1f) {
					landings.add(Triple(tSec, error.y, actualFlight - predictedFlight))
				}
			},
		)

		for ((tSec, errorY, flightGap) in landings) {
			println(
				"landing at t=%.3f s: arc finished %+.4f m relative to observed, flight %+.3f s longer than predicted".format(
					tSec,
					errorY,
					flightGap,
				),
			)
		}

		assertTrue(
			landings.isNotEmpty(),
			"no arc lasted long enough to describe a jump, so there is no landing to report on",
		)

		val (_, errorY, flightGap) = landings.first()

		assertTrue(
			errorY < 0f,
			"the arc finished $errorY m relative to where the body was observed to " +
				"land. Finishing high means the launch velocity was over-measured, " +
				"which is the opposite of the late-detection bias this path has.",
		)

		// The same fact from the other side: real flight outlasts the flight the
		// arc predicted, because the arc was launched too slow.
		assertTrue(
			flightGap > 0f,
			"real flight was ${-flightGap} s shorter than the arc predicted, so the " +
				"launch velocity was over-measured rather than under-measured",
		)
	}

	/**
	 * Two jumps in a row are estimated identically badly, which locates the
	 * defect.
	 *
	 * `Localizer` has two mechanisms that suppress upward vertical velocity, and
	 * this separates them. The first is the anti-flyaway clamp, which discards
	 * upward CoM acceleration until the feet have been the world reference for
	 * 100 consecutive frames; `footFrames` resets whenever the reference is
	 * anything else, and a jump's own flight phase does exactly that, so a second
	 * jump 0.6 s later cannot have re-earned the release. The second is the floor
	 * clamp in `updateTargetCOM`, which zeroes vertical CoM velocity outright on
	 * every frame the lowest tracker is at or below the floor.
	 *
	 * The measurements say it is the floor clamp that matters. The default lead-in
	 * is long enough to release the anti-flyaway clamp before the crouch begins,
	 * and the estimated apex is still 0.02 m against a true 0.18 m -- so with that
	 * clamp demonstrably out of the way, the push-off still produces no height.
	 * Meanwhile the kinematic launch measurement reads the centre of mass rising
	 * at 1.3 m/s at takeoff, so the rise is present in the pose and is being
	 * discarded downstream. The floor clamp is what discards it: every frame of a
	 * push-off has a foot on the floor by definition, which is every frame the
	 * clamp fires.
	 *
	 * Hence "no better" rather than "worse", and hence the two jumps matching to
	 * the last digit: this is not a warmup artefact that a second attempt would
	 * shake off, it is structural. A body cannot acquire upward velocity through a
	 * channel that is reset to zero whenever a foot is touching the ground, and no
	 * amount of standing still first changes that.
	 */
	@Test
	fun repeatedJumpsAreEstimatedNoBetter() {
		val run = replay(repeats = 2)
		val first = run.forJump(0)
		val second = run.forJump(1)

		println(first.report("current jump 1"))
		println(second.report("current jump 2"))

		assertEquals(2, run.jumpCount, "expected two jumps in the replay")
		assertTrue(
			first.flightSamples.isNotEmpty() && second.flightSamples.isNotEmpty(),
			"both jumps must contain flight frames for this comparison to mean anything",
		)

		assertTrue(
			abs(second.apexErrorM) >= abs(first.apexErrorM) - 1e-3f,
			"the second jump was estimated better than the first " +
				"(apex error ${second.apexErrorM} m vs ${first.apexErrorM} m), so " +
				"something about the vertical channel does improve with repetition. " +
				"That contradicts the analysis above and this test no longer " +
				"measures what it claims.",
		)

		// Both jumps miss the height by essentially the whole jump, and by the
		// same amount. Pinned so that a change which fixes one and not the other
		// -- or which makes the failure depend on history -- is visible.
		assertTrue(
			abs(first.apexErrorM) > SyntheticMotion.jumpApexHeightM * 0.5f,
			"the uncorrected path recovered more than half the jump height " +
				"(apex error ${first.apexErrorM} m of ${SyntheticMotion.jumpApexHeightM} m); " +
				"if that is now true, the premise of issue #6 has changed",
		)
	}

	private fun replay(
		leadInSec: Float = defaultLeadInSec,
		repeats: Int = 1,
		ballisticFlight: Boolean = false,
		maxFlightSec: Float? = null,
		clock: FixedStepClock = FixedStepClock(1f / rateHz),
		onTakeoff: ((Float, Vector3) -> Unit)? = null,
		onLanding: ((Float, Vector3, Float, Float) -> Unit)? = null,
	): Run {
		// Rotation-only head tracker. This is the whole reason the harness
		// exists: give it a position and Localizer returns on its first line.
		val head = mkTracker(0, TrackerPosition.HEAD)
		val chest = mkTracker(1, TrackerPosition.CHEST)
		val hip = mkTracker(2, TrackerPosition.HIP)
		val leftThigh = mkTracker(3, TrackerPosition.LEFT_UPPER_LEG)
		val leftCalf = mkTracker(4, TrackerPosition.LEFT_LOWER_LEG)
		val rightThigh = mkTracker(5, TrackerPosition.RIGHT_UPPER_LEG)
		val rightCalf = mkTracker(6, TrackerPosition.RIGHT_LOWER_LEG)

		val trackers = listOf(head, chest, hip, leftThigh, leftCalf, rightThigh, rightCalf)
		val hpm = HumanPoseManager(trackers)
		hpm.skeleton.hasKneeTrackers = true

		// Localizer reads foot lock states out of the LegTweaks buffer, so the
		// corrections have to be on for it to have anything to anchor to.
		hpm.setLegTweaksEnabled(true)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, true)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, true)
		hpm.setToggle(SkeletonConfigToggles.SELF_LOCALIZATION, true)

		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val legTweaks = hpm.skeleton.legTweaks
		val localizer = hpm.skeleton.localizer

		localizer.config = LocalizerConfig().apply {
			useBallisticFlight = ballisticFlight
			maxFlightSec?.let { ballisticMaxFlightSec = it }
		}

		var currentTSec = 0f
		if (onTakeoff != null) {
			localizer.onTakeoff = { arc -> onTakeoff(currentTSec, arc.launchVelocity) }
		}
		if (onLanding != null) {
			localizer.onLanding = { arc, observed ->
				onLanding(
					currentTSec,
					arc.landingErrorM(observed),
					arc.flightTimeSec,
					arc.predictedFlightSec,
				)
			}
		}

		val samples = mutableListOf<Sample>()
		var standingComY: Float? = null
		var standingComXZ: Vector3? = null

		val leadInFrames = (leadInSec * rateHz).toInt()

		for (i in 0 until leadInFrames + jumpFrames * repeats) {
			// The lead-in is the profile held at t=0, which is standing. Holding
			// it rather than shifting the phase boundaries keeps the profile and
			// its analytic derivative untouched.
			val sinceLeadIn = i - leadInFrames
			val jumpIndex = if (sinceLeadIn < 0) 0 else sinceLeadIn / jumpFrames
			val t = if (sinceLeadIn < 0) 0f else (sinceLeadIn % jumpFrames) / rateHz
			currentTSec = t
			val frame = SyntheticMotion.at("jump", t)

			clock.advance()

			head.setRotation(Quaternion.IDENTITY)
			chest.setRotation(frame.chest)
			hip.setRotation(frame.hip)
			leftThigh.setRotation(frame.leftThigh)
			leftCalf.setRotation(frame.leftCalf)
			rightThigh.setRotation(frame.rightThigh)
			rightCalf.setRotation(frame.rightCalf)

			// getAcceleration() rotates the stored vector into world space by
			// the tracker's raw rotation, so what is stored has to be the
			// world-frame acceleration expressed back in tracker space.
			// Undoing it here rather than assuming identity keeps the ground
			// truth exact if these sequences ever pitch the torso.
			setWorldAcceleration(hip, frame.torsoAccel)
			setWorldAcceleration(chest, frame.torsoAccel)

			hpm.update()

			val com = legTweaks.bufferHead.centerOfMass

			// The first frames are the localizer warmup, during which it holds
			// the skeleton still by construction. Whatever CoM height that
			// settles at is the standing reference every later frame is
			// measured against -- an absolute height would just be measuring
			// the configured body proportions.
			if (standingComY == null && t >= 0.2f) {
				standingComY = com.y
				standingComXZ = Vector3(com.x, 0f, com.z)
			}

			if (standingComY != null) {
				val horizontal = Vector3(com.x, 0f, com.z) - standingComXZ!!
				samples.add(
					Sample(
						tSec = t,
						comHeightM = com.y - standingComY,
						truthHeightM = frame.comHeightM,
						reference = localizer.currentWorldReference,
						inFlight = frame.inFlight,
						horizontalDriftM = horizontal.len(),
						jumpIndex = jumpIndex,
					),
				)
			}
		}

		return Run(samples)
	}

	private fun setWorldAcceleration(tracker: Tracker, worldAccel: Vector3) {
		val rot = tracker.getRawRotation()
		tracker.setAcceleration(rot.inv().sandwich(worldAccel))
	}

	private fun mkTracker(id: Int, position: TrackerPosition): Tracker {
		val tracker = Tracker(
			device = null,
			id = id,
			name = position.name,
			trackerPosition = position,
			trackerNum = 0,
			hasPosition = false,
			hasRotation = true,
			isComputed = false,
			imuType = null,
			allowReset = true,
			allowMounting = true,
			isHmd = false,
			trackRotDirection = false,
		)
		tracker.status = TrackerStatus.OK
		return tracker
	}
}
