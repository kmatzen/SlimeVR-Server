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

/**
 * Drives a synthetic sequence through a real pipeline and records what the
 * contact detector decided, alongside what was true.
 *
 * Extracted from [ContactDetectionTest] so [ContactNoiseRobustnessTest] measures
 * the same thing rather than a second implementation of it. The baseline being
 * measured is the real [LegTweaksBuffer.checkState] inside a real
 * [HumanPoseManager] -- hysteresis, sensitivity scalars and all -- which is what
 * issue #5's audit insists a fair comparison requires.
 */
object ContactReplay {
	const val RATE_HZ = 100f
	const val FRAMES = 600

	/**
	 * Per-frame contact decisions from one replay, alongside what was true.
	 *
	 * Feet are kept separate rather than pooled: they are in antiphase in the
	 * walking sequence, so pooling would average a systematic lag on one foot
	 * against the same lag on the other and could hide a left/right asymmetry
	 * entirely.
	 */
	data class Run(
		val leftTruth: BooleanArray,
		val rightTruth: BooleanArray,
		val leftHeuristic: BooleanArray,
		val rightHeuristic: BooleanArray,
		val leftPositions: List<Vector3>,
		val rightPositions: List<Vector3>,
	)

	/**
	 * @param noise perturbation applied to every tracker rotation before the
	 *   solve. Null runs the clean sequence, which is what every measurement on
	 *   issue #5 was taken on.
	 */
	fun run(motion: String, noise: SensorNoise? = null): Run {
		noise?.reset()

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

		// The corrections have to be on: contact state is computed inside
		// LegTweaksBuffer and is only populated when the buffer is active.
		hpm.setLegTweaksEnabled(true)
		hpm.setToggle(SkeletonConfigToggles.SKATING_CORRECTION, true)
		hpm.setToggle(SkeletonConfigToggles.FLOOR_CLIP, true)

		val clock = FixedStepClock(1f / RATE_HZ)
		hpm.skeleton.legTweaks.clock = clock.clock
		hpm.skeleton.kinematicHeading.clock = clock.clock

		val leftTruth = BooleanArray(FRAMES)
		val rightTruth = BooleanArray(FRAMES)
		val leftHeuristic = BooleanArray(FRAMES)
		val rightHeuristic = BooleanArray(FRAMES)
		val leftPositions = mutableListOf<Vector3>()
		val rightPositions = mutableListOf<Vector3>()

		for (i in 0 until FRAMES) {
			val seconds = i / RATE_HZ
			val frame = SyntheticMotion.at(motion, seconds)
			clock.advance()

			fun perturbed(index: Int, rotation: Quaternion) = noise?.perturb(index, i, seconds, rotation) ?: rotation

			// The HMD is a positional reference rather than an IMU, so it is left
			// clean. Perturbing it would move the whole body and confound a
			// measurement about where the feet are with one about where the head
			// is.
			hmd.position = Vector3(0f, height * frame.headHeightFraction, 0f)
			hmd.setRotation(Quaternion.IDENTITY)
			chest.setRotation(perturbed(1, frame.chest))
			hip.setRotation(perturbed(2, frame.hip))
			leftThigh.setRotation(perturbed(3, frame.leftThigh))
			leftCalf.setRotation(perturbed(4, frame.leftCalf))
			rightThigh.setRotation(perturbed(5, frame.rightThigh))
			rightCalf.setRotation(perturbed(6, frame.rightCalf))

			hpm.update()

			// Truth comes from the sequence definition, so it is unaffected by
			// the noise: the foot really was where the clean motion put it, and
			// the detector's job is to say so from a corrupted view of it.
			leftTruth[i] = frame.leftFootContact
			rightTruth[i] = frame.rightFootContact

			val buffer = hpm.skeleton.legTweaks.bufferHead
			leftHeuristic[i] = buffer.leftLegState == LegTweaksBuffer.LOCKED
			rightHeuristic[i] = buffer.rightLegState == LegTweaksBuffer.LOCKED

			// The computed foot trackers, matching what the replay suite
			// measures elsewhere: this is the pipeline's actual output and the
			// only trajectory an offline labeller would have to work from.
			leftPositions.add(hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position)
			rightPositions.add(hpm.getComputedTracker(TrackerRole.RIGHT_FOOT).position)
		}

		return Run(
			leftTruth = leftTruth,
			rightTruth = rightTruth,
			leftHeuristic = leftHeuristic,
			rightHeuristic = rightHeuristic,
			leftPositions = leftPositions,
			rightPositions = rightPositions,
		)
	}

	/**
	 * The causal stillness rule: contact when the foot has stayed within a
	 * radius over a trailing window, and is near the floor.
	 *
	 * Deliberately not a mode on [OfflineContactLabeller]: that class's one
	 * guarantee is that it cannot run live, and adding a causal mode would make
	 * it a detector that happens to default to offline. This is a measurement
	 * probe -- the thing issue #5's finding is *about* -- not a component.
	 *
	 * Same radius, same height rule and same total window width as the labeller,
	 * with the window ending at the frame being labelled instead of straddling
	 * it. Nothing else differs, which is what makes the comparison a measurement
	 * of lookahead alone.
	 */
	fun trailingStillness(
		positions: List<Vector3>,
		labeller: OfflineContactLabeller = OfflineContactLabeller(),
	): BooleanArray {
		val window = labeller.stillnessWindowFrames
		val out = BooleanArray(positions.size)
		for (i in positions.indices) {
			val here = positions[i]
			var still = true
			for (j in maxOf(0, i - 2 * window)..i) {
				if ((positions[j] - here).len() > labeller.stillnessRadiusM) {
					still = false
					break
				}
			}
			out[i] = still && here.y <= labeller.floorLevelM + labeller.floorDistanceCutoffM
		}
		return out
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
