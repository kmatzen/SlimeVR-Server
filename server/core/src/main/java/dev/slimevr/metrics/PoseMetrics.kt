package dev.slimevr.metrics

import dev.slimevr.tracking.processor.BoneType
import dev.slimevr.tracking.processor.skeleton.HumanSkeleton
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * Pose-quality metrics, accumulated over a sequence of solved frames.
 *
 * These measure the failure modes the tracking stack is actually judged on:
 * feet that slide while planted, feet that sink through the floor, and feet
 * that disagree about where the floor is.
 *
 * The equivalent quantities already exist in `dev.slimevr.autobone.errors`, but
 * only as an objective function for the AutoBone optimiser -- they take two
 * [HumanSkeleton] instances and return a scalar to minimise. This class exposes
 * the same *ideas* in the shape a regression suite needs: accumulated over a
 * whole recording, in physical units, with no optimiser attached.
 *
 * All lengths are metres. SlimeVR's world frame is Y-up with the floor at y=0.
 */
class PoseMetricsAccumulator(
	/**
	 * A foot is treated as planted when its ankle is within this distance of
	 * the floor. Generous enough to cover a standing pose whose ankle never
	 * sits exactly at zero.
	 */
	private val plantedThresholdM: Float = 0.05f,
) {
	private var frames = 0
	private var durationSec = 0f

	private var leftPrev: Vector3? = null
	private var rightPrev: Vector3? = null

	private var slideTotalM = 0f
	private var plantedSec = 0f

	private var clipSumM = 0f
	private var clipMaxM = 0f
	private var clipFrames = 0

	private var heightDisagreementSumM = 0f
	private var heightDisagreementFrames = 0

	/**
	 * Records one solved frame from the skeleton's own bone tree.
	 *
	 * Note this observes the *solver's* output, before `LegTweaks`. The leg
	 * corrections are written into a buffer consumed by the computed trackers
	 * rather than back into the bone transforms, so floor clip and skating
	 * corrections are not visible here. To measure the pipeline's final output
	 * -- which is what SteamVR actually receives -- use [observeAnkles] with
	 * the computed foot trackers instead.
	 */
	fun observe(skeleton: HumanSkeleton, dtSec: Float) {
		observeAnkles(
			skeleton.getBone(BoneType.LEFT_LOWER_LEG).getTailPosition(),
			skeleton.getBone(BoneType.RIGHT_LOWER_LEG).getTailPosition(),
			dtSec,
		)
	}

	/** Records one solved frame from explicit ankle positions. */
	fun observeAnkles(left: Vector3, right: Vector3, dtSec: Float) {
		val leftPlanted = left.y <= plantedThresholdM
		val rightPlanted = right.y <= plantedThresholdM

		// Foot slide: horizontal travel of an ankle while it is planted. A
		// planted foot is in contact with the world and must not move; whatever
		// distance it covers is error, and it is the artifact users describe as
		// skating.
		if (leftPlanted) {
			leftPrev?.let { slideTotalM += horizontalDistance(it, left) }
			plantedSec += dtSec
		}
		if (rightPlanted) {
			rightPrev?.let { slideTotalM += horizontalDistance(it, right) }
			plantedSec += dtSec
		}
		leftPrev = if (leftPlanted) left else null
		rightPrev = if (rightPlanted) right else null

		// Floor clip: penetration below the floor plane.
		val deepest = -minOf(left.y, right.y, 0f)
		if (deepest > 0f) {
			clipSumM += deepest
			clipMaxM = max(clipMaxM, deepest)
			clipFrames++
		}

		// Both feet on the ground should agree about where the ground is.
		if (leftPlanted && rightPlanted) {
			heightDisagreementSumM += abs(left.y - right.y)
			heightDisagreementFrames++
		}

		frames++
		durationSec += dtSec
	}

	/**
	 * [heightM] is supplied by the caller rather than read from the skeleton,
	 * since user height lives on `HumanPoseManager` rather than on
	 * [HumanSkeleton].
	 */
	fun result(heightM: Float = 0f): PoseMetrics = PoseMetrics(
		frames = frames,
		durationSec = durationSec,
		// Normalised by planted time rather than wall time, so a sequence that
		// spends more time in the air is not flattered by the average.
		footSlideMPerSec = if (plantedSec > 0f) slideTotalM / plantedSec else 0f,
		footSlideTotalM = slideTotalM,
		floorClipMeanM = if (clipFrames > 0) clipSumM / clipFrames else 0f,
		floorClipMaxM = clipMaxM,
		floorClipFraction = if (frames > 0) clipFrames.toFloat() / frames else 0f,
		footHeightDisagreementM = if (heightDisagreementFrames > 0) {
			heightDisagreementSumM / heightDisagreementFrames
		} else {
			0f
		},
		heightM = heightM,
	)

	private fun horizontalDistance(a: Vector3, b: Vector3): Float {
		val dx = b.x - a.x
		val dz = b.z - a.z
		return sqrt(dx * dx + dz * dz)
	}
}

data class PoseMetrics(
	val frames: Int,
	val durationSec: Float,
	/** Mean horizontal ankle speed while planted. The foot-skating metric. */
	val footSlideMPerSec: Float,
	val footSlideTotalM: Float,
	val floorClipMeanM: Float,
	val floorClipMaxM: Float,
	/** Fraction of frames in which either foot was below the floor. */
	val floorClipFraction: Float,
	val footHeightDisagreementM: Float,
	val heightM: Float,
) {
	/** Flat key/value view, for baseline comparison and reporting. */
	fun toMap(): Map<String, Float> = linkedMapOf(
		"foot_slide_m_per_sec" to footSlideMPerSec,
		"foot_slide_total_m" to footSlideTotalM,
		"floor_clip_mean_m" to floorClipMeanM,
		"floor_clip_max_m" to floorClipMaxM,
		"floor_clip_fraction" to floorClipFraction,
		"foot_height_disagreement_m" to footHeightDisagreementM,
		"height_m" to heightM,
	)
}
