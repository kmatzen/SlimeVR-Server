package dev.slimevr.replay

import kotlin.math.abs

/**
 * Scores a sequence of contact decisions against ground truth.
 *
 * Two families of number, because they answer different questions and the
 * second is the one issue #5 actually cares about.
 *
 * **Frame-wise** -- precision, recall, F1 -- asks how often the decision was
 * right. It is dominated by the long stretches where the foot is obviously
 * planted or obviously in the air, which every method gets right, so it
 * saturates near 1 and hides the interesting part.
 *
 * **Transition timing** asks *when* the method noticed. Issue #5 puts this first
 * among the failure modes it lists:
 *
 * > Detect the contact but get its *timing* wrong by 50 ms → error injected at
 * > exactly the moment of highest acceleration
 *
 * A detector that is right 98% of the time but consistently three frames late is
 * worse, for translation, than one that is right 96% of the time with no lag --
 * because the frames it gets wrong are the ones where the foot is accelerating
 * hardest, and an anchor placed at the wrong instant injects exactly the error
 * an anchor exists to prevent. Frame-wise scoring cannot see that difference and
 * transition timing is entirely about it.
 */
object ContactMetrics {

	/**
	 * @param lagFrames signed offset of each detected transition from the true
	 *   one, positive meaning late. One entry per true transition that was
	 *   matched; unmatched transitions are counted in [missedTransitions].
	 */
	data class Result(
		val truePositives: Int,
		val falsePositives: Int,
		val falseNegatives: Int,
		val trueNegatives: Int,
		val lagFrames: List<Int>,
		val missedTransitions: Int,
		val spuriousTransitions: Int,
	) {
		val precision: Float
			get() = if (truePositives + falsePositives == 0) 0f else truePositives.toFloat() / (truePositives + falsePositives)

		val recall: Float
			get() = if (truePositives + falseNegatives == 0) 0f else truePositives.toFloat() / (truePositives + falseNegatives)

		val f1: Float
			get() = if (precision + recall == 0f) 0f else 2f * precision * recall / (precision + recall)

		val accuracy: Float
			get() {
				val total = truePositives + falsePositives + falseNegatives + trueNegatives
				return if (total == 0) 0f else (truePositives + trueNegatives).toFloat() / total
			}

		/** Mean signed lag: positive means the method is systematically late. */
		val meanLagFrames: Float
			get() = if (lagFrames.isEmpty()) 0f else lagFrames.sum().toFloat() / lagFrames.size

		val meanAbsLagFrames: Float
			get() = if (lagFrames.isEmpty()) 0f else lagFrames.sumOf { abs(it) }.toFloat() / lagFrames.size

		val maxAbsLagFrames: Int
			get() = lagFrames.maxOfOrNull { abs(it) } ?: 0

		fun report(label: String, rateHz: Float): String = buildString {
			append("%-22s".format(label))
			append(" P=%.3f R=%.3f F1=%.3f".format(precision, recall, f1))
			append(" acc=%.3f".format(accuracy))
			append(" lag=%+.2f fr (%+.0f ms)".format(meanLagFrames, meanLagFrames * 1000f / rateHz))
			append(" |lag|max=%d".format(maxAbsLagFrames))
			append(" missed=%d spurious=%d".format(missedTransitions, spuriousTransitions))
		}
	}

	/**
	 * Compare [predicted] against [truth], frame for frame.
	 *
	 * Contact is the positive class, so a false positive is claiming a planted
	 * foot that was actually in the air -- the failure issue #5 describes as
	 * anchoring the body to a moving foot and lurching the world.
	 *
	 * @param matchWindowFrames how far from a true transition a predicted one may
	 *   be and still count as the same event rather than as one missed plus one
	 *   spurious.
	 */
	fun score(
		predicted: BooleanArray,
		truth: BooleanArray,
		matchWindowFrames: Int = 15,
	): Result {
		require(predicted.size == truth.size) {
			"predicted has ${predicted.size} frames, truth has ${truth.size}"
		}

		var tp = 0
		var fp = 0
		var fn = 0
		var tn = 0
		for (i in truth.indices) {
			when {
				predicted[i] && truth[i] -> tp++
				predicted[i] && !truth[i] -> fp++
				!predicted[i] && truth[i] -> fn++
				else -> tn++
			}
		}

		val trueEvents = transitionsOf(truth)
		val predictedEvents = transitionsOf(predicted)

		val lags = mutableListOf<Int>()
		var missed = 0
		val usedPredictions = mutableSetOf<Int>()

		for (event in trueEvents) {
			// Nearest prediction of the same kind. Matching liftoffs only to
			// liftoffs matters: near a touchdown there is often a liftoff a few
			// frames away, and pairing those would report a small lag for two
			// events that have nothing to do with each other.
			val candidate = predictedEvents
				.withIndex()
				.filter { (index, p) ->
					index !in usedPredictions &&
						p.toContact == event.toContact &&
						abs(p.frame - event.frame) <= matchWindowFrames
				}
				.minByOrNull { (_, p) -> abs(p.frame - event.frame) }

			if (candidate == null) {
				missed++
			} else {
				usedPredictions.add(candidate.index)
				lags.add(candidate.value.frame - event.frame)
			}
		}

		return Result(
			truePositives = tp,
			falsePositives = fp,
			falseNegatives = fn,
			trueNegatives = tn,
			lagFrames = lags,
			missedTransitions = missed,
			spuriousTransitions = predictedEvents.size - usedPredictions.size,
		)
	}

	/** @param toContact true if this transition is a touchdown, false a liftoff. */
	data class Transition(val frame: Int, val toContact: Boolean)

	fun transitionsOf(values: BooleanArray): List<Transition> {
		val out = mutableListOf<Transition>()
		for (i in 1 until values.size) {
			if (values[i] != values[i - 1]) out.add(Transition(i, values[i]))
		}
		return out
	}
}
