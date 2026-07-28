package dev.slimevr.replay

/**
 * Committed expected values for [SkeletonReplayTest].
 *
 * Plain text rather than JSON on purpose: the file is meant to be read in a
 * pull request, and `key value tolerance` on one line per metric shows a
 * reviewer exactly what moved and by how much. A baseline nobody reads is a
 * baseline nobody maintains.
 *
 * To regenerate, run the test with `-Dreplay.writeBaseline=true` and copy the
 * emitted block over `replay-baseline.txt`. Read the diff before committing it
 * -- regenerating a baseline is how a real regression gets blessed as expected
 * behaviour.
 */
object ReplayBaseline {
	private const val RESOURCE = "/replay-baseline.txt"

	data class Entry(val value: Float, val tolerance: Float)

	fun load(): Map<String, Entry> {
		val stream = ReplayBaseline::class.java.getResourceAsStream(RESOURCE)
			?: return emptyMap()

		return stream.bufferedReader().useLines { lines ->
			lines.mapNotNull { raw ->
				val line = raw.substringBefore('#').trim()
				if (line.isEmpty()) return@mapNotNull null
				val parts = line.split(Regex("\\s+"))
				if (parts.size < 3) return@mapNotNull null
				val value = parts[1].toFloatOrNull() ?: return@mapNotNull null
				val tolerance = parts[2].toFloatOrNull() ?: return@mapNotNull null
				parts[0] to Entry(value, tolerance)
			}.toMap()
		}
	}

	/** Formats measured values as a baseline file body. */
	fun format(values: Map<String, Float>): String {
		val sb = StringBuilder()
		sb.append("# skeleton replay baseline\n")
		sb.append("# key  value  tolerance\n")
		sb.append("#\n")
		sb.append("# Tolerances should come from measured run-to-run spread.\n")
		sb.append("# Replay is deterministic for these metrics, so the spread is\n")
		sb.append("# zero and a tight absolute tolerance is legitimate.\n")
		for ((key, value) in values) {
			sb.append("%-44s %12.6f %12.6f\n".format(key, value, defaultTolerance(value)))
		}
		return sb.toString()
	}

	/**
	 * Absolute floor rather than a pure percentage: several of these metrics
	 * are legitimately zero, and a percentage tolerance of zero is zero, which
	 * would make the check infinitely strict.
	 */
	private fun defaultTolerance(value: Float): Float = maxOf(kotlin.math.abs(value) * 0.02f, 1e-4f)
}
