package dev.slimevr.rawsamples

import java.io.File

/**
 * One sensor's raw capture: both streams, plus the scale factors that turn
 * counts into physical units.
 *
 * The scale factors arrive in a `StreamInfo` batch rather than being assumed,
 * and the tracker repeats that batch every couple of seconds -- on UDP, a
 * server that started capturing late, or simply lost the first datagram, would
 * otherwise hold counts it cannot convert into anything.
 */
class RawSampleCapture(
	val sensorName: String,
	val accTs: Float,
	val gyrTs: Float,
	val accScale: Float,
	val gyrScale: Float,
) {
	val accel = RawSampleStream(RawSampleKind.ACCEL, micros(accTs))
	val gyro = RawSampleStream(RawSampleKind.GYRO, micros(gyrTs))

	fun stream(kind: RawSampleKind) = when (kind) {
		RawSampleKind.ACCEL -> accel
		RawSampleKind.GYRO -> gyro
	}

	val sampleCount: Int get() = accel.sampleCount + gyro.sampleCount
	val isComplete: Boolean get() = accel.isComplete && gyro.isComplete

	companion object {
		private fun micros(seconds: Float): Long = Math.round(seconds.toDouble() * 1_000_000.0)
	}
}

/**
 * Writes a capture as `# slimevr-imu-log v1`.
 *
 * ## Why this exact format
 *
 * `tools/fusion-bench` in the firmware repository already reads and writes it --
 * `dataset.cpp` parses `t_us,ax,ay,az,gx,gy,gz` with `acc_scale`/`gyr_scale`
 * header keys. Emitting it here means a wirelessly captured seven-tracker log
 * replays through the existing bench with no new tooling, which closes the
 * remaining half of issue #1 as a side effect rather than as a project.
 *
 * The row shape is the firmware's: accelerometer rows fill the `a*` columns and
 * leave the `g*` columns empty, gyroscope rows the reverse. Both streams are
 * merged into one file in nominal-time order, because that is the order the
 * fusion filter consumed them in and reproducing it is the point.
 *
 * ## Gap markers
 *
 * Written as `#` comments, which the bench's parser skips, so a holed recording
 * is still loadable -- it simply reads as two shorter captures with a documented
 * hole between them rather than as one continuous capture that silently jumps.
 *
 * That distinction is the reason this class exists rather than a `joinToString`.
 */
object ImuLogWriter {
	const val HEADER = "# slimevr-imu-log v1"

	fun write(file: File, capture: RawSampleCapture) {
		file.writeText(render(capture))
	}

	fun render(capture: RawSampleCapture): String {
		val out = StringBuilder()
		out.append(HEADER).append('\n')
		out.append("# sensor ").append(capture.sensorName).append('\n')
		out.append("# acc_ts ").append(g9(capture.accTs)).append('\n')
		out.append("# gyr_ts ").append(g9(capture.gyrTs)).append('\n')
		out.append("# acc_scale ").append(g9(capture.accScale)).append('\n')
		out.append("# gyr_scale ").append(g9(capture.gyrScale)).append('\n')
		out.append("# note raw uncalibrated counts; apply *_scale to convert\n")

		// Stated up front rather than only implied by the markers below, so
		// whoever opens the file knows before reading a single row whether it
		// is whole. A capture with holes is usable; one whose holes are a
		// surprise is not.
		if (!capture.isComplete) {
			out.append("# INCOMPLETE -- see gap markers below\n")
		}
		out.append("# ").append(capture.accel.summary()).append('\n')
		out.append("# ").append(capture.gyro.summary()).append('\n')
		out.append("t_us,ax,ay,az,gx,gy,gz\n")

		for (row in rows(capture)) {
			out.append(row).append('\n')
		}
		return out.toString()
	}

	/**
	 * Both streams merged in nominal-time order, with a gap marker wherever a
	 * run ends and the next one does not continue it.
	 */
	private fun rows(capture: RawSampleCapture): List<String> {
		class Entry(val micros: Long, val text: String, val order: Int)

		val entries = mutableListOf<Entry>()

		fun emit(stream: RawSampleStream, accel: Boolean) {
			var previousEnd: Long? = null
			for (run in stream.runs) {
				val end = previousEnd
				if (end != null && run.startMicros > end) {
					val missing = if (stream.stepMicros > 0) {
						(run.startMicros - end) / stream.stepMicros
					} else {
						0
					}
					// Ordered before the row at the same instant, so the marker
					// reads as "the hole ends here" rather than trailing the
					// first sample after it.
					entries.add(
						Entry(
							run.startMicros,
							"# gap ${stream.kind.name.lowercase()} missing=$missing " +
								"from_us=$end to_us=${run.startMicros}",
							-1,
						),
					)
				}
				for (i in 0 until run.count) {
					val t = run.sampleMicros(i, stream.stepMicros)
					val x = run.samples[i * 3]
					val y = run.samples[i * 3 + 1]
					val z = run.samples[i * 3 + 2]
					val text = if (accel) "$t,$x,$y,$z,,," else "$t,,,,$x,$y,$z"
					entries.add(Entry(t, text, if (accel) 0 else 1))
				}
				previousEnd = run.endMicros(stream.stepMicros)
			}
		}

		emit(capture.accel, accel = true)
		emit(capture.gyro, accel = false)

		// Stable in time, then markers, then accelerometer before gyroscope at
		// the same instant. Any total order would parse; a deterministic one
		// means two captures of the same data produce the same file.
		entries.sortWith(compareBy({ it.micros }, { it.order }))
		return entries.map { it.text }
	}

	/**
	 * The shortest text that reads back as the same `Float`.
	 *
	 * The firmware writes `%.9g` of the value widened to `double`, for the
	 * stated reason that the round trip through text must not perturb a scale
	 * factor. This serves that intent better rather than differently: widening
	 * a float to double first exposes its binary representation, so 0.01f prints
	 * as `0.00999999978` -- correct, and unreadable in a file someone has to
	 * interpret two years from now. `Float.toString` is defined to emit the
	 * shortest decimal that round-trips, which is exactly the guarantee wanted.
	 *
	 * The header is parsed by key with the value read as a number, so both forms
	 * load identically in `tools/fusion-bench`.
	 */
	private fun g9(value: Float): String = value.toString()
}
