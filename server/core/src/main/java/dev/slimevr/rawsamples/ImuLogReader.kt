package dev.slimevr.rawsamples

import java.io.File

/**
 * Reads a `.imu` sidecar back.
 *
 * The counterpart to [ImuLogWriter], and the reason it exists is issue #41: an
 * `.imu` is currently written and never read by this repository, so the raw
 * samples a capture collects cannot be analysed here at all. Everything
 * downstream of a raw capture goes through `tools/fusion-bench`, which answers
 * questions about *fusion* rather than about the samples themselves.
 *
 * Gap markers are parsed rather than skipped. A run of samples separated from
 * the next by a marked hole is not continuous, and anything measuring intervals
 * across one -- which is precisely what contact timing does -- would otherwise
 * read the hole as a very fast event.
 */
class ImuLogReader(
	val sensorName: String,
	val accTs: Float,
	val gyrTs: Float,
	val accScale: Float,
	val gyrScale: Float,
	/** Accelerometer samples in file order, as `(micros, x, y, z)` raw counts. */
	val accel: Samples,
	val gyro: Samples,
	/** Nominal times at which a marked gap begins, one per `# gap` line. */
	val gapMicros: List<Long>,
	val incomplete: Boolean,
) {
	class Samples(
		val micros: LongArray,
		val values: ShortArray,
	) {
		val size: Int get() = micros.size

		fun x(i: Int) = values[i * 3].toInt()
		fun y(i: Int) = values[i * 3 + 1].toInt()
		fun z(i: Int) = values[i * 3 + 2].toInt()

		/** Magnitude in physical units, given the stream's scale factor. */
		fun magnitude(i: Int, scale: Float): Double {
			val dx = x(i).toDouble()
			val dy = y(i).toDouble()
			val dz = z(i).toDouble()
			return Math.sqrt(dx * dx + dy * dy + dz * dz) * scale
		}

		val durationSeconds: Double
			get() = if (size < 2) 0.0 else (micros[size - 1] - micros[0]) / 1e6
	}

	val accelDurationSeconds: Double get() = accel.durationSeconds

	companion object {
		fun read(file: File): ImuLogReader = parse(file.readText())

		fun parse(text: String): ImuLogReader {
			var sensorName = ""
			var accTs = 0f
			var gyrTs = 0f
			var accScale = 0f
			var gyrScale = 0f
			var incomplete = false
			val gaps = mutableListOf<Long>()

			val accMicros = mutableListOf<Long>()
			val accValues = mutableListOf<Short>()
			val gyrMicros = mutableListOf<Long>()
			val gyrValues = mutableListOf<Short>()

			for (raw in text.lineSequence()) {
				val line = raw.trim()
				if (line.isEmpty()) continue

				if (line.startsWith("#")) {
					val body = line.removePrefix("#").trim()
					when {
						body.startsWith("sensor ") -> sensorName = body.removePrefix("sensor ").trim()

						body.startsWith("acc_ts ") -> accTs = body.removePrefix("acc_ts ").trim().toFloat()

						body.startsWith("gyr_ts ") -> gyrTs = body.removePrefix("gyr_ts ").trim().toFloat()

						body.startsWith("acc_scale ") -> accScale = body.removePrefix("acc_scale ").trim().toFloat()

						body.startsWith("gyr_scale ") -> gyrScale = body.removePrefix("gyr_scale ").trim().toFloat()

						body.startsWith("INCOMPLETE") -> incomplete = true

						body.startsWith("gap ") -> {
							// `# gap gyro missing=3 from_us=5000 to_us=20000`
							body.split(" ")
								.firstOrNull { it.startsWith("to_us=") }
								?.removePrefix("to_us=")
								?.toLongOrNull()
								?.let { gaps.add(it) }
						}
					}
					continue
				}
				if (line.startsWith("t_us")) continue

				// `t,ax,ay,az,,,` or `t,,,,gx,gy,gz`
				val cols = line.split(',')
				if (cols.size < 7) continue
				val t = cols[0].toLongOrNull() ?: continue

				if (cols[1].isNotEmpty()) {
					accMicros.add(t)
					accValues.add(cols[1].toShort())
					accValues.add(cols[2].toShort())
					accValues.add(cols[3].toShort())
				} else if (cols[4].isNotEmpty()) {
					gyrMicros.add(t)
					gyrValues.add(cols[4].toShort())
					gyrValues.add(cols[5].toShort())
					gyrValues.add(cols[6].toShort())
				}
			}

			return ImuLogReader(
				sensorName = sensorName,
				accTs = accTs,
				gyrTs = gyrTs,
				accScale = accScale,
				gyrScale = gyrScale,
				accel = Samples(accMicros.toLongArray(), accValues.toShortArray()),
				gyro = Samples(gyrMicros.toLongArray(), gyrValues.toShortArray()),
				gapMicros = gaps,
				incomplete = incomplete,
			)
		}
	}
}
