package dev.slimevr.rawsamples

import java.util.concurrent.ConcurrentHashMap

/**
 * Collects raw sample batches from every tracker for the duration of a capture.
 *
 * Off unless a capture is running. Raw streaming is started and stopped by a
 * command to the trackers, so nothing arrives when nothing asked for it -- but
 * this gates on [isCapturing] anyway, because a tracker that missed a stop
 * command should not quietly grow the server's heap.
 *
 * Keyed by `(deviceId, sensorId)`. One physical tracker can carry more than one
 * IMU, and their streams have separate nominal timelines and separate loss
 * accounting, so merging them would make both uninterpretable.
 */
class RawSampleCollector {
	data class Key(val deviceId: Int, val sensorId: Int)

	private val captures = ConcurrentHashMap<Key, RawSampleCapture>()

	@Volatile
	var isCapturing: Boolean = false
		private set

	/** Batches that arrived before any `StreamInfo` described how to scale them. */
	@Volatile
	var unscalableBatches: Long = 0
		private set

	fun start() {
		captures.clear()
		unscalableBatches = 0
		isCapturing = true
	}

	fun stop() {
		isCapturing = false
	}

	/**
	 * Records the stream's metadata.
	 *
	 * Repeated by the tracker every couple of seconds. The first one wins:
	 * replacing the capture on every repeat would throw away everything
	 * collected so far, and the values do not change within a session.
	 */
	fun streamInfo(
		deviceId: Int,
		sensorId: Int,
		sensorName: String,
		accTs: Float,
		gyrTs: Float,
		accScale: Float,
		gyrScale: Float,
	) {
		if (!isCapturing) return
		captures.computeIfAbsent(Key(deviceId, sensorId)) {
			RawSampleCapture(sensorName, accTs, gyrTs, accScale, gyrScale)
		}
	}

	/**
	 * Records one batch of samples.
	 *
	 * A batch that arrives before its `StreamInfo` is counted and discarded
	 * rather than kept with guessed scale factors. Raw counts with the wrong
	 * scale are not a degraded capture, they are a wrong one, and the tracker
	 * repeats the info often enough that the loss is bounded by a couple of
	 * seconds at the very start.
	 */
	fun samples(
		deviceId: Int,
		sensorId: Int,
		kind: RawSampleKind,
		sequence: Long,
		droppedTotal: Int,
		baseMicros: Long,
		values: ShortArray,
		count: Int,
	) {
		if (!isCapturing) return
		val capture = captures[Key(deviceId, sensorId)]
		if (capture == null) {
			unscalableBatches++
			return
		}
		capture.stream(kind).accept(sequence, droppedTotal, baseMicros, values, count)
	}

	/** Every sensor that produced samples, in a stable order. */
	fun results(): Map<Key, RawSampleCapture> = captures.entries
		.sortedWith(compareBy({ it.key.deviceId }, { it.key.sensorId }))
		.associate { it.key to it.value }

	val sampleCount: Int get() = captures.values.sumOf { it.sampleCount }

	val isComplete: Boolean get() = unscalableBatches == 0L && captures.values.all { it.isComplete }
}
