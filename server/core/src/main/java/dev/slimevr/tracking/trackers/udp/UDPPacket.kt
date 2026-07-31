package dev.slimevr.tracking.trackers.udp

import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import java.io.IOException
import java.nio.BufferUnderflowException
import java.nio.ByteBuffer

sealed class UDPPacket(val packetId: Int) {
	@Throws(IOException::class, BufferUnderflowException::class)
	open fun readData(buf: ByteBuffer) {}

	@Throws(IOException::class)
	open fun writeData(buf: ByteBuffer) {}

	companion object {
		/**
		 * Naively read null-terminated ASCII string from the byte buffer
		 *
		 * @param buf
		 * @return
		 * @throws IOException
		 */
		@Throws(IOException::class)
		fun readASCIIString(buf: ByteBuffer): String {
			val sb = StringBuilder()
			while (true) {
				val c = (buf.get().toInt() and 0xFF).toChar()
				if (c.code == 0) break
				sb.append(c)
			}
			return sb.toString()
		}

		@JvmStatic
		@Throws(IOException::class)
		fun readASCIIString(buf: ByteBuffer, length: Int): String {
			var length = length
			val sb = StringBuilder()
			while (length-- > 0) {
				val c = (buf.get().toInt() and 0xFF).toChar()
				if (c.code == 0) break
				sb.append(c)
			}
			return sb.toString()
		}

		/**
		 * Naively write null-terminated ASCII string to byte buffer
		 *
		 * @param str
		 * @param buf
		 * @throws IOException
		 */
		@Throws(IOException::class)
		fun writeASCIIString(str: String, buf: ByteBuffer) {
			for (element in str) {
				buf.put((element.code and 0xFF).toByte())
			}
			buf.put(0.toByte())
		}
	}
}

sealed interface SensorSpecificPacket {
	val sensorId: Int
	companion object {
		/**
		 * Sensor with id 255 is "global" representing a whole device
		 *
		 * @param sensorId
		 * @return
		 */
		fun isGlobal(sensorId: Int): Boolean = sensorId == 255
	}
}

sealed interface RotationPacket : SensorSpecificPacket {
	var rotation: Quaternion
}

data object UDPPacket0Heartbeat : UDPPacket(0)
data object UDPPacket1Heartbeat : UDPPacket(1)
data class UDPPacket1Rotation(override var rotation: Quaternion = Quaternion.IDENTITY) :
	UDPPacket(1),
	RotationPacket {
	override val sensorId = 0
	override fun readData(buf: ByteBuffer) {
		rotation = UDPUtils.getSafeBufferQuaternion(buf)
	}
}

data class UDPPacket3Handshake(
	var boardType: BoardType = BoardType.UNKNOWN,
	var imuType: IMUType = IMUType.UNKNOWN,
	var mcuType: MCUType = MCUType.UNKNOWN,
	var protocolVersion: Int = 0,
	var firmware: String? = null,
	var macString: String? = null,
) : UDPPacket(3) {
	override fun readData(buf: ByteBuffer) {
		if (buf.remaining() == 0) return
		if (buf.remaining() > 3) {
			boardType = BoardType.getById(buf.int.toUInt()) ?: BoardType.UNKNOWN
		}
		if (buf.remaining() > 3) {
			imuType = IMUType.getById(buf.int.toUInt()) ?: IMUType.UNKNOWN
		}
		if (buf.remaining() > 3) {
			mcuType = MCUType.getById(buf.int.toUInt()) ?: MCUType.UNKNOWN
		} // MCU TYPE
		if (buf.remaining() > 11) {
			buf.int
			buf.int
			buf.int // IMU info
		}
		if (buf.remaining() > 3) protocolVersion = buf.int
		var length = 0
		if (buf.remaining() > 0) length = buf.get().toInt()
		// firmware version length is 1 longer than
		// that because it's nul-terminated
		firmware = readASCIIString(buf, length)
		val mac = ByteArray(6)
		if (buf.remaining() >= mac.size) {
			buf.get(mac)
			macString = String.format(
				"%02X:%02X:%02X:%02X:%02X:%02X",
				mac[0],
				mac[1],
				mac[2],
				mac[3],
				mac[4],
				mac[5],
			)
			if (macString == "00:00:00:00:00:00") macString = null
		}
	}

	override fun writeData(buf: ByteBuffer) {
		// Never sent back in current protocol
		// Handshake for RAW SlimeVR and legacy owoTrack has different packet id
		// byte
		// order from normal packets
		// So it's handled by raw protocol call
	}
}

data class UDPPacket4Acceleration(var acceleration: Vector3 = Vector3.NULL) :
	UDPPacket(4),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		acceleration = Vector3(UDPUtils.getSafeBufferFloat(buf), UDPUtils.getSafeBufferFloat(buf), UDPUtils.getSafeBufferFloat(buf))

		sensorId = try {
			buf.get().toInt() and 0xFF
		} catch (e: BufferUnderflowException) {
			// for owo track app
			0
		}
	}
}

data class UDPPacket10PingPong(var pingId: Int = 0) : UDPPacket(10) {
	override fun readData(buf: ByteBuffer) {
		pingId = buf.int
	}

	override fun writeData(buf: ByteBuffer) {
		buf.putInt(pingId)
	}
}

data class UDPPacket11Serial(var serial: String = "") : UDPPacket(11) {
	override fun readData(buf: ByteBuffer) {
		val length = buf.int
		val sb = StringBuilder(length)
		for (i in 0 until length) {
			val ch = Char(buf.get().toUShort())
			sb.append(ch)
		}
		serial = sb.toString()
	}
}

data class UDPPacket12BatteryLevel(
	var voltage: Float? = null,
	var level: Float = 0.0f,
) : UDPPacket(12) {

	override fun readData(buf: ByteBuffer) {
		if (buf.remaining() >= 8) {
			voltage = UDPUtils.getSafeBufferFloat(buf)
			level = UDPUtils.getSafeBufferFloat(buf)
		} else {
			level = UDPUtils.getSafeBufferFloat(buf)
		}
	}
}

data class UDPPacket13Tap(var tap: SensorTap = SensorTap(0)) :
	UDPPacket(13),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		tap = SensorTap(buf.get().toInt() and 0xFF)
	}
}

data class UDPPacket14Error(var errorNumber: Int = 0) :
	UDPPacket(14),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		errorNumber = buf.get().toInt() and 0xFF
	}
}

data class UDPPacket15SensorInfo(
	var sensorStatus: Int = 0,
	var sensorType: IMUType = IMUType.UNKNOWN,
	var sensorConfig: SensorConfig? = null,
	var hasCompletedRestCalibration: Boolean? = null,
	var trackerPosition: TrackerPosition? = null,
	var trackerDataType: TrackerDataType = TrackerDataType.ROTATION,
) : UDPPacket(15),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		sensorStatus = buf.get().toInt() and 0xFF
		if (buf.remaining() > 0) {
			sensorType = IMUType.getById(buf.get().toUInt() and 0xFFu) ?: IMUType.UNKNOWN
		}
		if (buf.remaining() > 1) {
			sensorConfig = SensorConfig(buf.getShort().toUShort())
		}
		if (buf.remaining() > 0) hasCompletedRestCalibration = buf.get().toInt() and 0xFF != 0
		if (buf.remaining() > 0) trackerPosition = TrackerPosition.getById(buf.get().toInt() and 0xFF)
		if (buf.remaining() > 0) trackerDataType = TrackerDataType.getById(buf.get().toUInt() and 0xFFu) ?: TrackerDataType.ROTATION
	}

	companion object {
		fun getStatus(sensorStatus: Int): TrackerStatus? = when (sensorStatus) {
			0 -> TrackerStatus.DISCONNECTED
			1 -> TrackerStatus.OK
			2 -> TrackerStatus.ERROR
			else -> null
		}
	}
}

data class UDPPacket16Rotation2(override var rotation: Quaternion = Quaternion.IDENTITY) :
	UDPPacket(16),
	RotationPacket {
	override val sensorId = 1
	override fun readData(buf: ByteBuffer) {
		rotation = UDPUtils.getSafeBufferQuaternion(buf)
	}
}

data class UDPPacket17RotationData(
	var rotation: Quaternion = Quaternion.IDENTITY,
	var dataType: Int = 0,
	var calibrationInfo: Int = 0,
) : UDPPacket(17),
	SensorSpecificPacket {
	override var sensorId: Int = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		dataType = buf.get().toInt() and 0xFF
		rotation = UDPUtils.getSafeBufferQuaternion(buf)
		calibrationInfo = buf.get().toInt() and 0xFF
	}

	companion object {
		const val DATA_TYPE_NORMAL = 1
		const val DATA_TYPE_CORRECTION = 2
	}
}

data class UDPPacket18MagnetometerAccuracy(var accuracyInfo: Float = 0.0f) :
	UDPPacket(18),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		accuracyInfo = UDPUtils.getSafeBufferFloat(buf)
	}
}

data class UDPPacket19SignalStrength(var signalStrength: Int = 0) :
	UDPPacket(19),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		signalStrength = buf.get().toInt()
	}
}

data class UDPPacket20Temperature(var temperature: Float = 0.0f) :
	UDPPacket(20),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		temperature = UDPUtils.getSafeBufferFloat(buf)
	}
}

data class UDPPacket21UserAction(var type: Int = 0) : UDPPacket(21) {
	override fun readData(buf: ByteBuffer) {
		type = buf.get().toInt() and 0xFF
	}

	companion object {
		const val RESET_FULL = 2
		const val RESET_YAW = 3
		const val RESET_MOUNTING = 4
		const val PAUSE_TRACKING = 5
	}
}

class UDPPacket22FeatureFlags(
	var firmwareFeatures: FirmwareFeatures = FirmwareFeatures(),
) : UDPPacket(22) {
	override fun readData(buf: ByteBuffer) {
		firmwareFeatures = FirmwareFeatures.from(buf, buf.remaining())
	}

	override fun writeData(buf: ByteBuffer) {
		buf.put(ServerFeatureFlags.packed)
	}
}

data class UDPPacket23RotationAndAcceleration(
	override var rotation: Quaternion = Quaternion.IDENTITY,
	var acceleration: Vector3 = Vector3.NULL,
) : UDPPacket(23),
	RotationPacket {
	override var sensorId: Int = 0
	override fun readData(buf: ByteBuffer) {
		// s16 s16 s16 s16 s16 s16 s16
		// qX  qY  qZ  qW  aX  aY  aZ
		sensorId = buf.get().toInt() and 0xFF
		val scaleR = 1 / (1 shl 15).toFloat() // Q15: 1 is represented as 0x7FFF and -1 as 0x8000
		val x = buf.short * scaleR
		val y = buf.short * scaleR
		val z = buf.short * scaleR
		val w = buf.short * scaleR
		rotation = Quaternion(w, x, y, z).unit()
		val scaleA = 1 / (1 shl 7).toFloat() // The same as the HID scale
		acceleration = Vector3(buf.short * scaleA, buf.short * scaleA, buf.short * scaleA)
	}
}

data class UDPPacket24AckConfigChange(
	override var sensorId: Int = 0,
	var configType: ConfigTypeId = ConfigTypeId(0u),
) : UDPPacket(24),
	SensorSpecificPacket {
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		configType = ConfigTypeId(buf.getShort().toUShort())
	}
}

data class UDPPacket25SetConfigFlag(
	override var sensorId: Int = 255,
	var configType: ConfigTypeId,
	var state: Boolean,
) : UDPPacket(25),
	SensorSpecificPacket {
	override fun writeData(buf: ByteBuffer) {
		buf.put(sensorId.toByte())
		buf.putShort(configType.v.toShort())
		buf.put(if (state) 1 else 0)
	}
}

data class UDPPacket26FlexData(
	var flexData: Float = 0f,
) : UDPPacket(26),
	SensorSpecificPacket {

	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		flexData = UDPUtils.getSafeBufferFloat(buf)
	}
}

data class UDPPacket27Position(
	var position: Vector3 = Vector3.NULL,
) : UDPPacket(27),
	SensorSpecificPacket {
	override var sensorId = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		val x = UDPUtils.getSafeBufferFloat(buf)
		val y = UDPUtils.getSafeBufferFloat(buf)
		val z = UDPUtils.getSafeBufferFloat(buf)
		position = Vector3(x, y, z)
	}
}

/**
 * Round-trip clock synchronisation, the four-timestamp NTP exchange.
 *
 * The server writes its transmit time and sends; the tracker fills in the two
 * timestamps from its own microsecond clock and echoes it back. The server then
 * has all four times and can estimate the offset and rate of that tracker's
 * clock -- see [ClockSync].
 *
 * Tracker timestamps are 32-bit because that is what `micros()` returns on an
 * ESP; they wrap every ~71.6 minutes and must be unwrapped before use.
 */
data class UDPPacket28TimeSync(
	/** Server transmit time, microseconds, echoed back unmodified. */
	var serverTxMicros: Long = 0,
	/** Tracker clock when it received the request. */
	var trackerRxMicros: Long = 0,
	/** Tracker clock when it sent the reply. */
	var trackerTxMicros: Long = 0,
) : UDPPacket(28) {
	override fun readData(buf: ByteBuffer) {
		serverTxMicros = buf.long
		trackerRxMicros = buf.int.toLong() and 0xFFFFFFFFL
		trackerTxMicros = buf.int.toLong() and 0xFFFFFFFFL
	}

	override fun writeData(buf: ByteBuffer) {
		buf.putLong(serverTxMicros)
		// Zeroed on the outbound leg; the tracker fills these in.
		buf.putInt(0)
		buf.putInt(0)
	}
}

/**
 * Rotation carrying the tracker's own measurement time.
 *
 * Identical to [UDPPacket17RotationData] with a trailing 32-bit timestamp, so
 * the sample can be converted into the server's timebase instead of being
 * assumed to have happened the instant it arrived.
 *
 * Sent only to servers that advertise [ServerFeatureFlags.PROTOCOL_SAMPLE_TIMESTAMPS],
 * so older servers never see it.
 */
data class UDPPacket29RotationDataTimestamped(
	var rotation: Quaternion = Quaternion.IDENTITY,
	var dataType: Int = 0,
	var calibrationInfo: Int = 0,
	/** Raw tracker `micros()`; wraps every ~71.6 minutes and must be unwrapped. */
	var timestampMicros: Long = 0,
) : UDPPacket(29),
	SensorSpecificPacket {
	override var sensorId: Int = 0
	override fun readData(buf: ByteBuffer) {
		sensorId = buf.get().toInt() and 0xFF
		dataType = buf.get().toInt() and 0xFF
		rotation = UDPUtils.getSafeBufferQuaternion(buf)
		calibrationInfo = buf.get().toInt() and 0xFF
		timestampMicros = buf.int.toLong() and 0xFFFFFFFFL
	}
}

/**
 * A batch of raw, pre-calibration IMU samples, or the metadata needed to scale
 * them.
 *
 * The tracker sends these only to servers advertising
 * [ServerFeatureFlags.PROTOCOL_RAW_SAMPLES], so nothing changes for a server
 * that does not want them. See kmatzen/SlimeVR-Tracker-ESP#23.
 *
 * Two shapes behind one packet id, discriminated by [batchType], because they
 * belong to one stream and are useless apart -- the same reasoning as the
 * firmware's existing `InspectionPacketType`.
 *
 * Sample times are *nominal*: accumulated from the configured sample period
 * rather than read from a clock, because the configured period is what the
 * on-device fusion integrates. Replaying them reproduces what the filter saw,
 * and the regularity is what makes a missing-sample count exact rather than
 * inferred. [realMicros] carries the tracker's true clock at flush so the gap
 * between configured and actual output rate stays measurable.
 */
data class UDPPacket30RawSampleBatch(
	var batchType: Int = 0,
	var sensorName: String = "",
	var accTs: Float = 0f,
	var gyrTs: Float = 0f,
	var accScale: Float = 0f,
	var gyrScale: Float = 0f,
	var kind: Int = 0,
	/** Counts batches *produced*, so a gap means data was lost in transit. */
	var sequence: Long = 0,
	/** Cumulative samples the tracker discarded to buffer overrun. */
	var dropped: Long = 0,
	/**
	 * Cumulative samples the sensor's hardware FIFO discarded before the
	 * firmware saw them.
	 *
	 * Separate from [dropped] because the causes are: the network path could not
	 * keep up, versus the sensor drain loop could not. It is also the only hole
	 * nothing else can see -- the batch sequence stays unbroken across one, and
	 * the nominal clock closes over it.
	 */
	var fifoDropped: Long = 0,
	var baseNominalMicros: Long = 0,
	var realMicros: Long = 0,
	var sampleCount: Int = 0,
	var samples: ShortArray = ShortArray(0),
) : UDPPacket(30),
	SensorSpecificPacket {
	override var sensorId: Int = 0

	val isStreamInfo: Boolean get() = batchType == TYPE_STREAM_INFO
	val isSamples: Boolean get() = batchType == TYPE_SAMPLES

	override fun readData(buf: ByteBuffer) {
		batchType = buf.get().toInt() and 0xFF
		sensorId = buf.get().toInt() and 0xFF
		when (batchType) {
			TYPE_STREAM_INFO -> {
				accTs = buf.float
				gyrTs = buf.float
				accScale = buf.float
				gyrScale = buf.float
				val nameLength = buf.get().toInt() and 0xFF
				val bytes = ByteArray(nameLength.coerceAtMost(buf.remaining()))
				buf.get(bytes)
				sensorName = String(bytes, Charsets.UTF_8)
			}

			TYPE_SAMPLES -> {
				kind = buf.get().toInt() and 0xFF
				sequence = buf.int.toLong() and 0xFFFFFFFFL
				dropped = buf.int.toLong() and 0xFFFFFFFFL
				fifoDropped = buf.int.toLong() and 0xFFFFFFFFL
				baseNominalMicros = buf.long
				realMicros = buf.int.toLong() and 0xFFFFFFFFL
				val declared = buf.short.toInt() and 0xFFFF
				// Trust the buffer over the declared count. A truncated datagram
				// would otherwise throw here, and one malformed packet must not
				// take down the receive loop mid-capture.
				sampleCount = minOf(declared, buf.remaining() / 6)
				samples = ShortArray(sampleCount * 3)
				for (i in samples.indices) {
					samples[i] = buf.short
				}
			}
		}
	}

	// Generated equals/hashCode would compare the ShortArray by identity, which
	// is the wrong answer for a data class carrying one.
	override fun equals(other: Any?): Boolean = this === other

	override fun hashCode(): Int = System.identityHashCode(this)

	companion object {
		const val TYPE_STREAM_INFO = 0
		const val TYPE_SAMPLES = 1
	}
}

data class UDPPacket200ProtocolChange(
	var targetProtocol: Int = 0,
	var targetProtocolVersion: Int = 0,
) : UDPPacket(200) {
	override fun readData(buf: ByteBuffer) {
		targetProtocol = buf.get().toInt() and 0xFF
		targetProtocolVersion = buf.get().toInt() and 0xFF
	}

	override fun writeData(buf: ByteBuffer) {
		buf.put(targetProtocol.toByte())
		buf.put(targetProtocolVersion.toByte())
	}
}

class UDPUtils {
	companion object {
		fun getSafeBufferQuaternion(byteBuffer: ByteBuffer): Quaternion {
			val x = byteBuffer.getFloat()
			val y = byteBuffer.getFloat()
			val z = byteBuffer.getFloat()
			val w = byteBuffer.getFloat()

			return if (
				(x.isNaN() || y.isNaN() || z.isNaN() || w.isNaN()) ||
				(x == 0f && y == 0f && z == 0f && w == 0f)
			) {
				Quaternion.IDENTITY
			} else {
				Quaternion(w, x, y, z)
			}
		}
		fun getSafeBufferFloat(byteBuffer: ByteBuffer): Float {
			val value = byteBuffer.getFloat()
			return if (value.isNaN()) {
				0f
			} else {
				value
			}
		}
	}
}
