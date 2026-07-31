package dev.slimevr.poseframeformat.trackerdata

enum class TrackerFrameData(val id: Int) {
	DESIGNATION_STRING(0),
	ROTATION(1),
	POSITION(2),
	TRACKER_POSITION_ENUM(3),
	ACCELERATION(4),
	RAW_ROTATION(5),

	/**
	 * When the sample this frame came from was taken, in the server's timebase.
	 *
	 * Appended as id 6 rather than woven in, so a `.pfr` written before this
	 * existed simply does not set the flag and reads back exactly as it did.
	 */
	SAMPLE_TIMESTAMP(6),
	;

	val flag: Int = 1 shl id

	/*
	 * Inline is fine for these, there's no negative to inlining them as they'll never
	 * change, so any warning about it can be safely ignored
	 */
	inline fun check(dataFlags: Int): Boolean = dataFlags and flag != 0

	inline fun add(dataFlags: Int): Int = dataFlags or flag

	inline fun remove(dataFlags: Int): Int = dataFlags xor flag
}
