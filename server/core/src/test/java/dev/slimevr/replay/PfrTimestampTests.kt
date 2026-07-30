package dev.slimevr.replay

import dev.slimevr.poseframeformat.PfrIO
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.poseframeformat.player.PlayerTracker
import dev.slimevr.poseframeformat.player.TrackerFramesPlayer
import dev.slimevr.poseframeformat.trackerdata.TrackerFrame
import dev.slimevr.poseframeformat.trackerdata.TrackerFrameData
import dev.slimevr.poseframeformat.trackerdata.TrackerFrames
import dev.slimevr.tracking.trackers.TimeAlignment
import dev.slimevr.tracking.trackers.TrackerPosition
import dev.slimevr.tracking.trackers.TrackerStatus
import io.eiren.util.collections.FastList
import io.github.axisangles.ktmath.Quaternion
import org.junit.jupiter.api.Test
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.io.DataInputStream
import java.io.DataOutputStream
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * The `.pfr` sample-timestamp field, and what it exists to make possible.
 *
 * ## Why the field exists
 *
 * `.pfr` stored rotation, position, acceleration and raw rotation, and no time
 * at all. [dev.slimevr.poseframeformat.PoseRecorder] samples tracker *state* on
 * a uniform tick, so the per-tracker sample times -- the quantity
 * [TimeAlignment] exists to remove -- were discarded at capture.
 *
 * The consequence was a third silent no-op of the kind #29 found two of:
 * [PlayerTracker] called the untimestamped setter, so no replayed tracker had
 * any sample history, `isEligible` was false for every one of them, and
 * `align()` returned having touched nothing. The replay completed, every metric
 * was produced, and time alignment had never run -- on any recording, ever.
 *
 * Unlike `imu_type` and `stay_aligned.*`, this one could not have been fixed by
 * a sidecar field afterwards. The data was never written down. That is what
 * makes it worth doing before the corpus exists rather than after.
 */
class PfrTimestampTests {

	private val rotation = Quaternion(1f, 0f, 0f, 0f)

	private fun frame(micros: Long?) = TrackerFrame(
		trackerPosition = TrackerPosition.LEFT_LOWER_LEG,
		rotation = rotation,
		sampleServerMicros = micros,
	)

	private fun roundTrip(frame: TrackerFrame): TrackerFrame {
		val bytes = ByteArrayOutputStream()
		DataOutputStream(bytes).use { PfrIO.writeFrame(it, frame) }
		return DataInputStream(ByteArrayInputStream(bytes.toByteArray())).use { PfrIO.readFrame(it) }
	}

	@Test
	fun aTimestampSurvivesTheRoundTrip() {
		val read = roundTrip(frame(1_234_567_890_123L))

		assertEquals(1_234_567_890_123L, read.sampleServerMicros)
		assertEquals(1_234_567_890_123L, read.tryGetSampleServerMicros())
		assertTrue(read.hasData(TrackerFrameData.SAMPLE_TIMESTAMP))
	}

	/**
	 * A recording from firmware that reports no sample times must be
	 * indistinguishable from one written before the field existed -- no flag, no
	 * bytes, and a reader that returns null rather than zero.
	 *
	 * Zero would be worse than null: it is `TimeAlignment`'s sentinel for "never
	 * timestamped", so writing it for every tracker would claim they were all
	 * sampled at the same instant, which is the fiction the field exists to
	 * remove.
	 */
	@Test
	fun anUntimestampedFrameWritesNothing() {
		val withTime = roundTrip(frame(1_000L))
		val without = roundTrip(frame(null))

		assertNull(without.sampleServerMicros)
		assertNull(without.tryGetSampleServerMicros())
		assertTrue(!without.hasData(TrackerFrameData.SAMPLE_TIMESTAMP))
		assertNotNull(withTime.sampleServerMicros)
	}

	/**
	 * The compatibility guarantee, stated as bytes rather than as intent.
	 *
	 * A `.pfr` written before this change sets no bit 6, so every existing
	 * recording -- including anything AutoBone saved -- reads back exactly as it
	 * did. Appending the field last, matching its flag id, is what makes that
	 * true: fields are read in flag order, so nothing before it shifts.
	 */
	@Test
	fun anOldRecordingIsUnaffected() {
		val old = TrackerFrame(
			trackerPosition = TrackerPosition.CHEST,
			rotation = rotation,
		)
		val bytes = ByteArrayOutputStream()
		DataOutputStream(bytes).use { PfrIO.writeFrame(it, old) }
		val written = bytes.toByteArray()

		// Byte-for-byte what the previous format produced: the flags word, then
		// the rotation and the position enum. No timestamp bit, no extra bytes.
		val flags = DataInputStream(ByteArrayInputStream(written)).readInt()
		assertTrue(!TrackerFrameData.SAMPLE_TIMESTAMP.check(flags))
		assertEquals(4 + 16 + 4, written.size)

		val read = roundTrip(old)
		assertNull(read.sampleServerMicros)
		assertEquals(rotation, read.rotation)
	}

	private fun recording(leftMicros: List<Long?>, rightMicros: List<Long?>): PoseFrames {
		fun holder(name: String, position: TrackerPosition, times: List<Long?>) = TrackerFrames(
			name,
			FastList<TrackerFrame?>().apply {
				times.forEach { add(TrackerFrame(position, rotation, sampleServerMicros = it)) }
			},
		)

		return PoseFrames(
			FastList<TrackerFrames>().apply {
				add(holder("left", TrackerPosition.LEFT_LOWER_LEG, leftMicros))
				add(holder("right", TrackerPosition.RIGHT_LOWER_LEG, rightMicros))
			},
		)
	}

	/**
	 * The whole point: a replayed recording must produce trackers that
	 * [TimeAlignment] will actually act on.
	 *
	 * Two trackers, sampled 4 ms apart on the capture machine's clock. Before
	 * this change `participants` was 0 on every recording; the assertion is that
	 * it is now 2, and that the spread the alignment sees is the spread that was
	 * captured.
	 */
	@Test
	fun aReplayedRecordingTakesPartInTimeAlignment() {
		val base = 9_000_000_000_000L
		val player = TrackerFramesPlayer(
			recording(
				leftMicros = List(5) { base + it * 10_000L },
				rightMicros = List(5) { base + it * 10_000L + 4_000L },
			),
		)
		player.trackers.forEach { it.status = TrackerStatus.OK }

		for (i in 0 until 5) player.setCursors(i)

		val alignment = TimeAlignment()
		alignment.align(player.trackers.toList())

		println("participants=${alignment.participants} spread=${alignment.spreadMicros} us")

		assertEquals(2, alignment.participants, "replayed trackers are not eligible for time alignment")
		assertEquals(4_000L, alignment.spreadMicros, "the captured 4 ms skew did not survive replay")
	}

	/**
	 * Rebasing must remove the capture machine's epoch and nothing else.
	 *
	 * The same recording shifted by an arbitrary constant has to replay
	 * identically, or a corpus file would mean something different depending on
	 * when it was captured.
	 */
	@Test
	fun replayIsIndependentOfTheCaptureEpoch() {
		fun spreadFor(base: Long): Long {
			val player = TrackerFramesPlayer(
				recording(
					leftMicros = List(4) { base + it * 10_000L },
					rightMicros = List(4) { base + it * 10_000L + 2_500L },
				),
			)
			player.trackers.forEach { it.status = TrackerStatus.OK }
			for (i in 0 until 4) player.setCursors(i)

			val alignment = TimeAlignment()
			alignment.align(player.trackers.toList())
			return alignment.spreadMicros
		}

		assertEquals(spreadFor(0L), spreadFor(1_700_000_000_000_000L))
		assertEquals(2_500L, spreadFor(42L))
	}

	/**
	 * The earliest sample must not rebase onto zero.
	 *
	 * Zero is `TimeAlignment.isEligible`'s sentinel for "this tracker has never
	 * reported a timestamp", so a recording whose first frame landed there would
	 * have its first frame silently excluded.
	 */
	@Test
	fun theEarliestSampleIsNotRebasedOntoTheSentinel() {
		val player = TrackerFramesPlayer(
			recording(leftMicros = listOf(5_000L), rightMicros = listOf(5_000L)),
		)
		player.trackers.forEach { it.status = TrackerStatus.OK }
		player.setCursors(0)

		for (tracker in player.trackers) {
			assertEquals(
				PlayerTracker.REPLAY_EPOCH_MICROS,
				tracker.sampleHistory.newestMicros,
				"the earliest sample rebased onto ${tracker.sampleHistory.newestMicros}, " +
					"which TimeAlignment reads as 'never timestamped'",
			)
		}
	}

	/**
	 * A recorder ticking faster than a tracker reports sees the same sample
	 * twice. That is not a clock fault and must not be counted as one.
	 *
	 * `TrackerSampleHistory.outOfOrderSamples` documents itself as meaning UDP
	 * reordering, and that "a count that grows at anything like the sample rate
	 * is a broken clock estimate". Replaying duplicates straight into the
	 * history would make that counter grow at exactly the sample rate on every
	 * recording -- a permanent false alarm.
	 */
	@Test
	fun aRepeatedSampleIsNotCountedAsAClockFault() {
		val repeated = listOf(1_000L, 1_000L, 1_000L, 2_000L, 2_000L, 3_000L)
		val player = TrackerFramesPlayer(recording(repeated, repeated))
		player.trackers.forEach { it.status = TrackerStatus.OK }

		for (i in repeated.indices) player.setCursors(i)

		for (tracker in player.trackers) {
			println(
				"history size=${tracker.sampleHistory.size} " +
					"outOfOrder=${tracker.sampleHistory.outOfOrderSamples}",
			)
			assertEquals(
				0L,
				tracker.sampleHistory.outOfOrderSamples,
				"replaying a repeated sample was counted as out-of-order",
			)
			// Three distinct instants in, three retained.
			assertEquals(3, tracker.sampleHistory.size)
		}
	}

	/**
	 * Seeking backwards -- what the GUI scrubber does -- must not leave the
	 * history describing samples that, from the new cursor, have not happened.
	 */
	@Test
	fun seekingBackwardsDiscardsTheHistory() {
		val times = List(6) { 1_000L + it * 1_000L }
		val player = TrackerFramesPlayer(recording(times, times))
		player.trackers.forEach { it.status = TrackerStatus.OK }

		for (i in times.indices) player.setCursors(i)
		player.setCursors(1)

		for (tracker in player.trackers) {
			assertEquals(
				1,
				tracker.sampleHistory.size,
				"a backwards seek left ${tracker.sampleHistory.size} samples in the history",
			)
			assertEquals(0L, tracker.sampleHistory.discontinuities)
		}
	}

	/**
	 * A recording with no timestamps replays exactly as it did before the field
	 * existed: untimestamped, no history, and therefore no time alignment.
	 *
	 * That is the correct behaviour rather than a shortfall -- there is no
	 * information to align with -- but it must be *visibly* correct, because it
	 * is indistinguishable from the bug this change fixes if nobody checks.
	 */
	@Test
	fun anUntimestampedRecordingStillReplays() {
		val player = TrackerFramesPlayer(
			recording(leftMicros = List(4) { null }, rightMicros = List(4) { null }),
		)
		player.trackers.forEach { it.status = TrackerStatus.OK }
		for (i in 0 until 4) player.setCursors(i)

		assertNull(player.timestampOrigin)

		val alignment = TimeAlignment()
		alignment.align(player.trackers.toList())
		assertEquals(0, alignment.participants)

		for (tracker in player.trackers) {
			assertEquals(rotation, tracker.getRawRotation())
			assertEquals(0L, tracker.sampleHistory.newestMicros)
		}
	}

	/**
	 * A fleet where only some trackers report timestamps must not have the
	 * others silently pulled in, and must not lose alignment for the ones that
	 * do. The flag is per frame, so this mirrors the live behaviour exactly.
	 */
	@Test
	fun aMixedFleetAlignsOnlyTheTimestampedTrackers() {
		val base = 500_000L
		val player = TrackerFramesPlayer(
			recording(
				leftMicros = List(4) { base + it * 10_000L },
				rightMicros = List(4) { null },
			),
		)
		player.trackers.forEach { it.status = TrackerStatus.OK }
		for (i in 0 until 4) player.setCursors(i)

		val alignment = TimeAlignment()
		alignment.align(player.trackers.toList())

		// One eligible tracker is below the two alignment needs, so nothing is
		// touched -- which is what happens live with a single timestamped
		// tracker, and is the honest outcome rather than a failure.
		assertEquals(1, alignment.participants)
		assertEquals(0L, player.trackers[1].sampleHistory.newestMicros)
	}
}
