package dev.slimevr.replay

import dev.slimevr.config.StayAlignedConfig
import dev.slimevr.poseframeformat.CorpusMetadata
import dev.slimevr.poseframeformat.PfrIO
import dev.slimevr.poseframeformat.PoseFrames
import dev.slimevr.tracking.processor.config.SkeletonConfigOffsets
import dev.slimevr.tracking.trackers.udp.IMUType
import java.io.File

/**
 * A committed `.pfr` recording together with the capture metadata needed to
 * interpret it.
 *
 * The sidecar schema itself lives in [CorpusMetadata], in main sources, because
 * `dev.slimevr.poseframeformat.CorpusCapture` writes the files this reads. A
 * schema defined twice drifts, and it drifts in the damaging direction: a
 * writer that omits a field this reader treats as optional produces a recording
 * that replays *successfully* with a correction silently switched off. This
 * class keeps what is specific to the suite -- where the corpus lives on disk,
 * how a recording is discovered and loaded -- and delegates the format to that
 * one definition.
 *
 * See `test/resources/corpus/README.md` for the schema and the capture
 * protocol.
 */
class CorpusRecording(
	val name: String,
	val file: File,
	val metadata: CorpusMetadata,
) {
	val fields: Map<String, String> get() = metadata.fields
	val offsets: Map<SkeletonConfigOffsets, Float> get() = metadata.offsets
	val imuType: IMUType? get() = metadata.imuType
	val stayAligned: StayAlignedConfig? get() = metadata.stayAligned

	/** Sample rate of the capture. Absent from the `.pfr` container itself. */
	val rateHz: Float get() = metadata.rateHz

	/** What this recording is for -- the failure mode it exercises. */
	val description: String get() = metadata.description

	fun load(): PoseFrames = PfrIO.readFromFile(file)

	/** One-line inventory entry, for the suite's report. */
	fun summary(): String = "%-20s %6.1f Hz  %s".format(name, rateHz, description)

	companion object {
		/** Resource directory holding the corpus, relative to test resources. */
		const val RESOURCE_DIR = "/corpus"

		/** Re-exported so the suite's own checks read against one list. */
		val REQUIRED = CorpusMetadata.REQUIRED

		/**
		 * Every recording in the committed corpus.
		 *
		 * Returns empty when the directory holds no `.pfr` files, which is the
		 * state of the repository until captures are made. It throws rather
		 * than skipping when a recording is *present but unusable*, because
		 * those are different situations: the first is work not yet done, the
		 * second is a committed file nobody can interpret.
		 */
		fun discover(): List<CorpusRecording> {
			val dir = directory() ?: return emptyList()
			val recordings = dir.listFiles { f -> f.isFile && f.name.endsWith(".pfr") }
				?: return emptyList()

			return recordings.sortedBy { it.name }.map { pfr ->
				val name = pfr.name.removeSuffix(".pfr")
				val meta = File(pfr.parentFile, "$name.meta")
				require(meta.isFile) {
					"corpus recording '$name' has no $name.meta sidecar. The .pfr " +
						"format carries no sample rate, so the file cannot be " +
						"replayed without one -- see corpus/README.md"
				}
				parse(name, pfr, meta.readText())
			}
		}

		/**
		 * Resolved through the classloader rather than a source-relative path,
		 * so this reads the same copy of the corpus the rest of the suite loads
		 * its resources from.
		 */
		fun directory(): File? {
			val url = CorpusRecording::class.java.getResource(RESOURCE_DIR) ?: return null
			if (url.protocol != "file") return null
			return File(url.toURI()).takeIf { it.isDirectory }
		}

		fun parse(name: String, file: File, text: String): CorpusRecording = CorpusRecording(
			name = name,
			file = file,
			metadata = CorpusMetadata.parse(name, text),
		)
	}
}
