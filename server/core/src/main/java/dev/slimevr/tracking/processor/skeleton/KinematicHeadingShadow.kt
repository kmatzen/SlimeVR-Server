package dev.slimevr.tracking.processor.skeleton

import dev.slimevr.math.Angle
import dev.slimevr.tracking.processor.Bone
import dev.slimevr.tracking.processor.HingeJoint
import dev.slimevr.tracking.processor.KinematicHeadingSolver
import dev.slimevr.tracking.trackers.Tracker
import io.eiren.util.logging.LogManager
import io.github.axisangles.ktmath.Quaternion

/**
 * Runs the kinematic heading solve alongside Stay Aligned and logs both,
 * without touching a tracker.
 *
 * Stay Aligned nudges one tracker per tick toward a relaxed-pose model.
 * [KinematicHeadingSolver] derives the same quantity — relative tracker
 * heading — from the joints between the trackers instead, with an explicit
 * error model and the ability to say it does not know. Which one is better on
 * real motion, where the hinge assumption is approximate and joints are only
 * intermittently observable, is an empirical question, and this is the thing
 * that makes it answerable: both numbers, per tracker, from the same session.
 *
 * Shadow only, deliberately. Replacing the correction path before there is
 * evidence would be swapping one unmeasured heuristic for another.
 *
 * ## What the two numbers mean
 *
 * The solve is fed `tracker.getRotation()`, which is the rotation the skeleton
 * actually consumes and therefore already carries Stay Aligned's correction.
 * The kinematic figure is consequently a *residual*: what the joints say is
 * still wrong after Stay Aligned has had its say. Near zero means Stay Aligned
 * already has that pair right; the sum of the two columns is roughly what the
 * kinematic method would have asked for on its own.
 *
 * ## Cross-limb heading is not observable here
 *
 * Worth stating plainly, because the solver is built to reconcile a whole body
 * and this skeleton does not give it one. Hinges exist only at knees and elbows
 * (see [HingeJoint]), so the joint graph is not connected — it is a handful of
 * two-tracker components, one per limb, with no constraint tying a leg to the
 * torso or to the other leg. Only relative heading *within* a component is
 * observable, so each is solved against its own pinned reference and the
 * numbers are not comparable across limbs. Connecting the graph needs a
 * constraint this skeleton does not currently declare.
 *
 * ## Windowing
 *
 * [dev.slimevr.tracking.processor.KinematicHeading] accumulates without
 * forgetting, so left running it converges on a lifetime average — the wrong
 * shape for drift, which is exactly the time-varying thing being measured.
 * Each report therefore covers one bounded window and the estimators are reset
 * afterwards, which also discards history invalidated by a reset.
 */
class KinematicHeadingShadow(
	/**
	 * The skeleton's bones. Captured once: the objects are stable for the life
	 * of the skeleton, and it is only their constraints, parents and attached
	 * trackers that are re-read on rebuild.
	 */
	private val bones: Array<Bone>,
) {

	/** Where window boundaries come from; replay substitutes its own. */
	var clock: FrameClock = FrameClock.SYSTEM

	/** One tracker's two answers, both relative to its limb's reference. */
	data class TrackerResidual(
		val tracker: Tracker,
		/** What the joints say is still wrong. Meaningless unless [solved]. */
		val kinematicYaw: Angle,
		/** What Stay Aligned is currently applying. */
		val stayAlignedYaw: Angle,
		val solved: Boolean,
	)

	/** One connected component of the joint graph — in practice, one limb. */
	data class LimbReport(
		val label: String,
		val reference: Tracker,
		val observableJoints: Int,
		val totalJoints: Int,
		val residuals: List<TrackerResidual>,
	)

	/**
	 * The most recent window. Exposed so an evaluation harness can read it
	 * directly rather than parsing the log.
	 */
	var lastReport: List<LimbReport> = emptyList()
		private set

	private class Limb(
		val label: String,
		val reference: Tracker,
		val trackers: List<Tracker>,
		val jointCount: Int,
	) {
		val solver = KinematicHeadingSolver()

		/** Reused across frames; this runs at the server's full tick rate. */
		val rotations = HashMap<Int, Quaternion>(trackers.size * 2)
	}

	private var limbs: List<Limb> = emptyList()

	@Volatile
	private var stale = true
	private var windowStartNanos = 0L

	/**
	 * Marks the joint list as needing a rebuild.
	 *
	 * Both inputs can change under us: which tracker drives a bone, and the
	 * bone's `rotationOffset`, which the hinge axis is derived from.
	 */
	fun invalidate() {
		stale = true
	}

	/** Discards accumulated evidence, e.g. after a reset moved every tracker. */
	fun reset() {
		for (limb in limbs) limb.solver.reset()
		windowStartNanos = clock.nanos()
	}

	/**
	 * Folds in one frame, and reports if the window has elapsed.
	 *
	 * Must be called with the trackers time-aligned, i.e. after
	 * [dev.slimevr.tracking.trackers.TimeAlignment] has run for this tick.
	 */
	fun update() {
		if (stale) {
			rebuild()
			stale = false
		}
		if (limbs.isEmpty()) return

		for (limb in limbs) {
			limb.rotations.clear()
			for (tracker in limb.trackers) {
				limb.rotations[tracker.id] = tracker.getRotation()
			}
			limb.solver.observe(limb.rotations)
		}

		val now = clock.nanos()
		if (now - windowStartNanos < WINDOW_NANOS) return
		windowStartNanos = now

		report()
		for (limb in limbs) limb.solver.reset()
	}

	private fun rebuild() {
		val joints = HingeJoint.collect(bones.asIterable())
		limbs = componentsOf(joints).map { build(it) }
		windowStartNanos = clock.nanos()
		lastReport = emptyList()

		if (limbs.isEmpty()) {
			LogManager.debug("[KinematicHeading] No hinge joint has trackers on both sides")
		}
	}

	/**
	 * Splits the joints into connected components.
	 *
	 * Union-find over tracker ids. The components are what can be solved
	 * against a single gauge; see the note on the class.
	 */
	private fun componentsOf(joints: List<HingeJoint>): List<List<HingeJoint>> {
		val parent = HashMap<Int, Int>()

		fun find(x: Int): Int {
			var root = x
			while (parent[root] != root) root = parent[root]!!
			var cur = x
			while (parent[cur] != root) {
				val next = parent[cur]!!
				parent[cur] = root
				cur = next
			}
			return root
		}

		for (joint in joints) {
			parent.putIfAbsent(joint.parentTracker.id, joint.parentTracker.id)
			parent.putIfAbsent(joint.childTracker.id, joint.childTracker.id)
			parent[find(joint.parentTracker.id)] = find(joint.childTracker.id)
		}

		return joints.groupBy { find(it.parentTracker.id) }.values.toList()
	}

	private fun build(joints: List<HingeJoint>): Limb {
		val trackers = LinkedHashMap<Int, Tracker>()
		for (joint in joints) {
			trackers.putIfAbsent(joint.parentTracker.id, joint.parentTracker)
			trackers.putIfAbsent(joint.childTracker.id, joint.childTracker)
		}

		// Pin the most root-ward tracker: the one no joint treats as a child.
		// Only relative heading is observable, so something has to be pinned,
		// and choosing the limb's root keeps the reported offsets pointing the
		// way a correction would be applied. Ties broken by id for determinism.
		val childIds = joints.mapTo(HashSet()) { it.childTracker.id }
		val reference = trackers.values
			.filter { it.id !in childIds }
			.minByOrNull { it.id }
			?: trackers.values.minByOrNull { it.id }!!

		val limb = Limb(
			label = joints.joinToString("+") { it.boneType.name },
			reference = reference,
			trackers = trackers.values.toList(),
			jointCount = joints.size,
		)

		for (joint in joints) {
			limb.solver.addJoint(
				joint.parentTracker.id,
				joint.childTracker.id,
				joint.parentAxis,
				joint.childAxis,
			)
		}

		LogManager.debug(
			"[KinematicHeading] Tracking ${limb.label} (${joints.size} joint(s), " +
				"reference ${reference.name})",
		)
		return limb
	}

	private fun report() {
		val reports = ArrayList<LimbReport>(limbs.size)

		for (limb in limbs) {
			val observable = limb.solver.observableJoints
			val solutions = limb.solver.solve(limb.reference.id)

			val residuals = limb.trackers.map { tracker ->
				val solution = solutions[tracker.id]
				TrackerResidual(
					tracker = tracker,
					kinematicYaw = Angle.ofRad((solution?.yawRad ?: 0.0).toFloat()),
					stayAlignedYaw = tracker.stayAligned.yawCorrection,
					solved = solution?.solved ?: false,
				)
			}

			reports.add(
				LimbReport(
					label = limb.label,
					reference = limb.reference,
					observableJoints = observable,
					totalJoints = limb.jointCount,
					residuals = residuals,
				),
			)
		}

		lastReport = reports

		for (limbReport in reports) {
			// Nothing observable means the geometry never constrained heading
			// this window — a near-vertical hinge axis, or a limb that did not
			// move. That is a result, but not one worth a line every window.
			if (limbReport.observableJoints == 0) continue

			LogManager.debug(
				"[KinematicHeading] ${limbReport.label}: " +
					"${limbReport.observableJoints}/${limbReport.totalJoints} joints observable",
			)
			for (residual in limbReport.residuals) {
				if (residual.tracker.id == limbReport.reference.id) continue
				val kinematic = if (residual.solved) {
					"%+.2f".format(residual.kinematicYaw.toDeg())
				} else {
					"unsolved"
				}
				LogManager.debug(
					"[KinematicHeading]   ${residual.tracker.name}: " +
						"kinematic $kinematic deg, " +
						"stay aligned ${"%+.2f".format(residual.stayAlignedYaw.toDeg())} deg",
				)
			}
		}
	}

	companion object {
		/**
		 * Window length. Long enough that a limb has plausibly moved through
		 * enough of its range to make the hinge axis observable, short enough
		 * that the answer still describes drift rather than a session average.
		 */
		private const val WINDOW_NANOS = 10_000_000_000L
	}
}
