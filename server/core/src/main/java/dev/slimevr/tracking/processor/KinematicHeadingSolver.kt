package dev.slimevr.tracking.processor

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Solves for a globally consistent set of tracker yaw offsets from the joints
 * between them.
 *
 * [KinematicHeading] measures one joint. A body is a graph of them, and the
 * per-joint answers have to be reconciled: with several joints in a chain,
 * naively applying each pairwise correction in turn accumulates whatever error
 * each one has. This solves all of them at once instead.
 *
 * Each observable joint contributes an equation `yaw[b] - yaw[a] = m`, weighted
 * by that joint's confidence. Minimising the weighted squared residual is a
 * weighted graph Laplacian system, small enough (a handful of trackers) to
 * solve directly.
 *
 * ## Gauge
 *
 * Only *relative* headings are observable — rotating every tracker by the same
 * angle changes nothing about the pose, which is precisely why a 6-DoF setup
 * has no absolute heading in the first place. The system is therefore singular
 * until one tracker is pinned, and [solve] takes that reference explicitly
 * rather than picking one silently.
 *
 * ## Trackers it cannot reach
 *
 * Joints only constrain heading where a hinge exists — knees and elbows in this
 * skeleton. A tracker with no observable path back to the reference is reported
 * as unsolved rather than given a fabricated zero. Distinguishing "no
 * correction needed" from "no information" matters: the first is a result, the
 * second is silence, and treating silence as a result is how a correction
 * system introduces error it was supposed to remove.
 */
class KinematicHeadingSolver {

	private class Edge(
		val a: Int,
		val b: Int,
		val axisA: Vector3,
		val axisB: Vector3,
		val estimator: KinematicHeading = KinematicHeading(),
	)

	private val edges = mutableListOf<Edge>()
	private val ids = mutableListOf<Int>()

	/** Result for one tracker. */
	data class Solution(
		/** Yaw offset to apply about world vertical, radians. */
		val yawRad: Double,
		/** False when no observable joint path reached this tracker. */
		val solved: Boolean,
	)

	private fun indexOf(id: Int): Int {
		val i = ids.indexOf(id)
		if (i >= 0) return i
		ids.add(id)
		return ids.size - 1
	}

	/**
	 * Registers a hinge between two trackers.
	 *
	 * [axisA] and [axisB] are the hinge axis in each tracker's own frame. For
	 * this skeleton's knees and elbows that is the local -X axis, matching the
	 * hinge constraint in `Constraint.kt`.
	 */
	fun addJoint(trackerA: Int, trackerB: Int, axisA: Vector3, axisB: Vector3) {
		edges.add(Edge(indexOf(trackerA), indexOf(trackerB), axisA, axisB))
	}

	/** Feeds one frame of tracker orientations, keyed by the ids used above. */
	fun observe(rotations: Map<Int, Quaternion>) {
		for (e in edges) {
			val ra = rotations[ids[e.a]] ?: continue
			val rb = rotations[ids[e.b]] ?: continue
			e.estimator.addSample(ra, e.axisA, rb, e.axisB)
		}
	}

	/** Joints currently confident enough to contribute. */
	val observableJoints: Int
		get() = edges.count { it.estimator.hasEstimate }

	fun reset() {
		for (e in edges) e.estimator.reset()
	}

	/**
	 * Solves for every tracker's yaw offset relative to [referenceTracker],
	 * which is pinned at zero.
	 */
	fun solve(referenceTracker: Int): Map<Int, Solution> {
		val n = ids.size
		val result = mutableMapOf<Int, Solution>()
		if (n == 0) return result

		val refIdx = ids.indexOf(referenceTracker)
		if (refIdx < 0) return result

		val usable = edges.filter { it.estimator.hasEstimate }

		// Which trackers are reachable from the reference through observable
		// joints. Anything outside this set has no information at all.
		val reachable = BooleanArray(n)
		reachable[refIdx] = true
		var changed = true
		while (changed) {
			changed = false
			for (e in usable) {
				if (reachable[e.a] && !reachable[e.b]) {
					reachable[e.b] = true
					changed = true
				} else if (reachable[e.b] && !reachable[e.a]) {
					reachable[e.a] = true
					changed = true
				}
			}
		}

		// Weighted Laplacian: L y = c, with the reference row pinned.
		val l = Array(n) { DoubleArray(n) }
		val c = DoubleArray(n)
		for (e in usable) {
			val w = e.estimator.concentration * e.estimator.observability
			if (w <= 0.0) continue
			val m = e.estimator.relativeHeadingRad
			l[e.a][e.a] += w
			l[e.b][e.b] += w
			l[e.a][e.b] -= w
			l[e.b][e.a] -= w
			c[e.a] -= w * m
			c[e.b] += w * m
		}

		for (j in 0 until n) l[refIdx][j] = 0.0
		l[refIdx][refIdx] = 1.0
		c[refIdx] = 0.0

		// Unreachable trackers would leave the system singular; pin them too and
		// mark them unsolved afterwards.
		for (i in 0 until n) {
			if (!reachable[i]) {
				for (j in 0 until n) l[i][j] = 0.0
				l[i][i] = 1.0
				c[i] = 0.0
			}
		}

		val y = gaussianSolve(l, c, n)

		for (i in 0 until n) {
			result[ids[i]] = Solution(
				yawRad = if (y == null) 0.0 else wrapPi(y[i]),
				solved = reachable[i] && y != null,
			)
		}
		return result
	}

	private fun gaussianSolve(a: Array<DoubleArray>, b: DoubleArray, n: Int): DoubleArray? {
		val m = Array(n) { i -> DoubleArray(n + 1) { j -> if (j < n) a[i][j] else b[i] } }
		for (col in 0 until n) {
			var pivot = col
			for (r in col until n) {
				if (abs(m[r][col]) > abs(m[pivot][col])) pivot = r
			}
			if (abs(m[pivot][col]) < 1e-12) return null
			val t = m[col]
			m[col] = m[pivot]
			m[pivot] = t
			for (r in 0 until n) {
				if (r == col) continue
				val f = m[r][col] / m[col][col]
				if (f == 0.0) continue
				for (k in col..n) m[r][k] -= f * m[col][k]
			}
		}
		return DoubleArray(n) { i -> m[i][n] / m[i][i] }
	}

	private fun wrapPi(a: Double): Double {
		// Via sin/cos so the result is the canonical representative regardless
		// of how many turns the solver accumulated.
		return atan2(sin(a), cos(a))
	}
}
