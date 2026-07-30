package dev.slimevr.config

class AutoBoneConfig {
	var cursorIncrement = 2
	var minDataDistance = 1
	var maxDataDistance = 1
	var numEpochs = 50
	var printEveryNumEpochs = 25
	var initialAdjustRate = 10.0f
	var adjustRateDecay = 1.0f
	var slideErrorFactor = 1.0f
	var offsetSlideErrorFactor = 0.0f
	var footHeightOffsetErrorFactor = 0.0f
	var bodyProportionErrorFactor = 0.05f
	var heightErrorFactor = 0.0f
	var positionErrorFactor = 0.0f
	var positionOffsetErrorFactor = 0.0f
	var calcInitError = false
	var randomizeFrameOrder = true
	var scaleEachStep = true
	var sampleCount = 1500
	var sampleRateMs = 20L
	var saveRecordings = false
	var useSkeletonHeight = false
	var randSeed = 4L
	var useFrameFiltering = false
	var maxFinalError = 0.03f

	// #region Levenberg-Marquardt

	/**
	 * Solve with Levenberg-Marquardt instead of the greedy coordinate search.
	 *
	 * Off by default. Issue #7 asks for the existing optimiser to be kept
	 * behind a flag for comparison, and until the two have been compared on
	 * real recordings the existing one is the one users have been getting.
	 */
	var useLevenbergMarquardt = false

	/**
	 * Iteration cap. This is a backstop, not a schedule -- the solve is
	 * expected to stop on [lmCostTolerance] or [lmParameterTolerance], and
	 * hitting this instead is reported as a non-convergence.
	 */
	var lmMaxIterations = 100

	/** Relative cost reduction below which the solve is done. */
	var lmCostTolerance = 1e-8f

	/** Relative parameter movement below which the solve is done. */
	var lmParameterTolerance = 1e-8f

	/**
	 * Finite-difference step in log-length space.
	 *
	 * The skeleton solve is `Float`, so residuals carry ~1e-7 relative noise
	 * and differencing amplifies it by 1/h. For central differences the total
	 * error is minimised near `(3ε)^(1/3) ≈ 7e-3`.
	 */
	var lmJacobianStep = 5e-3f

	/**
	 * Weight on the soft height-normalisation residual.
	 *
	 * Large, because it stands in for what the existing algorithm does by hard
	 * renormalisation after every step. It is soft rather than hard so that
	 * `JᵀJ` stays non-singular in the uniform-scale direction and a covariance
	 * exists at all -- see [dev.slimevr.autobone.leastsquares.AutoBoneObjective].
	 */
	var lmHeightConstraintWeight = 100f

	/**
	 * Cap on frame pairs entering the objective, or 0 for no cap.
	 *
	 * Every pair costs two skeleton solves per residual evaluation, and a
	 * Jacobian needs 2n of those. A few hundred well-spread pairs constrain
	 * nine bone lengths as well as a few thousand do.
	 */
	var lmMaxFramePairs = 250
	// #endregion
}
