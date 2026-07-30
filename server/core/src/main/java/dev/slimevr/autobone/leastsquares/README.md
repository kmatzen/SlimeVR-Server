# AutoBone as least squares

Solves for bone lengths with Levenberg-Marquardt instead of the greedy
coordinate search in `AutoBone.step()`, and reports how well the recording
actually determined each one.

Off by default. Set `autoBone.useLevenbergMarquardt = true` to use it. Issue #7
asks for the existing optimiser to stay available for comparison, and until the
two have been compared on real recordings the existing one is what users have
been getting.

## What the existing optimiser actually is

Worth stating plainly, because issue #7 describes it as gradient descent over a
sum of squared residuals and it is neither.

`AutoBone.step()` is a **greedy coordinate search**. For each bone in turn it
proposes a length change whose direction comes from `BoneContribution` — a
geometric estimate of how much that bone can move the foot along the observed
slide direction — and keeps the change only if the error drops. No gradient is
formed.

The objective is **the square of a weighted sum**, not a sum of squares: per
frame pair it computes `errorDeriv = Σ wₖ·eₖ`, and `0.5·errorDeriv²` is used
only to size the step. The accept/reject test is applied per frame pair, so the
procedure never descends any single global objective.

The consequence matters for how this change can be validated. Issue #7 proposes
"LM and gradient descent should converge to the same optimum" as the correctness
test, and there is no shared optimum to converge to until one is written down.
`AutoBoneObjective` writes it down.

## The objective

For enabled error terms *k* with configured weights *wₖ*, over sampled frame
pairs *p*:

```
F(θ) = ½ Σₚ Σₖ (wₖ/P)·eₖ(p;θ)²  +  ½·w_h·h(θ)²
```

Dividing by the pair count keeps `F` a mean over the recording, so the weights
mean what they already mean in the config and the scale does not move when a
recording gets longer.

Every `eₖ` is non-negative and vanishes at a perfect fit, so squaring them is
well posed. What changes against the current scalar is that cross terms
disappear — `Σ(w·e)²` rather than `(Σw·e)²`. That is the standard least-squares
reading of the same error terms, and the form that makes a Jacobian, a
covariance and a convergence criterion exist.

### Log lengths

Parameters are `θᵢ = ln(Lᵢ)`.

Bone lengths are positive and an unconstrained solver does not know that. The
existing code copes with a guard — *"No small or negative numbers!!! Bad
algorithm!"* — that silently skips any proposal below 0.01 m, and a guard that
silently skips is one that can stall the search without saying so. In log space
a negative length is not representable.

It also makes steps scale-free: 1% of a 0.05 m neck and 1% of a 0.45 m thigh are
the same distance in θ, which is what a solver with one shared damping parameter
needs.

### The height residual

`h(θ) = Σ(adjustable height bones) − normalizedHeight`, as a **soft** residual.

The existing algorithm hard-renormalises the height offsets after every step.
Doing that inside a residual function would make the objective exactly invariant
to a uniform scaling of the height bones, so `JᵀJ` would be exactly singular in
that direction and the covariance — the thing this is for — would not exist.

## Covariance, and one easy way to get it wrong

Commons-math's `Evaluation.getCovariances()` returns `(JᵀJ)⁻¹` and `getSigma()`
returns `sqrt(diag((JᵀJ)⁻¹))`. **Neither is scaled by the residual variance.**
Reporting `getSigma` directly is wrong by the noise level, and wrong in the
flattering direction whenever the fit is poor. What is computed here:

```
s²      = Σr²/(m−n)        residual variance
Cov(θ)  = s²·(JᵀJ)⁻¹       in log-length space
σ_L     = L·σ_θ            delta method, back to metres
```

A log-space sigma is already a relative uncertainty, which is usually the more
useful number: `σ_θ = 0.05` means "determined to about ±5%".

`JᵀJ` is inverted through an eigendecomposition rather than directly, because
the eigenvalues are useful output in their own right. The smallest one names the
parameter combination the recording failed to determine.

## Why the eigenvector matters more than the diagonal

Individually well-determined parameters can still leave a *combination* badly
determined, and that is invisible in the diagonal of the covariance.

`AutoBoneLeastSquaresTests` demonstrates it on a recording where the knee never
bends, so the leg swings as one rigid segment:

| recording | total leg length | thigh/shin split | condition number |
| --- | --- | --- | --- |
| knee bends | recovered | recovered, ±3% | 199 |
| knee rigid | recovered to 0.01% | **off by 4.1 cm** | 20 007 |

Both come back as two confident-looking numbers. The covariance is what
distinguishes them, and it does so by naming the direction — `+0.73·upperLeg
−0.68·lowerLeg`, i.e. the split — rather than by any single bone looking
suspicious. This is the shape of the "AutoBone gave me weird proportions"
report where total limb length is right and the split is not.

## Validation

There are no real recordings in the repository (issue #15), so the tests
generate one whose answer is known: motion is produced *from* a skeleton with
chosen bone lengths, and the headset is translated per frame so the ankles land
on the same point every time. Since the chain hangs rigidly from the headset,
one correction plants the feet exactly.

Replayed with the lengths it was generated from, that recording has zero slide
by construction — so the optimum is known and "did the solver find it" is a real
question. Measured: the worst slide residual at the true lengths is 7e-9.

Results from a start 24% short on the thigh and 22% long on the shin, which also
swaps their order:

| | thigh | shin | mean step error | evaluations |
| --- | --- | --- | --- | --- |
| start | 0.32 | 0.56 | 5.65e-3 | — |
| **LM** | **0.42** | **0.46** | **2.9e-8** | 110 |
| first-order descent | 0.338 | 0.540 | 4.56e-3 | 110 |
| truth | 0.42 | 0.46 | | |

The descent baseline uses the same decay rule as `AutoBone.decayFunc`, the same
finite-difference gradients, and the same objective, with its initial rate swept
over seven decades and the best result taken — rates at and above 1e3 collapse
the lengths to zero and 1e6 diverges to NaN. Sweeping is itself the complaint:
the answer depends on the schedule rather than on the data. LM has no such knob.

Uncertainties are checked for calibration, not just for being small: with sensor
noise applied, the true lengths must fall within 2σ of the estimate.

## Height

`scaleEachStep` decides whether height is solved for, matching what the setting
already means for the greedy path: set, and height is pinned to the target;
clear, and it is estimated.

The greedy path estimates it in the block guarded by `!scaleEachStep`, which
tries one step up and one step down in height and keeps whichever scored better
— a one-dimensional line search interleaved with, but separate from, the bone
search. Here height is one more column of the Jacobian, as `θ = ln(height)`, so
it can trade off against bone lengths inside a single step and the coupling
between them appears in the covariance instead of being invisible.

Each residual evaluation rescales the recording by `1/height` before measuring.
The offsets live in normalised units where the user is 1 unit tall, so it is the
recording that moves to meet them.

Recovering it is tested the same way the lengths are. The synthetic recording is
generated unscaled, so the correct height parameter is exactly 1.0 by
construction:

| | thigh | shin | height |
| --- | --- | --- | --- |
| start | 0.36 | 0.52 | 1.15 |
| **solved** | **0.4199** | **0.4601** | **1.0000 ± 0.0006** |
| truth | 0.42 | 0.46 | 1.00 |

The obvious worry is that height is redundant with a uniform scale-up of every
bone. It is not, and that is asserted rather than assumed: the recording's
tracker positions are scaled by `1/height` while the bones are not, so the two
do not cancel. Freeing height moves the condition number from 199 to 249 — an
extra free parameter always costs something, but a redundant one would cost
orders of magnitude.

## What this does not do

- **Surface uncertainty in the UI.** `AutoBoneResults.solution` carries it and
  `AutoBoneSolution.poorlyDetermined()` is the query a UI wants. Putting it on
  screen is blocked outside this repository: `solarxr-protocol` is the upstream
  `SlimeVR/SolarXR-Protocol`, so the schema change cannot land here. The delta
  is one backward-compatible optional field —

  ```
  table SkeletonPart {
      bone: SkeletonBone;
      value: float;
      /// 1σ uncertainty on `value`, in metres, when the estimate came from a
      /// solver that produces one. Null when unavailable.
      sigma: float = null;
  }
  ```

  — plus regenerated bindings, a submodule bump, and GUI work to render it.
  `RPCAutoBoneHandler` already builds this vector, at the
  `createAdjustedSkeletonPartsVector` call, so the server side is a one-line
  change once the field exists.
- **Retire `initialAdjustRate` / `adjustRateDecay`.** Issue #7 puts that after
  the new path is trusted, and it is not trusted until it has been run against
  the greedy path on real recordings.
- **Compare against the greedy path head to head.** That needs the recordings
  issue #15 is about. The comparison here is against first-order descent on the
  same objective, which is the specific claim issue #7 makes about iteration
  counts.
