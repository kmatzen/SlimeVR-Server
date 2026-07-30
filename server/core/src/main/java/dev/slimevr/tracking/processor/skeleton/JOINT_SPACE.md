# Correcting the legs in joint space

Groundwork for issue #4, which argues the tracking pipeline's limit is
architectural: a chain of stages each correcting symptoms produced by the one
before it, information flowing only forwards, so a constraint discovered late
cannot inform the joint angles that violated it.

**This is not the sliding-window estimator that issue proposes.** It is that
issue's central claim turned into a measurement, plus the smallest change that
tests whether the claim leads anywhere. Off by default behind
`legTweaks.projectToJointSpace`.

## The measurement issue #4 was missing

Issue #4 lists a third prerequisite beyond the two it opened with:

> the corrected baselines are almost all exactly zero once the leg corrections
> engage, so they gate "the corrections still work" and cannot express
> degradation. Evaluating this proposal needs recordings with a nonzero
> residual — #15.

The premise is right and the conclusion does not follow. The baselines are
exactly zero because every metric in them asks whether the output is in the
right *place*. None asks whether the output is a body.

A skeleton has fixed segment lengths. Forward kinematics cannot violate them —
it composes rotations down a tree of fixed-length bones. `LegTweaks` does not
work that way:

- `correctClipping()` translates the ankle and the knee by one displacement and
  the hip by a different one
- `correctSkating()` sets a locked ankle's horizontal position directly

Both are the right *intent* — the foot really was planted, the foot really was
below the floor — expressed in the only notation available at that point in the
pipeline, which is position. And moving one end of a segment without the other
changes its length.

So there is a nonzero residual on synthetic motion after all. It was in a
channel nobody was measuring.

## What it costs

`SegmentConsistencyAccumulator` compares the distance between computed joint
positions against the same distance in the bone tree. Both are available on the
same frame of the same run: the corrections are written into the computed
trackers and never back into the bones, so the bone tree still holds the
uncorrected forward-kinematic pose and is an exact reference.

400 frames at 100 Hz:

| motion | | foot slide | floor clip | segment deformation (max) |
| --- | --- | --- | --- | --- |
| squat | FK only | 0.050289 m/s | 0.233865 m | 0.000000 m |
| squat | +legtweaks | **0.000000** | **0.000000** | **0.161902 m (11.4%)** |
| walk-in-place | FK only | 0.308897 m/s | 0.000000 m | 0.000000 m |
| walk-in-place | +legtweaks | **0.000000** | 0.000000 | **0.050914 m** |
| lean | FK only | 0.164599 m/s | 0.000000 m | 0.000000 m |
| lean | +legtweaks | **0.000000** | 0.000000 | **0.032851 m** |
| stand | either | 0.000000 | 0.000000 | 0.000000 m |

Forward kinematics alone deforms by **exactly zero** on every sequence, which is
the metric's own calibration — if it did not, the metric would be measuring some
offset between tracker bones and joint positions rather than deformation.

The corrections drive slide and clip to exactly zero and deform the leg by up to
**16 cm, 11% of a segment's length**. The error did not go away. It moved.

That is issue #4's architectural argument, stated as a number, on data that
already exists.

## The smallest constructive test

If the deformation is the price of the correction, nothing can be done about it
short of the full estimator. If it is not, the layered architecture is paying a
cost it does not have to, which is a much stronger version of the issue's claim.

`JointSpaceProjection` tests that directly. It takes the same ankle targets the
corrections already computed and asks a different question: *what is the closest
pose the leg can actually reach that puts the ankle there?*

That is two-link inverse kinematics with a closed form. Given the hip, the ankle
target and the two segment lengths, the knee lies on the intersection of two
spheres — a circle — and the only remaining freedom is the swing plane, resolved
by staying as close as possible to where the knee already was. Segment lengths
are preserved structurally: the output is *constructed* from them, so it cannot
violate them. No optimiser, no iteration, no per-frame allocation.

## Result

| motion | | foot slide | floor clip | segment deformation |
| --- | --- | --- | --- | --- |
| squat | +legtweaks | 0.000000 | 0.000000 | 0.161902 m |
| squat | **+jointspace** | **0.000000** | **0.000000** | **0.000000 m** |
| walk-in-place | +legtweaks | 0.000000 | 0.000000 | 0.050914 m |
| walk-in-place | **+jointspace** | **0.000000** | **0.000000** | **0.000000 m** |
| lean | +legtweaks | 0.000000 | 0.000000 | 0.032851 m |
| lean | **+jointspace** | **0.035034 m/s** | **0.000000** | **0.000000 m** |

On squat and walk-in-place the deformation was **free to remove**. Identical
slide, identical floor clip, zero deformation. The 16 cm of stretch bought
nothing — the same ankle targets were reachable by a pose the skeleton could
actually hold.

## The case where it is not free, which is the interesting one

`lean` is different, and it is the result worth dwelling on. Removing the
deformation costs 0.035 m/s of foot slide.

The `lean` sequence rotates the chest and hip while the headset stays pinned, so
the feet necessarily swing — the skeleton hangs from the head and the legs are
the far end of the chain. Uncorrected, the feet slide at 0.165 m/s. `LegTweaks`
pins them anyway, and the leg stretches to cover the difference.

The projection cannot do that, so it surfaces the disagreement: the pose says the
ankle is *here*, the contact constraint says it is *there*, and the leg is not
long enough to satisfy both. Something has to give, and the projection makes it
the ankle rather than the bone.

That is the trade-off issue #4 is about. A single objective would weigh the two
against each other using their uncertainties — perhaps the tracker orientations
are less trustworthy than the contact, in which case the joint angles should bend
further and the residual should land on the orientation term instead. A chain of
stages cannot do that. It can only let the later stage win, and the current code
lets it win by deforming the body.

**The projection does not solve this.** It relocates the residual from a channel
with no metric to one with a metric. That is worth doing on its own, and it is
also a demonstration that the objective needs writing down properly.

## What this does not do

- **It is not the sliding-window estimator.** One constraint, one stage, closed
  form. No window, no marginalisation, no optimiser, no orientation residuals,
  no uncertainty. The proposal in issue #4 remains unbuilt.
- **It does not subsume `LegTweaks`.** It runs after the corrections and cleans
  up after them; it does not replace their decision about *where* the foot should
  be. Contact detection still lives in `LegTweaksBuffer` — see #5, which measures
  it.
- **It does not touch the hip.** The corrected hip is taken as the root of the
  leg chain. The segments above it are not what these corrections deform.
- **It is not evaluated on real recordings.** Everything here is synthetic. What
  it establishes is that the deformation exists, is large, and is mostly
  unnecessary — none of which needed recordings. Whether removing it improves
  what a user perceives is a different question and does need #15.

## Corrections to issue #4 that this confirms or extends

- The third prerequisite (#15) applies to the metrics that were chosen, not to
  the question. Kinematic consistency is measurable now.
- The architectural argument holds, and the mechanism is more specific than the
  issue states: it is not merely that information flows forward, it is that the
  only notation available to a late stage is position, and position edits on a
  jointed body are not closed over valid poses.
- "Subsumes most of `LegTweaks`" remains the right ambition, but the measurements
  suggest the first target inside `LegTweaks` is not the corrections themselves —
  they work — it is that they have nowhere to put a constraint.
