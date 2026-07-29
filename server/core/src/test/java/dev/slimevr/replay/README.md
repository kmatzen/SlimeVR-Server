# Skeleton replay regression suite

Replays a deterministic motion sequence through a real `HumanPoseManager`,
reduces the result to pose-quality metrics, and compares them against committed
baselines. This is what lets a change to the solver or to the leg corrections be
argued about with numbers instead of impressions.

```sh
./gradlew :server:core:test --tests 'dev.slimevr.replay.*' --info
```

`--info` shows the metrics table; without it you only get pass/fail.

## What's here

| file | role |
| --- | --- |
| `SyntheticMotion.kt` | deterministic tracker motion — `stand`, `squat`, `walk-in-place`, `lean` |
| `SkeletonReplayTest.kt` | drives the pipeline, computes metrics, gates on the baseline |
| `FixedStepClock.kt` | frame clock driven by the sequence's timestep, not the host |
| `ReplayBaseline.kt` | loads/formats `test/resources/replay-baseline.txt` |
| `dev.slimevr.metrics.PoseMetrics` | the metrics themselves (in `main`, so non-test code can use them) |
| `dev.slimevr.tracking.processor.skeleton.FrameClock` | the injection point, in `main` |

## Why synthetic motion

The server already has a full `.pfr` record/replay implementation in
`dev.slimevr.poseframeformat`, including `TrackerFramesPlayer`, which replays
recordings back in as live trackers. That is the right input for testing against
real captures — but there are no recordings in the repository.

Synthetic motion fills the gap and is complementary rather than inferior: it
needs no captured data, it is reproducible, and it can isolate one behaviour at
a time in a way a real recording never can. A `.pfr` corpus should be added on
top of this, not instead of it.

The sequences are anatomically approximate on purpose. A regression baseline
needs repeatability, not realism.

## Measure the computed trackers, not the skeleton bones

This is the subtle part, and getting it wrong makes the whole suite silently
vacuous.

`LegTweaks` does not write its corrections back into the skeleton's bone
transforms. It writes them into a buffer consumed by the computed trackers, and
`HumanSkeleton.update()` calls `updateComputedTrackers()` *before*
`legTweaks.tweakLegs()`. So reading `skeleton.getBone(LEFT_LOWER_LEG)` shows the
solver's raw output with none of the leg corrections applied.

Measured from the bones, enabling floor clip changes nothing at all. Measured
from the computed trackers, the same sequence goes from 0.234 m of floor
penetration to 0.0. The suite therefore reads
`hpm.getComputedTracker(TrackerRole.LEFT_FOOT).position`, which is also what
SteamVR actually receives.

`skatingCorrectionChangesTheResult` exists specifically to catch this class of
mistake: it fails if toggling the correction changes no metric, on the grounds
that a test exercising a disabled code path is worse than no test.

## Determinism, and the injected clock

`metricsMatchBaseline` covers **both** configurations: the plain solver, and the
solver with the leg corrections engaged (the `+legtweaks` baseline keys). Both
are bit-reproducible, so tolerances are tight throughout.

That was not always true. `LegTweaksBuffer` derives foot velocities from the
interval between consecutive frames, and it used to read that interval from
`System.nanoTime()` — so velocities came from real elapsed time rather than the
sequence's timestep, and two replays of byte-identical input did not agree.
Measured drift was around 1e-4 m/s of foot slide on the machine that first
recorded it and a few times 1e-6 on others; the magnitude was never the point,
since it was set by machine speed and load rather than by the code under test.

The buffer now takes its timestamps from a `FrameClock`. Production keeps
`FrameClock.SYSTEM` and behaves exactly as before. Replay installs a
`FixedStepClock` advanced once per frame by the sequence's own timestep, so the
interval is whatever the sequence says it is and the run-to-run spread is
exactly zero. `replayIsDeterministicWithLegTweaks` asserts equality, not a
bound.

Two consequences worth knowing:

- **Install the clock last.** Each of the `SkeletonConfigToggles` setters resets
  the frame buffer, and so does assigning `legTweaks.clock`. Assigning it after
  the toggles guarantees no frame stamped by the system clock survives into the
  replay. Mixing stamps from two clocks produces one garbage velocity frame,
  which is exactly the kind of thing that shows up as an unexplained baseline
  movement months later.
- **A stalled clock is now reachable.** `getTimeDelta()` returns 1/dt, so a zero
  interval would put an infinity into every threshold comparison in the buffer.
  `System.nanoTime()` made that all but impossible; an injected clock does not,
  so a zero interval is treated as the no-parent case and returns 0.
  `aStalledClockDoesNotProduceInfiniteVelocities` pins that.

### Why most `+legtweaks` numbers are zero

Because the corrections fully eliminate the artifacts these metrics measure on
clean synthetic input — verified, not assumed. On `squat` the left ankle goes
from a minimum of -0.2339 m (clipping on all 400 frames) to a minimum of ~0 and
a maximum of 0.0129 m, still planted on all 400 frames, with horizontal travel
while planted falling from 0.2012 m to 0. The feet are being locked, not lifted
out of the measured region.

So those lines gate *"the corrections still work at all"* — any nonzero value is
a regression. They cannot express *"slightly worse"*, because there is no
headroom left. Graded measurement wants recordings with real sensor noise and
drift, where the residual is not zero.

## The metrics

All lengths in metres; the world frame is Y-up with the floor at y=0.

- **`foot_slide_m_per_sec`** — mean horizontal ankle speed while planted. A
  planted foot is in contact with the world and must not move, so whatever
  distance it covers is error. This is the skating metric, and it is the
  acceptance criterion for contact-detection work.
- **`floor_clip_mean_m` / `floor_clip_max_m` / `floor_clip_fraction`** —
  penetration below the floor plane.
- **`foot_height_disagreement_m`** — both feet on the ground should agree about
  where the ground is.
- **`height_m`** — guards against proportion changes moving everything else.

`dev.slimevr.autobone.errors` already contains equivalent *ideas* — `SlideError`
is documented as "the change in position of the ankle over time" — but only as
an objective function for the optimiser, taking two skeletons and returning a
scalar to minimise. `PoseMetrics` exposes the same quantities in the shape a
regression suite needs: accumulated over a whole sequence, in physical units,
with no optimiser attached.

## Regenerating the baseline

```sh
./gradlew :server:core:test --tests 'dev.slimevr.replay.SkeletonReplayTest' \
    -Dreplay.writeBaseline=true
```

That emits a formatted block between `--- BEGIN replay-baseline.txt ---` and
`--- END ---`. Copy the *values* into `test/resources/replay-baseline.txt`,
keeping the explanatory comments already in that file.

**Read the diff before committing it.** Regenerating a baseline is how a real
regression gets blessed as expected behaviour, and it is the single most likely
way for this suite to stop being useful.

Tolerances should come from measured run-to-run spread, not from taste. With the
clock injected the spread is genuinely zero on every covered configuration, so
the committed tolerances are tight throughout (2% with a 1e-4 absolute floor —
the floor exists because several metrics are legitimately zero, and 2% of zero
is zero, which would be infinitely strict).
