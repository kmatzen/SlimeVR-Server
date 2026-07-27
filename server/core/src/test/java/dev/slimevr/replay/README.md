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
| `ReplayBaseline.kt` | loads/formats `test/resources/replay-baseline.txt` |
| `dev.slimevr.metrics.PoseMetrics` | the metrics themselves (in `main`, so non-test code can use them) |

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

## The determinism caveat

`metricsMatchBaseline` covers only the configuration with leg corrections
**disabled**. That path is bit-reproducible, so its tolerances are tight.

The leg corrections are not reproducible. `LegTweaksBuffer.kt:180` stamps every
frame with `System.nanoTime()` and `HumanPoseManager.kt:513` reads
`System.currentTimeMillis()`, so velocities are derived from real elapsed time
rather than the frame's simulated timestep. Two replays of byte-identical input
disagree by roughly 1e-4 m/s of foot slide — about 0.3% of the signal.

Small, but not zero, and that is the whole problem: it puts a floor under how
tight any baseline on those metrics can be, and that floor is set by machine
speed and load rather than by the code under test.

`clockDependentStagesAreOnlyApproximatelyReproducible` measures this and asserts
a loose bound, as a guard against it getting worse. **An injectable clock is the
prerequisite for extending baseline coverage to the leg corrections** — once it
lands, that test becomes an equality assertion and those metrics join the
baseline file.

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

Run the suite, take the printed `current` column, and write it into
`test/resources/replay-baseline.txt`.

**Read the diff before committing it.** Regenerating a baseline is how a real
regression gets blessed as expected behaviour, and it is the single most likely
way for this suite to stop being useful.

Tolerances should come from measured run-to-run spread, not from taste. For the
deterministic path the spread is genuinely zero, so the committed tolerances are
tight (2% with a 1e-4 absolute floor — the floor exists because several metrics
are legitimately zero, and 2% of zero is zero, which would be infinitely
strict).
