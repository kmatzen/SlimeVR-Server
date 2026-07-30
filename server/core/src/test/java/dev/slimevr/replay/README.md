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
| `TimeSkewReplayTest.kt` | what per-tracker clock skew costs, and how much `TimeAlignment` recovers |
| `IKSolverReplayTest.kt` | what the IK solver does, now that it is called at all |
| `CorpusRecording.kt` | discovers committed `.pfr` recordings and validates their capture metadata |
| `CorpusReplay.kt` | drives a recording through the pipeline into the same metrics |
| `CorpusReplayTest.kt` | gates the corpus on the baseline, and proves the `.pfr` path end to end |
| `CorpusMetadataTests.kt` | the sidecar parser's own tests |
| `FixedStepClock.kt` | frame clock driven by the sequence's timestep, not the host |
| `ReplayBaseline.kt` | loads/formats `test/resources/replay-baseline.txt` |
| `dev.slimevr.metrics.PoseMetrics` | the metrics themselves (in `main`, so non-test code can use them) |
| `dev.slimevr.tracking.processor.skeleton.FrameClock` | the injection point, in `main` |

## Two motion sources, one set of metrics

| source | what it is | status |
| --- | --- | --- |
| `SyntheticMotion` | closed-form sequences, no assets | 4 sequences, baselined |
| `.pfr` corpus | real captures replayed by `TrackerFramesPlayer` | **wired, empty** |

Everything downstream of the motion source is shared — `PoseMetrics`,
`ReplayBaseline`, `FixedStepClock`, the configuration matrix — so a corpus
metric and a synthetic metric of the same name mean the same thing, and a change
to the metrics moves both together.

Synthetic motion is complementary rather than inferior: it needs no captured
data, it is reproducible, and it can isolate one behaviour at a time in a way a
real recording never can. The sequences are anatomically approximate on purpose;
a regression baseline needs repeatability, not realism.

What it cannot give is sensor noise, yaw drift, mounting error, or dropout —
the conditions the heuristics were written for. The consequence is visible in
`replay-baseline.txt`: almost every `+legtweaks` value is `0.000000`, because
the corrections fully absorb clean input. Those lines gate *"the corrections
still work at all"* and nothing finer. **Recordings are the difference between a
suite that detects breakage and one that detects degradation.**

### The corpus path is built and empty

`server/core/src/test/resources/corpus/` takes a `.pfr` plus a `.meta` sidecar
and needs no code change to pick them up. See its README for the capture
protocol, the metadata schema, provenance, and the baseline policy.

The one thing that could not be built is the recordings themselves, which need
hardware and a wearer.

Wiring nothing exercises is wiring that rots, so
`pfrRoundTripReproducesTheSyntheticBaseline` keeps the path honest until
captures land: it pushes `squat` out through `PfrIO`, reads it back, replays it
through the corpus driver, and requires the result to match the *committed
synthetic baseline* for that same sequence. It currently matches to
`0.000000` on all 14 metrics across both configurations, which pins two separate
claims:

- **The format is lossless for what the pipeline consumes.** Rotations and
  positions survive a write/read cycle bit-identically.
- **The corpus driver is equivalent to the synthetic driver.** Trackers
  reconstructed by `TrackerFrames.toTracker()` are *not* configured identically
  to the ones `SkeletonReplayTest` builds by hand — notably the reconstructed
  head tracker is not flagged `isHmd`. Those differences demonstrably do not
  reach the solved pose, which is what makes a corpus metric comparable to a
  synthetic one.

It is not a substitute for the corpus and cannot be: it is the same clean
synthetic motion. It proves the path works, not that the pipeline handles
reality.

### `.pfr` does not store a sample rate

Worth knowing before capturing anything. `PoseFrames.frameInterval` exists in
memory and defaults to 0.02 s, but `PfrIO` neither writes nor reads it — the
format is a tracker count, then per tracker a name, a frame count, and the
frames.

Every time-normalised metric depends on that number. `foot_slide_m_per_sec` is
metres over planted *seconds*; replay a 100 Hz capture as 50 Hz and the same
file reports half the skating. `LegTweaksBuffer` is worse, because it compares
frame-interval-derived velocities against fixed thresholds — a wrong rate does
not scale the output, it changes which frames count as planted at all.

That is why `rate_hz` is a required sidecar field and a `.pfr` without a
sidecar is a hard failure rather than a file replayed at a guessed rate.

## The IK solver, and what turning it on is worth

`IKSolver.solve()` had no call site — not here, not in SlimeVR upstream, and
never at any point in either history. The introducing commit (upstream
`0a08d574`, "Positional tracker support (#920)", 31 Oct 2025) added the field,
the chain builder, the `USE_POSITION` wiring and the reset path, and no caller.
`IKSolver.enabled` is read only by the early return at the top of `solve()`, so
the setting could not do anything. See issue #4.

It is now called from `HumanSkeleton.updatePose()`, and `USE_POSITION` defaults
to **false** — it shipped as true while doing nothing, so leaving it on would
change every existing user's pose on upgrade using a path that has never run.

Two measured facts, both pinned by `IKSolverReplayTest`:

- **On a rotation-only tracker set it changes nothing, and cannot.**
  `buildChains` discards the root chain unless some chain has a *tail*
  constraint — its own comment is "check if there is any constraints (other
  than the head) in the model" — and a normal SlimeVR set has exactly one
  positional tracker, the headset, which *is* the root. So `rootChain` is null
  and `solve()` returns on its first line. Turning the setting on cannot affect
  the common configuration.
- **Given a positional tracker below the root it does a great deal.** Adding a
  left-foot tracker that reports position, on `squat`:

  | metric | off | on |
  | --- | --- | --- |
  | `foot_slide_m_per_sec` | 0.050289 | 0.022453 |
  | `floor_clip_mean_m` | 0.141896 | 0.000060 |
  | `floor_clip_max_m` | 0.233865 | 0.000060 |
  | `floor_clip_fraction` | 0.997500 | 0.002500 |
  | `foot_height_disagreement_m` | 0.000000 | 0.016111 |

  Floor penetration essentially eliminated, foot slide down 55%. The one metric
  that worsens is the feet disagreeing about the floor, which is expected when
  only *one* foot has a positional constraint pulling it.

**Disable the leg corrections when comparing anything here.** The first version
of this test left them on, and on clean synthetic input they drive every metric
to exactly zero — so it simultaneously reported that the solver changed nothing
and that it changed everything, both meaningless. Same trap as the `+legtweaks`
baseline rows.

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

## Measuring clock skew

`TimeSkewReplayTest` uses the same machinery for a different question: not *did
this change regress*, but *what is this correction worth*. Issue #2 asked for
the number before the correction was written, on the grounds that a small number
is also a result.

Three runs per motion, differing only in *when* each tracker sampled it:

| run | tracker *k* samples at | meaning |
| --- | --- | --- |
| `reference` | `t - maxDelay` | everyone equally late — input lag, no disagreement. The target. |
| `skewed` | `t - delay(k)` | independent delays, server told nothing. The old behaviour. |
| `aligned` | `t - delay(k)`, timestamped | same samples, `TimeAlignment` resolves them to a common tick. |

The gap between `skewed` and `reference` is what skew costs. The gap between
`aligned` and `reference` is what survives correcting it.

The `timestamped` flag is a straight toggle between old and new behaviour rather
than a test-only pipeline: with no sample history `TimeAlignment` finds no
participants and touches nothing, which is exactly what happens on firmware that
cannot report sample timestamps.

With 1.3–14.9 ms of per-tracker delay — single-digit milliseconds of spread is
the WiFi behaviour issue #2 describes — and the plain solver:

| motion | metric | reference | skewed | aligned | recovered |
| --- | --- | --- | --- | --- | --- |
| `walk-in-place` | `foot_slide_m_per_sec` | 0.295661 | 0.303746 | 0.296161 | 93.8% |
| `walk-in-place` | `foot_slide_total_m` | 1.561095 | 1.606827 | 1.563737 | 94.2% |
| `squat` | `floor_clip_mean_m` | 0.148815 | 0.148930 | 0.148816 | 98.9% |
| `squat` | `foot_height_disagreement_m` | 0.000000 | 0.000229 | 0.000001 | 99.7% |
| `lean` | `foot_slide_total_m` | 1.212115 | 1.206900 | 1.212094 | 99.6% |

So on this corpus the cost is **~2.7% of foot slide during fast leg motion**, and
interpolation removes about 94% of it. That is a real effect and a modest one —
worth knowing in both directions. Four things bound how far it generalises:

- **The residual is not expected to be zero.** Slerp between two real samples is
  not the true motion between them, so a sample interval's worth of
  interpolation error survives by construction.
- **`walk-in-place` is the sensitive case and it is the mild one.** 1 Hz leg
  lifts are slow next to real fast motion, which is where issue #2 expects the
  damage. This corpus cannot show that; recordings can.
- **The `+legtweaks` columns are almost all zero**, for the same reason the
  baseline's are — see below. They measure how much skew damage survives the
  heuristics, not how much there was.
- **The headset is excluded.** It is a position source with no rotation history,
  so nothing can interpolate it, and in production its samples arrive over a
  different transport carrying no tracker timestamp. Assertions are restricted
  to motions with a static head height (`SyntheticMotion.staticHeadHeight`),
  which removes that term rather than hiding it. It is a real limitation: an
  un-timestamped headset leading the IMUs leaves a head-to-body offset alignment
  cannot correct. `squat` is reported, not asserted, for exactly this reason.

The first 20 frames are excluded from the metrics. Until a tracker's history
brackets the reference, alignment clamps instead of interpolating; that
transient is a genuine startup cost but it is not a property of the correction,
and on the low-signal motions it is larger than the skew being measured.

Two guards keep the measurement from passing by measuring nothing:
`skewDegradesTheWalkPose` fails if the skew does not move foot slide at all, and
`interpolationRecoversMostOfWhatSkewCosts` fails if no metric cleared the
threshold to be judged.

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

Corpus keys live in the same file under a `corpus:` prefix and are regenerated
separately, since the two suites measure different motion:

```sh
./gradlew :server:core:test --tests 'dev.slimevr.replay.CorpusReplayTest' \
    -Dreplay.writeBaseline=true
```

Each recording gets its own metric block, keyed
`corpus:<name>[+legtweaks]/<metric>` — the reasoning is in `corpus/README.md`.
Removing a recording means removing its block;
`everyCorpusBaselineKeyHasARecording` fails otherwise, so a recording that
silently stops loading cannot quietly take its metrics out of the suite.

**Read the diff before committing it.** Regenerating a baseline is how a real
regression gets blessed as expected behaviour, and it is the single most likely
way for this suite to stop being useful.

Tolerances should come from measured run-to-run spread, not from taste. With the
clock injected the spread is genuinely zero on every covered configuration, so
the committed tolerances are tight throughout (2% with a 1e-4 absolute floor —
the floor exists because several metrics are legitimately zero, and 2% of zero
is zero, which would be infinitely strict).
