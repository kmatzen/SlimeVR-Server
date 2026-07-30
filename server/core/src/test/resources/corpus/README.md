# `.pfr` recording corpus

Real motion recordings, replayed through the pipeline by
`dev.slimevr.replay.CorpusReplayTest` and gated on the same
`replay-baseline.txt` as the synthetic suite.

**The corpus is currently empty.** Capturing recordings needs hardware and a
wearer; everything around them — discovery, metadata validation, the replay
driver, baseline keys, the CI table — is built and tested. Adding a recording is
the two-file drop described below, with no code change.

## Why this exists when synthetic motion already does

`SyntheticMotion` gives repeatability, isolation, and no assets to maintain. It
cannot give sensor noise, yaw drift, mounting error, or dropout, and those are
precisely what the heuristics in `LegTweaks` and `StayAligned` were written to
cope with.

The concrete consequence is visible in `replay-baseline.txt`: almost every
`+legtweaks` value is `0.000000`, because the corrections fully absorb clean
synthetic input. Those lines gate *"the corrections still work at all"* and
nothing finer — a change that made them meaningfully worse but still nominally
functional would have to move a metric off zero to be caught. Real recordings
have a nonzero residual by construction, and a nonzero residual is what a graded
regression metric requires.

## Adding a recording

Drop two files in this directory:

```
walk-in-place.pfr     the recording
walk-in-place.meta    how it was captured
```

Both are picked up automatically. A `.pfr` without a `.meta` is a hard failure,
not a skip — see below.

Capture with the server's own recorder (`PoseRecorder`, exposed in the GUI as
the AutoBone recording flow), which writes the same format this reads.

### Why the sidecar is mandatory

**The `.pfr` container does not store a sample rate.** `PoseFrames.frameInterval`
exists in memory and defaults to 0.02 s, but `PfrIO.writeFrames` never writes it
and `PfrIO.readFrames` never reads it — the format is a tracker count, then per
tracker a name, a frame count, and the frames. A `.pfr` on disk is a sequence of
poses with no statement of how fast they were sampled.

Every time-normalised metric depends on that number:

- `foot_slide_m_per_sec` is metres over planted *seconds*, and the seconds come
  entirely from the assumed rate. Replay a 100 Hz capture as 50 Hz and the same
  file reports half the skating.
- `LegTweaksBuffer` derives foot velocities from the frame interval and compares
  them against fixed thresholds. A wrong rate does not scale the output, it
  changes which frames are treated as planted at all.

So `rate_hz` is required and is read from the sidecar. The remaining required
fields answer the question that decides whether a recording survives: *can
someone who was not there tell what this file is, two years from now, when its
metrics move?* A recording that cannot be interpreted will eventually be deleted
by someone who cannot tell whether it still means anything.

### `.meta` format

`key = value`, one per line. `#` starts a comment. Blank lines ignored. Plain
text rather than JSON for the same reason `replay-baseline.txt` is: the file
exists to be read in a pull request diff.

| key | required | meaning |
| --- | --- | --- |
| `rate_hz` | ✅ | sample rate of the capture. Not recoverable from the `.pfr`. |
| `description` | ✅ | what this recording is for — the failure mode it exercises |
| `captured` | ✅ | ISO date of capture |
| `capturer` | ✅ | who wore the trackers and recorded it |
| `consent` | ✅ | their agreement to redistribution under the repo's licence |
| `trackers` | ✅ | count and body placement |
| `firmware` | | firmware version on the trackers |
| `notes` | | anything about the environment worth knowing |
| `offset.<NAME>` | | skeleton config in force at capture, e.g. `offset.UPPER_LEG = 0.42` |

`<NAME>` is a `SkeletonConfigOffsets` constant. Any offset given is applied
before replay, so a recording of a person who is not 1.58 m tall is not solved
against default proportions. Offsets not given keep their defaults.

Required fields are checked at load. Leaving one at a template placeholder
(`TODO`, `TBD`, `unknown`, …) is rejected explicitly: an unfilled field reads as
provenance without being any.

### Example

```ini
# walk-in-place.meta
rate_hz     = 100
description = 1 Hz in-place stepping; the main LegTweaks skating case
captured    = 2026-08-14
capturer    = A. Person <a@example.com>
consent     = Agreed to redistribution under the repository's licence, 2026-08-14
trackers    = 7 — head (HMD), chest, hip, both upper legs, both lower legs
firmware    = SlimeVR-Tracker-ESP v0.5.0
notes       = Hardwood floor, no magnetic anomalies noted

offset.UPPER_LEG = 0.42
offset.LOWER_LEG = 0.44
```

## Provenance and licensing

These are motion recordings of a real person. Whoever captured one should be
recorded, once and in writing, as having agreed to its redistribution under the
repository's licence — that is what the `consent` field is. It is cheap now and
awkward later.

Recordings are motion only: rotations, positions, accelerations. They contain no
video and no audio. They are still a recording of how a specific identifiable
person moves, so treat consent as real rather than as paperwork.

## What a good corpus covers

One recording per failure mode, from issue #1:

| recording | purpose |
| --- | --- |
| `standing-still` | pure drift; the null test |
| `walk-in-place` | `LegTweaks`' main case |
| `sit-to-stand`, `crouch` | floor clip and height behaviour |
| `full-body-loop` | fixed choreography ending where it started, so end-vs-start error is directly measurable |
| `known-bad-*` | reproduces an open bug, and then serves as its fix-verification |

Keep them short. Metrics are averages, and a long recording buries the interval
that matters in one that does not.

## Baseline policy

**Each recording gets its own metric block, keyed `corpus:<name>[+legtweaks]/<metric>`.**

Issue #15 asked for this to be decided up front rather than discovered later.
The alternative — contributing to a shared metric set — produces a more compact
baseline, but a movement in it cannot be attributed to a recording without
re-running the suite locally, which is exactly the moment a reviewer gives up
and regenerates instead. Per-recording blocks cost lines in a file nobody reads
top to bottom and buy attribution in the diff, which is the only part anybody
does read.

The `corpus:` prefix keeps a recording from ever colliding with a synthetic
sequence name.

Regenerate the same way as the synthetic block:

```sh
./gradlew :server:core:test --tests 'dev.slimevr.replay.CorpusReplayTest' \
    -Dreplay.writeBaseline=true
```

Then copy the emitted `corpus:` lines into `test/resources/replay-baseline.txt`.
**Read the diff first** — regenerating a baseline is how a real regression gets
blessed as expected behaviour.

Removing a recording means removing its baseline block.
`everyCorpusBaselineKeyHasARecording` fails otherwise, so a recording that
silently stops loading cannot quietly take its metrics out of the suite.

## The gap this does not close

`.pfr` stores **fused tracker output**, not raw IMU samples. So this
regression-tests the server given fixed tracker behaviour, which is the right
scope for this repository. It cannot catch a firmware fusion regression, and it
cannot evaluate a change that alters what the trackers report.

Closing that loop needs the raw-sample format from
kmatzen/SlimeVR-Tracker-ESP#3, where `tools/fusion-bench` already replays raw
IMU samples through the fusion filter against its own CI baseline. The two
formats were designed with the eventual join in mind; nothing has been built to
join them.
