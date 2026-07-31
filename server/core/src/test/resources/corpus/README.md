# `.pfr` recording corpus

Real motion recordings, replayed through the pipeline by
`dev.slimevr.replay.CorpusReplayTest` and gated on the same
`replay-baseline.txt` as the synthetic suite.

**The corpus is currently empty.** Capturing recordings needs hardware and a
wearer; everything around them — the capture command, discovery, metadata
validation, the replay driver, baseline keys, the CI table — is built and
tested. Adding a recording is one console command and a two-file drop, with no
code change.

**Start at [Building the dataset](#building-the-dataset)** if you are planning a
session rather than adding a file. It covers what unlocks at how many trackers —
several open questions need far fewer than a full body — and the decisions that
have to be made before anything is recorded rather than after.

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

## Capturing one

With trackers connected, type this at the running server's console:

```
record-corpus <name> <seconds> [rate-hz] [output-dir]
```

for example `record-corpus walk-in-place 180`. It prompts for the three fields
no machine can supply — what the recording is for, who wore the trackers, and
their consent — then records, and writes `<name>.pfr` and `<name>.meta` together
into `./corpus` (or the directory you name). Move both into this directory and
commit them as a pair.

`rate-hz` defaults to 100. Sample rate is not recoverable afterwards, so it is
recorded rather than assumed — see below.

### Why a command rather than the AutoBone flow

Earlier revisions of this file said to capture with the AutoBone recording flow
in the GUI, "which writes the same format this reads." **It does not.**
`AutoBoneHandler` calls `AutoBone.saveRecording`, which writes `.pfs` — a
different container, into AutoBone's own directory, with no sidecar. Renaming a
`.pfs` to `.pfr` does not convert it.

Worse, the mistake is silent in the direction that matters. `discover()` matches
`*.pfr` and ignores anything else, so a `.pfs` dropped into this directory is
not an error — the recording simply is not in the suite, and no test says so.

### What the command fills in for you

Everything it can read off the running server, because those are the fields a
person would otherwise transcribe from memory after the session:

| field | source |
| --- | --- |
| `rate_hz` | the rate the recording was made at |
| `captured` | today |
| `trackers` | count and body placement of the connected trackers |
| `firmware` | firmware versions reported by the devices |
| `imu_type` | the IMU the trackers report |
| `stay_aligned.*` | the relaxed poses in force, when Stay Aligned is enabled |
| `offset.*` | the skeleton proportions in force |

The last three are the ones worth the machinery. All three fail *silently* when
absent or wrong — the replay completes, every metric is produced, and yaw
correction never ran. See "Two fields that decide whether Stay Aligned runs at
all" below for the mechanism.

The command also writes deliberately *less* than it knows in one case: if Stay
Aligned was disabled at capture it writes no `stay_aligned.*` keys at all,
rather than writing the disabled poses. Any such key switches the correction
**on** for the replay, so emitting them would make the replay run a correction
the recording was never made under.

After writing, it reads both files back and parses the sidecar with the same
loader this suite uses. A capture that could not be replayed fails there, while
the wearer is still wearing the trackers, rather than in a pull request weeks
later.

It will also warn — without failing — when a recording answers less than the
session probably intended: no IMU type reported, or Stay Aligned off. Both
produce a perfectly valid recording of leg behaviour and a useless one for
issue #3.

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
| `imu_type` | | which IMU, e.g. `BMI270`. **Required to replay Stay Aligned at all** — see below |
| `stay_aligned.<pose>.<field>` | | the relaxed pose in force at capture. **Also required to replay Stay Aligned** |

`<NAME>` is a `SkeletonConfigOffsets` constant. Any offset given is applied
before replay, so a recording of a person who is not 1.58 m tall is not solved
against default proportions. Offsets not given keep their defaults.

Required fields are checked at load. Leaving one at a template placeholder
(`TODO`, `TBD`, `unknown`, …) is rejected explicitly: an unfilled field reads as
provenance without being any.

`record-corpus` applies the same rules to what you type *before* it starts
recording, against the same list, so a session is never spent producing a
recording that will be rejected when someone tries to commit it.

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

### Two fields that decide whether Stay Aligned runs at all

Both optional in the schema and mandatory in practice for any recording captured
to exercise yaw correction. Both fail *silently* when absent: the replay
completes, every metric is produced, and the correction never ran.

`record-corpus` fills both from the running server, which is why it exists. The
mechanism is still worth knowing, because it is what a hand-edited sidecar gets
wrong.

**`imu_type`.** `TrackerFrames.toTracker()` builds a tracker with no IMU type
unless told one, `Tracker.isImu()` is then false, and `AdjustTrackerYaw.adjust`
returns on that before doing anything. The `.pfr` container does not store the
IMU type, so it can only come from here. Leave it out and Stay Aligned is inert
for the entire replay.

**`stay_aligned.<pose>.<field>`.** `RelaxedPose.forPose` returns null when the
config for the player's current posture is disabled, and `adjustMovingTracker`
returns on that null — so a standing, moving player gets no centring force. A
default config has every pose disabled, which is what a recording that says
nothing gets.

`<pose>` is `standing`, `sitting` or `flat`. `<field>` is `enabled`,
`upper_leg_deg`, `lower_leg_deg` or `foot_deg`. Declaring any of them turns Stay
Aligned on for the replay; the values should be whatever the wearer actually had
captured at the time, because a recording made to exercise Stay Aligned is
uninterpretable without them.

```ini
imu_type = BMI270

stay_aligned.standing.enabled       = true
stay_aligned.standing.upper_leg_deg = 3.5
stay_aligned.standing.lower_leg_deg = 1.0
```

These are the same class of gap as the missing sample rate, and worth the same
treatment: cheap to record at capture time, impossible to recover later.

## Building the dataset

The tooling is finished. What gates the corpus now is how many trackers are on a
body, and a handful of decisions that become irreversible the moment anything is
recorded.

### The recordings unlock at different tracker counts

The table below implies a full body, which makes the corpus look like an
all-or-nothing prospect. It is not. The open questions need different numbers,
and the cheapest ones need very few:

| trackers | what it unblocks |
| --- | --- |
| **1** | Sensor characterisation through `tools/fusion-bench` — noise floor, gyroscope scale, ODR error, `restThAcc`. No skeleton involved, so no wearer either. |
| **2**, on one leg | **Issue #3's head-to-head.** `KinematicHeading` solves relative heading across a *single* hinge, so a thigh and a shin are the whole requirement. |
| **5** — hip, both thighs, both shins | Issue #5's contact timing, and issue #4's segment deformation under real noise. |
| **7** — the above plus chest and head | Issue #6's jump case, and anything about whole-body pose. |

The single-tracker tier is not a consolation prize. A stationary desk capture is
enough to measure the noise floor and to check the shipped `restThAcc` against
it, and on the first device tried that margin turned out to be 4.5× the settled
minimum — a tuning question answered with one tracker and no wearer.

The two-tracker tier deserves particular attention. Issue #3 is the only route
to bounded yaw this pair of repositories can still take, since the magnetometer
path is closed as won't-validate on the firmware side, and it needs one hinge.

### Decide these before the first session, not after

Everything here is cheap now and awkward later. The list in *"What to do about
it at capture time"* below covers the per-session mechanics; these are the ones
that need settling once, in advance.

1. **Where `.imu` sidecars live** — repository, LFS, or release assets. Raw runs
   about 17 kB/s for eight trackers against ~35 kB/s for the fused `.pfr`, so it
   is cheaper than what is already being paid, but five five-minute sessions is
   still on the order of 75 MB. The `.pfr` and `.meta` pair belongs in git
   regardless: it is what the suite gates on.
2. **Decided: there will be no external position reference.** No lighthouse and
   no pressure mat. What that costs, and what to use instead, is
   *"No lighthouse: what that costs"* below — it is less than it sounds, and
   two of the substitutes are free.
3. **The consent wording**, agreed before someone is standing in a room waiting
   for it. `record-corpus` will not proceed without it, which is the point.

### No lighthouse: what that costs

Recorded as a decision rather than a constraint, because it changes what some
recordings can claim and nothing about whether they are worth making.

**Most of the board never needed one.** The distinction is between metrics that
are *internally consistent* and metrics that are *absolute*:

- **Issue #4's segment deformation** is measured against the skeleton's own bone
  lengths. Forward kinematics reads exactly `0.000000`, which is the metric's own
  calibration. An external reference would not make it more true.
- **Issue #3's yaw** is *relative* heading across a hinge, and the hinge
  constraint is physics rather than a sensor: two segments joined by one must
  agree about where its axis points, and their disagreement is the error. It is
  also a head-to-head between two methods, which is relative by construction.

So the one- and two-tracker tiers above are entirely unaffected, and those are
the tiers that unblock the most per tracker.

**Issues #5 and #6 are the ones that lose something.** Contact *timing* and
vertical centre-of-mass error through flight are both absolute quantities, and
an IMU-only rig knows neither.

**Do not substitute the offline labeller for ground truth.** It is the obvious
idea and it is wrong: measured against the same sequences, it scores F1 0.902
where a causal trailing-stillness rule scores 0.935. Using a reference that is
worse than the method under test bakes its error into every result and looks
like a finding.

Three substitutes, cheapest first:

1. **For #6, flight time gives the apex analytically.** A centre of mass that
   leaves and returns to the same height satisfies `h = g·t²/8`. Time takeoff and
   landing and the true apex follows, with no position sensor anywhere. This is
   not circular: a free centre of mass follows a ballistic arc whatever the
   estimator does, so it is a legitimate reference for whether the estimator
   reproduced it.

2. **For both, try the raw accelerometer before buying anything.** A heel strike
   is a sharp impact transient. Fused output smooths it; the raw counts in the
   `.imu` sidecar do not, and at 120 Hz accelerometer / 240 Hz gyroscope a
   transient resolves to roughly 8 ms — comfortably inside the 50 ms that issue
   #5 names as the damaging threshold.

   **Untested.** The sensor sits on the shin rather than the foot, so the impact
   arrives through the leg and how sharp the edge remains is an open question. It
   is answerable from a single walking capture, which is why it is first on the
   list: if it holds, it replaces the pressure mat outright, and it is a
   capability that did not exist before raw samples were recorded.

3. **Failing that, video at 240 fps** resolves touchdown and liftoff to ±4 ms.
   The hard part is synchronising it to the recording, not the frame rate.

**What to write down if none of them work.** A recording with no timing reference
still exercises the detector under real sensor noise, which synthetic motion
cannot. The comparison simply becomes relative — method A against method B —
rather than absolute. That is a weaker claim, and worth stating as such in the
recording's `description` so nobody later reads it as more.

### Capture so that recordings are comparable, not merely present

- **Start every recording with a still segment.** It is what a later reader
  estimates drift and bias against, and it costs seconds.
- **Keep everything short except the drift recordings.** Yaw drift needs minutes
  to develop; the leg corrections resolve in seconds, and a long recording
  buries the interval that matters.
- **For issue #3, the wearer's actual stance must differ from their captured
  relaxed pose.** That is the ordinary case and needs no choreography, but it is
  the whole discriminating power of the recording — measured on synthetic motion,
  where the stance *is* the configured pose, Stay Aligned removes essentially all
  injected drift and the comparison says nothing.
- **Record a `known-bad-*` whenever a real complaint can be reproduced.** Those
  are the hardest recordings to recreate later and the most valuable, because
  they become their own fix-verification.

### What a good first session looks like

Not "a file exists". The suite already fails on a recording that cannot be
loaded, and `record-corpus` already warns about the four ways a recording answers
less than it appears to — no IMU type, Stay Aligned disabled, no sample
timestamps, no raw samples.

The criterion worth aiming at is **a recording that replays and produces a
non-zero, non-degenerate residual.** Every `+legtweaks` metric currently reads
`0.000000` on synthetic input, which gates *"the corrections still work"* and
nothing finer. A graded number is the thing real data is for, and the first
recording that produces one is the point at which the corpus starts paying for
itself.

## What to capture, and why each one

The table above is the generic list from issue #1. The work on issues #3–#6 has
since produced specific requirements, because each of those comparisons turned
out to need something a recording either has or does not. Capturing with these
in mind means one session answers four questions instead of three sessions
answering none of them cleanly.

| recording | length | what it must contain | unblocks |
| --- | --- | --- | --- |
| `standing-still` | 2–5 min | no deliberate motion; the null case for drift | #3 |
| `walk-in-place` | 2–5 min | continuous knee flexion, feet clearly leaving the floor | #3, #4, #5 |
| `crouch` | 30 s | deep knee bend, feet planted throughout | #4 |
| `sit-to-stand` | 30 s | posture change, so the relaxed pose in force changes | #3 |
| `jump` | 30 s | several standing jumps, with a still moment before each | #6 |

**Length matters for #3 and nothing else.** Yaw drift needs minutes to develop
past the 5° threshold that issue's acceptance criterion names. The leg
corrections resolve within a few seconds, so a long recording buries the
interval that matters — keep everything else short.

**For #3, the recording must break Stay Aligned's assumption.** Its centring
force pulls each tracker toward the captured relaxed pose, and measured on
synthetic motion — where the player's stance *is* the configured relaxed pose —
it removes essentially all injected drift. That result says nothing about real
use, because the model was exactly right. The discriminating case is a wearer
whose actual standing posture differs from what they captured, which is the
ordinary case and needs no choreography. It does need `stay_aligned.*` recorded
accurately, so the difference is visible rather than assumed away.

**For #5, contact timing is the metric, not foot slide.** The existing
heuristics drive slide to exactly zero on clean input, so slide has no headroom.
Transition timing does. A recording is most useful here if something independent
establishes when the feet actually landed. There will be no pressure mat — see
*"No lighthouse: what that costs"* — so the candidate is the impact transient in
the raw accelerometer, which the `.imu` sidecar now carries. Without some such
reference a recording still exercises the detector under real noise, which
synthetic motion cannot do, but the comparison becomes relative rather than
absolute.

**For #6, the apex comes from flight time rather than from a position sensor.**
The metric is vertical centre-of-mass error through flight. There will be no
lighthouse, and none is needed: a centre of mass that leaves and returns to the
same height satisfies `h = g·t²/8`, so timing takeoff and landing gives the true
apex outright. What the recording must contain is therefore *clean transitions* —
a still moment before each jump, and no shuffling on landing — because the
timing is the measurement.

**For #4, the useful recordings are the ones where the corrections engage.**
`crouch` for floor clip, `walk-in-place` for skating. What is being measured
there is the deformation the corrections introduce, which is nonzero on
synthetic input already — so these recordings confirm the effect under real
noise rather than establishing it.

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

## What a recording bakes in, and how long it stays useful

Worth reading before a capture session, because most of what follows is decided
at capture time and cannot be changed afterwards.

### `.pfr` stores fused output, so the whole firmware signal chain is frozen

A frame holds a rotation, a position and an acceleration — the *output* of the
tracker's fusion filter. Everything upstream of that is baked in and not
recoverable:

- **VQF and its parameters.** Note that tuned `DefaultVQFParams` currently reach
  only MPU9250 (kmatzen/SlimeVR-Tracker-ESP#4); every softfusion IMU runs stock
  VQF defaults.
- **Rest detection and rest-gated bias correction**, including `restThAcc`,
  which sits near a measured cliff.
- **The sensor error model** — gyroscope scale, accelerometer bias and scale —
  applied on the tracker before fusion.
- **Online estimation**, which is recursive and evolves *during* the session.
- **FIFO and output data rate**: which samples reached fusion at all.

So a recording answers *"how does the server behave given this tracker
behaviour"*. It stays valid for that forever. What it cannot do is answer
*"how does the server behave given current tracker behaviour"* once the firmware
changes.

That distinction matters more than it sounds. A recording does not become
**wrong**, it becomes **unrepresentative** — and for regression-testing the
server, fixed-and-unrepresentative is exactly what a baseline wants. It is only
a problem for questions of the form *"is method A better than method B on real
data"*, where it matters that the data resembles what users have.

### Raw samples: what makes a recording firmware-independent

Raw gyroscope and accelerometer counts, captured alongside the fused output.
With them, any fusion configuration and any calibration can be re-run offline,
forever — the recording stops being tied to the firmware that made it.

`record-corpus` captures them automatically when the trackers support it,
writing a third file per sensor:

```
walk-in-place.pfr     the fused recording
walk-in-place.meta    how it was captured
walk-in-place.imu     raw pre-calibration samples
```

The `.imu` is written in `# slimevr-imu-log v1` — the format
`tools/fusion-bench` in the firmware repository already parses — so a
wirelessly captured log replays through the existing bench with no new tooling:

```sh
./build/fusion-bench run walk-in-place.imu
```

That has been verified end to end against a server-written file, including one
carrying gap markers.

**Needs firmware with raw sample streaming** (kmatzen/SlimeVR-Tracker-ESP#23).
Without it nothing streams, the recording is fused-only, and `record-corpus`
says so — that state is not recoverable afterwards.

#### Gaps are marked, never concatenated over

Losing a fused rotation packet is harmless; the next supersedes it. **Losing a
raw sample corrupts every re-fusion run downstream of it**, silently, because
the filter integrates a hole it cannot see.

So a `.imu` never joins across a loss. Two independent kinds are counted apart,
because they have different causes and different fixes:

- **dropped on tracker** — its buffer overran and it discarded samples rather
  than overwriting. It could not send fast enough.
- **lost in transit** — batches that never arrived. The network dropped them.

Both appear in the file header, and each hole gets a marker giving its *exact*
size:

```
# INCOMPLETE -- see gap markers below
# gyro: 304 samples in 2 run(s), 19 batches, 0 dropped on tracker, 3 lost in transit (1 batches)
# gap gyro missing=3 from_us=5000 to_us=20000
```

The count is exact rather than estimated because sample times are *nominal* —
accumulated from the configured sample period rather than read from a clock,
since the configured period is what the on-device fusion integrates. That makes
the timeline perfectly regular, so a gap's size is arithmetic.

Markers are `#` comments, which the bench's parser skips, so a holed capture
still loads — as two shorter captures with a documented hole between them,
rather than as one continuous capture that silently jumps.

### What the raw file still does not fix



Raw IMU samples, captured alongside the fused output. With raw gyro and
accelerometer counts you can re-run any fusion configuration, and any
calibration, offline and forever.

The firmware can already log them, and in the right place — `RawSampleLogger` is
called *before* `calibrator.scaleAccelSample`, so a capture holds what the sensor
produced rather than what the current calibration made of it. But as it stands it
is `#ifdef RAW_SAMPLE_LOGGING`, writes over `Serial`, and covers one sensor at a
time, so it cannot be used during a wireless multi-tracker session. Joining the
two is the unbuilt half of kmatzen/SlimeVR-Tracker-ESP#3 ↔ #1.

**Until that exists, every recording here is tied to the firmware that made it.**

### What to do about it at capture time

Since the recordings cannot yet be made firmware-independent, make them
*interpretable* instead. A recording whose exact conditions are known can at
least be reasoned about later; one whose conditions are unknown gets deleted.

1. **Calibrate every tracker first, and finish.** Gyroscope scale is a manual
   loop — capture, run `tools/fusion-bench gyro-scale` on a host, then
   `SET GYROSCALE` over serial. Recording before that bakes in an uncorrected
   scale error that no later firmware fixes.
2. **Record the firmware build precisely.** `firmware` is filled from what the
   tracker reports, and `scripts/get_git_commit.py` embeds the *branch name*, so
   put the commit in `notes` as well.
3. **Note the calibration values in force** in `notes`, and whether online
   estimation was running. Otherwise the tracker's behaviour drifts within the
   recording and nothing says so.
4. **Prefer boards that build the full calibration flow.**
   `GUIDED_ACCEL_CALIBRATION` is compiled out on `BOARD_GLOVE_IMU_SLIMEVR_DEV`
   for want of flash, so captures from it are not comparable with the rest.
5. **Include a still segment** at the start of every recording. It gives a later
   reader something to estimate drift and bias against.

## Sample timestamps

Each frame optionally carries the instant its sample was taken, in the server's
timebase (`TrackerFrameData.SAMPLE_TIMESTAMP`). It is written whenever the
tracker's firmware reports sample times, and omitted otherwise, so a recording
from older firmware is byte-identical to what it would have been before the field
existed.

It is here for the reason the rest of this file keeps repeating. The recorder
ticks uniformly; real trackers do not report uniformly. The *spread* between
trackers at one instant is the entire quantity `TimeAlignment` exists to remove,
and it is not recoverable from a uniform frame rate. Without the field, every
replayed tracker had an empty sample history, fewer than two were eligible, and
the alignment pass returned having touched nothing — on every recording, on every
run.

Unlike `imu_type` and `stay_aligned.*`, this one could not have been repaired
with a sidecar field afterwards, because the data was never written down. Hence
adding it before the corpus exists rather than after.

Absolute values are the capture machine's clock and carry no meaning on replay;
`TrackerFramesPlayer` rebases them to the recording's earliest sample. Only the
differences matter, and rebasing preserves both of the ones that do — the spread
between trackers, and the interval between one tracker's samples.

`record-corpus` warns when a capture produced no timestamps, for the same reason
it warns about a missing IMU type.

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
