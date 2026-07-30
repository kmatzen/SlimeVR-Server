# Measuring Stay Aligned

Issue #3 has one piece of work left: decide, empirically, whether the kinematic
heading solve is better than Stay Aligned. The estimator (#12), the global solve
(#13) and the shadow integration (#18) are all merged. What remains is the
comparison, and it has been waiting on a recording corpus (#15).

**A corpus would not have unblocked it.** Stay Aligned could not be run in the
replay suite at all, for reasons that have nothing to do with what is being
replayed.

## The blocker

```kotlin
val yawCorrection =
    yawCorrectionPerSec *
        VRServer.instance.fpsTimer.timePerFrame *   // lateinit global
        numTrackers.toFloat()
```

`VRServer.instance` is `lateinit`, so reading it without a server throws. Every
replay harness constructs a `HumanPoseManager` directly. Enabling Stay Aligned in
any of them crashed on the first corrected frame.

That is why the whole `stayaligned` package had no test coverage, while the
estimator proposed to replace it has four test classes — the asymmetry was not a
judgement about which mattered, it was that one of them could be instantiated in
a test and the other could not.

Replaying a `.pfr` recording drives the same code path and hits the same global.
So the comparison #3 is waiting for was blocked on this, not on #15.

Fixed the same way #16 fixed `LegTweaks`: the frame interval is now injectable
and defaults to the server's timer, so production behaviour is unchanged.

### And a second one, which a corpus also would not have fixed

`StayAligned` is an `object` and its round-robin cursor is a field on it. Which
tracker gets adjusted first therefore depends on how many ticks any earlier
skeleton ran in the same JVM, so two replays of identical input diverge. That is
the property a regression baseline cannot be built on, and it is the same class
of problem the injected clock in #16 exists to solve. `StayAligned.reset()` now
clears it.

## Two silent no-ops worth knowing about

Both were found by trying to drive Stay Aligned and getting a corrected run
byte-identical to the uncorrected one.

**A tracker with no declared IMU type gets no yaw correction at all.**
`AdjustTrackerYaw.adjust` returns immediately unless `tracker.isImu()`, which
requires `imuType != null`. Nothing logs this.

**A standing, moving player gets no correction unless a relaxed pose has been
captured and enabled.** `RelaxedPose.forPose` returns null when the config for
the current posture is disabled, and `adjustMovingTracker` returns on that null.
The locked-tracker path still works at rest, so the failure is partial and
posture-dependent — which is harder to notice than a total one.

Neither is a defect exactly. Both are cases where the correct behaviour is to do
nothing and the system does nothing. But they are indistinguishable from
"Stay Aligned is running and finds nothing wrong", and that is worth being able
to tell apart.

## The first numbers

Injected yaw drift of 0.05 °/s into one calf tracker, `walk-in-place`, five
minutes at 100 Hz. Ground truth is exact because the drift was injected. The
metric is #3's own: relative heading error between thigh and calf, and the time
until it exceeds 5°.

| | final error | max error | time to 5° |
| --- | --- | --- | --- |
| uncorrected | +14.987° | 27.610° | 52.3 s |
| **Stay Aligned** | **+0.006°** | **0.021°** | **never** |

Stay Aligned removes essentially all of it. That is the first measurement of this
subsystem, and it is a good deal better than "it works, but it is a heuristic"
suggests.

## Why that does not settle the comparison

It looks decisive until you ask why it worked. Stay Aligned pulls each tracker
towards a captured relaxed pose, and in this sequence the player's true stance
*is* the configured relaxed pose. Its model of the answer is exactly right, so it
cannot be wrong.

That is the same flattery the kinematic method gets from the same input, pointed
the other way. Synthetic motion gives it exact hinges and continuous
observability, where a real knee is an approximate hinge that is only
intermittently flexed.

**Synthetic motion satisfies both methods' core assumptions perfectly.** Neither
is stressed, so a head-to-head on this input cannot discriminate between them
whatever the numbers say.

The obvious probe — mis-specify the relaxed pose and see what breaks — was run,
at 15° of error with no drift injected:

| | max manufactured error |
| --- | --- |
| relaxed pose correct | 0.000° |
| relaxed pose wrong by 15° | 0.081° |

0.08°. Stay Aligned is substantially more robust to a wrong relaxed pose than
this issue's framing implies, at least on symmetric motion where the two legs'
errors balance.

## What this means for issue #3

- **The infrastructure blocker is gone**, and it was a real one that would have
  survived #15.
- **Stay Aligned has a number for the first time.** It fully corrects injected
  drift when its pose model is right, and degrades gracefully when it is not.
- **#15 remains a genuine prerequisite for the decision**, but for a
  better-understood reason. It is not that synthetic sequences are too short —
  they are closed-form in time, so length is free, and this one runs five
  minutes. It is that synthetic motion cannot make either method's assumption
  false in a realistic way. The discriminating case needs a real knee that is not
  quite a hinge, and a real player who is not quite in their relaxed pose, at the
  same time.
- The comparison harness now exists and takes a recording the moment there is
  one.

## What this does not do

- It does not run the kinematic solve as a *corrector*. That integration
  (point 3 of the original plan) is still not done, deliberately — the shadow
  reports its residual alongside Stay Aligned's figure, and swapping it in before
  there is evidence would be replacing one unmeasured method with another.
- It does not measure either method under orientation noise.
- It does not address the cross-limb observability limit recorded on #18: hinges
  exist only at knees and elbows, so the joint graph is a set of per-limb
  components and no whole-body heading datum is observable from constraints
  alone.
