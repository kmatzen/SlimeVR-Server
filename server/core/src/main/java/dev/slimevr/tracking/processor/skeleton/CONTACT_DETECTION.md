# Foot-contact detection: measurement and labelling

Groundwork for issue #5, which proposes replacing the contact thresholds in
`LegTweaksBuffer.checkState()` with a learned classifier.

**No classifier is added here, and none should be until there are real
recordings.** What is added is the two things that have to exist before one can
be trained or believed: a way to score a contact detector against ground truth,
and a way to produce labels. Both were missing, and measuring with them changed
what issue #5 should probably do next.

## What was missing

Nothing in the repository had ever measured contact detection. Not because it
was overlooked but because there was nothing to measure against: contact labels
did not exist for any input.

`SyntheticMotion` now carries them. They are ground truth in the strong sense —
these sequences are *defined* by closed-form joint-angle functions, so which
foot is planted is a property of the definition rather than something inferred
from a signal. Nothing derives them from foot height or velocity. If they were
derived, scoring a detector against them would be scoring it against a slightly
different detector.

## Why this can be measured without #15

Issue #5 says real recordings are a hard prerequisite, because on synthetic
motion the existing heuristics drive foot slide to exactly zero and there is
nothing to beat.

That is true **of foot slide**, which is why slide is not the metric here.
Contact detection has its own error and it is not zero on clean geometry. A foot
can be declared planted a dozen frames after it left the floor while slide stays
at zero throughout — slide measures where the foot ends up, this measures when
the decision was made. Different quantities, and the second is measurable today.

This does not retire #15. A synthetic sequence has no IMU noise, no yaw drift and
no mounting error, and the caveat at the bottom of this page is load-bearing.

## Metrics

Two families, because the second is the one the issue cares about.

**Frame-wise** — precision, recall, F1 — asks how often the decision was right.
It is dominated by the long stretches where the foot is obviously planted or
obviously airborne, which everything gets right.

**Transition timing** asks *when* the method noticed. Issue #5 puts this first
among its failure modes:

> Detect the contact but get its *timing* wrong by 50 ms → error injected at
> exactly the moment of highest acceleration

A detector that is right 98% of the time but consistently late is worse, for
translation, than one right 96% of the time with no lag — the frames it gets
wrong are the ones where the foot is accelerating hardest. Frame-wise scoring
cannot see that difference.

## The baseline

`LegTweaksBuffer.checkState()`, running inside a real pipeline with its
hysteresis and sensitivity scalars intact — not a reimplementation. The audit on
issue #5 is explicit that this matters:

> The baseline to beat is a hysteretic state machine, not a memoryless
> threshold. […] The comparison needs to be against the hysteretic behaviour, or
> the model will look better than it is.

On `walk-in-place`, 6 s at 100 Hz, six liftoffs per foot:

| | precision | recall | F1 | mean abs. transition lag |
| --- | --- | --- | --- | --- |
| hysteretic thresholds | 0.667 | 0.997 | 0.799 | 12.5 frames (125 ms) |

Two things stand out.

**Recall 0.997, precision 0.667.** The detector almost never misses a real
contact and calls contact on about a third more frames than are real. It is not
noisy — it is systematically biased toward "planted", holding the lock well past
liftoff. That is issue #5's *"falsely detect a contact → the body is anchored to
a moving foot → the world lurches"*, and it is the dominant error.

**125 ms of mean transition error**, against the 50 ms the issue names as
damaging. Worst single transition is 13 frames. On clean geometry, with no noise
to excuse it.

For the sequences where both feet stay down throughout — `stand`, `squat`,
`lean` — the detector scores F1 0.999 and produces no liftoffs. The failure is
specific to feet that actually move.

## The offline labeller

`OfflineContactLabeller` implements approach (2) from issue #5's training-data
section: label a recording after the fact, using frames the runtime detector
could not have seen. It is not a detector and cannot run live — a test pins that
by checking that two trajectories identical up to frame *N* get different labels
before frame *N*.

The rule is deliberately simple: a foot is in contact when nothing in a window
around the frame is more than `stillnessRadiusM` from where the foot is now, and
the foot is near the floor; then contact and flight runs shorter than
`minSegmentFrames` are removed.

## The finding that was not expected

The offline labeller beats the heuristic — F1 0.902 against 0.799, mean lag 5.5
frames against 12.5. Which looks like a clean confirmation of the issue's
premise:

> An offline labeller can be far more accurate than a real-time detector because
> it can look ahead — that asymmetry is the whole trick.

But the labeller changed **two** things at once. It looks ahead, *and* it
replaced five thresholded signals plus hysteresis with a single question: did
this foot stay within 2 cm of where it is now?

Running the same stillness rule over a **trailing** window of the same total
width separates them. A trailing window is causal — it could run live — so
whatever it recovers needs no lookahead and no learning:

| | F1 | mean abs. transition lag |
| --- | --- | --- |
| hysteretic thresholds (causal) | 0.799 | 12.48 frames |
| **stillness, trailing (causal)** | **0.935** | **3.62 frames** |
| stillness, centred (offline) | 0.902 | 5.48 frames |

**Lookahead contributes nothing here. The rule change contributes everything.**
The causal trailing window is the best of the three, beating even the offline
labeller that can see the future.

The mechanism is not mysterious. Both windows refuse to call contact until the
whole window is still, so both shorten every contact interval by about the same
amount; what differs is where the shortening lands. Trailing puts all of it at
touchdown and none at liftoff — the instant the foot moves, the window
containing that motion is the current one. Centred splits it across both ends, so
it reports contact late *and* drops it early, and the early drop is a transition
error the trailing rule never makes.

## What this suggests for issue #5

- **There may be a large win available with no learning at all.** F1 0.799 →
  0.935 and 125 ms → 36 ms of transition lag, from replacing a five-signal
  hysteretic test with one stillness radius. If that survives contact with real
  data it is a much cheaper change than a classifier, and it is the thing a
  classifier would have to beat.
- **The training-data justification is weaker than stated.** Offline labels here
  are worse than a causal rule, so labelling the corpus with lookahead and
  training on the result could teach a classifier to reproduce transition errors
  that a simpler causal rule does not make. The labeller is still the right tool
  for producing labels from recordings — but "it can look ahead" is not, on this
  evidence, why it would be worth doing.
- **The baseline is now a number**, so any classifier proposed for this can be
  argued about with evidence.

## The caveat, which is large

All of the above is one synthetic sequence with no IMU noise, no yaw drift and no
mounting error.

The hysteresis and the five signals exist precisely to cope with those. A bare
stillness radius has no defence against noise — noise inflates apparent
displacement, which is exactly the quantity it thresholds on — so it is entirely
possible that the ordering reverses on real recordings, and that what looks like
50 redundant constants is 50 constants earning their keep. Nothing here should be
read as "the thresholds are bad"; what it says is "on clean geometry the
thresholds cost 125 ms of timing, and a simpler rule does not".

That is why #15 remains on the critical path for this issue, and why the
trailing-stillness result is written up rather than shipped as a detector.

The ordering is asserted in `ContactDetectionTest`, so if it ever reverses the
conclusion recorded here fails loudly rather than going quietly stale.
