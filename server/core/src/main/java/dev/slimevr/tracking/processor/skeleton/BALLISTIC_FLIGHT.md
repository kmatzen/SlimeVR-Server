# Ballistic flight

Predicts the centre of mass along a ballistic arc while both feet are off the
floor, instead of integrating the torso accelerometer through it.

Off by default. Set `localizer.useBallisticFlight = true` to use it. Issue #6
asks for the existing path to stay available for comparison, and until the two
have been compared on real recordings the existing one is what users have been
getting.

**Scope.** `Localizer.update()` returns on its first line when the head tracker
has a position, so none of this is in the path for anyone wearing a positional
headset. It affects standalone and HMD-less setups only.

## What is actually wrong with the vertical channel

Issue #6 says jumping is unreliable because "during flight there is no contact,
so the anchor is gone entirely and there is nothing to integrate against". The
audit on the issue already corrected that — there *is* a `FOLLOW_COM` path, and
it is bad rather than absent. Measuring it narrows the diagnosis considerably
further, and the result is not what either description predicts.

Replaying a synthetic jump with a true apex of **0.184 m**, the existing path
estimates an apex of **0.021 m**. The body essentially does not rise at all.

That is not accumulated drift, and it is not accelerometer noise — the input is
noise-free. Two mechanisms suppress upward vertical velocity, and the
measurements separate them:

- **The anti-flyaway clamp** in `getCOMVelocity` discards upward CoM
  acceleration until the feet have been the world reference for `WARMUP_FRAMES`
  consecutive frames. `footFrames` resets whenever the reference is anything
  else, so a jump's own flight phase clears it and a second jump cannot have
  re-earned the release.

- **The floor clamp** in `updateTargetCOM` sets vertical CoM velocity to zero
  outright on every frame the lowest tracker is at or below the floor.

The floor clamp is the one that matters. Give the existing path a lead-in long
enough to release the anti-flyaway clamp before the crouch begins and the apex
is *still* 0.021 m. Meanwhile the kinematic launch measurement reads the centre
of mass rising at 1.3 m/s at the moment of takeoff, so the rise is present in
the pose and is being discarded downstream.

The reason is structural. Every frame of a push-off has a foot on the floor —
that is what a push-off is — so every frame of a push-off is a frame the floor
clamp fires on. **A body cannot acquire upward velocity through a channel that
is reset to zero whenever a foot is touching the ground.** No amount of standing
still first changes it, which is why two consecutive jumps come out identical to
the last digit.

So the vertical channel was never integrating a jump badly. It was structurally
incapable of representing one.

## The arc

While a foot is planted it is a zero-velocity anchor and translation follows
from it. In flight there is no anchor and the only remaining observation is the
accelerometer, which has to be integrated twice, so bias grows quadratically
with nothing to correct against.

But flight is the one phase where the answer is known in advance. The only force
on an airborne body is gravity, so the centre of mass follows a parabola whose
shape is fixed and whose only free parameters are the position and velocity it
launched with:

```
p(t) = p₀ + v₀·t + ½·g·t²
```

That turns "integrate a noisy signal for half a second" into "measure two
vectors at one instant", and that instant is takeoff — the moment a foot was
still planted and the measurement was at its most reliable.

The accelerometer is not read at all during flight. Neither is
`CONSTANT_ACCELERATION`, nor the anti-flyaway clamp, nor the floor clamp's
velocity reset. The parabola replaces all four.

### It extrapolates; it does not fit

Issue #6 notes that a ballistic *fit* needs the flight phase to be over before
it can be fit — fine offline, useless for live VR — and asks for "a causal
approximation, fit incrementally and refined".

There is nothing to refine against. During flight the body is unobserved apart
from the accelerometer this deliberately ignores, so there is no mid-flight
measurement that could update the arc. What is implemented is therefore a pure
extrapolation from the takeoff measurement: zero latency, and wrong by however
wrong that one measurement was.

The honest improvement is not a better fit, it is a better takeoff measurement.
See *Where the remaining error is*.

### Measuring the launch velocity

The one number the whole arc depends on, and a straight bias-versus-noise trade.
A window of *W* seconds ending at takeoff reports roughly the velocity at *W/2*
before takeoff, so the bias it costs is proportional to the acceleration in that
axis — and the two axes are nothing alike at takeoff.

`getTakeoffVelocity` therefore uses a different window per axis:

| axis | acceleration at takeoff | window | why |
| --- | --- | --- | --- |
| vertical | ~25 m/s², the largest in the motion | 1 frame | any wider reads far below the launch speed |
| horizontal | small | the existing 100 ms | short window is dominated by the legs swinging |

The existing code's own `VELOCITY_SAMPLE_RATE` is 100 ms — despite the comment
next to it saying 10 ms — and using it for the vertical channel would launch
every arc at roughly half the true speed.

Applying the short window to *all* axes was measured and is worse: horizontal
drift rose to 0.70 m, because the short horizontal reading is contaminated by
limb motion and the arc extrapolates that contamination linearly across the
whole flight.

### Guards

- **Implausibly long flight.** VR users lean on furniture, kneel, sit on the
  floor and hold onto things, and contact detection reporting no planted foot is
  not the same claim as the body being airborne. Past
  `ballisticMaxFlightSec` (0.9 s, a 1.0 m rise — beyond standing-jump range for
  essentially everyone) the arc is abandoned and the frame falls back to the old
  path. Latched until the feet return: without the latch, abandoning clears
  `inFlight`, the next airborne frame launches a fresh arc, and the guard trips
  again, producing a relaunch cycle that is neither the arc nor the fallback.

- **The floor.** The floor is a hard constraint and the arc is a prediction, so
  when they disagree the arc gives way and flight ends. This matters more than
  it looks: the arc recomputes `targetCOM` from its launch state every frame, so
  without this it would discard the floor correction on the next frame and the
  floor would silently stop being enforced for the whole of flight.

- **Reset.** A reset teleports the skeleton to the origin, so an arc launched
  before it describes a body that no longer exists.

## Validation

There are no recordings in the repository (issue #15), and issue #6 lists
jumping recordings as a prerequisite. Those are needed for the sensor-facing
half of the question and cannot be substituted for. But a synthetic jump can
answer the method-facing half exactly, and does it in a way a recording never
will.

`SyntheticMotion`'s `jump` is defined as a closed-form height profile, and the
acceleration handed to the torso tracker is that profile's **analytic second
derivative**. Input and ground truth are the same object differentiated twice,
so an estimator given the acceleration and asked for the height is inverting an
operation with no noise in it. Residual is method error with nothing else mixed
in.

`theGroundTruthIsSelfConsistent` pins that relationship by integrating the
acceleration twice and checking it reproduces the profile — worst error 4 µm. If
the two ever drift apart, every number below is measured against a trajectory
the input does not describe.

### Results

One jump, 100 Hz, true apex 0.184 m, true flight 0.387 s. The lead-in is long
enough to release the anti-flyaway clamp, which is the existing path's best
case:

| | apex | apex error | max vertical error in flight | flight frames detected |
| --- | --- | --- | --- | --- |
| accelerometer integration | 0.021 m | −0.163 m | 0.473 m | 21 / 39 |
| **ballistic arc** | **0.089 m** | **−0.096 m** | **0.172 m** | **34 / 39** |
| truth | 0.184 m | | | |

Vertical error through flight falls by **2.8×**. Flight is detected across 34 of
39 frames rather than 21, which is a consequence rather than a separate change:
getting the height right lifts the feet, and lifted feet read as off the floor.

### Horizontal is reported, not claimed

Total horizontal drift *rises*, 0.166 m → 0.234 m, and that needs stating rather
than burying.

It is not attributable as error. The jump's *trajectory* is purely vertical but
its *pose* is not — the legs bend in the sagittal plane, which genuinely moves
the mass-weighted centre of mass fore and aft — so there is no clean horizontal
ground truth on this sequence.

The rise is also explainable without any horizontal change: a planted foot is
what pins horizontal translation, and the old path's smaller total was bought by
believing the feet were down for 18 of 39 flight frames. Comparing drift *rate*
per unanchored frame removes that advantage:

| | horizontal drift rate |
| --- | --- |
| accelerometer integration | 0.3638 m/s |
| ballistic arc | 0.3644 m/s |

Indistinguishable. The claim is "not made worse", which is what this can
support.

## Where the remaining error is

The arc recovers 0.089 m of a 0.184 m apex, so it is still short. The cause is
identified rather than assumed, and it is not the parabola.

Apex goes as v₀², and the measured launch speed is **1.305 m/s** against a true
**1.900 m/s** — a ratio of 0.687, whose square is 0.472, against the measured
apex ratio of 0.481. The entire remaining error is the launch measurement.

Most of that is detection latency. Takeoff is recognised at t = 0.630 s against
a true 0.600 s, and gravity has already taken 0.29 m/s off the launch speed in
those three frames. The rest is the CoM itself, derived from approximate body
proportions through a servo that lags by a frame.

The arc reports this itself, with no ground truth involved:
`theArcReportsFinishingLowBecauseTheLaunchIsMeasuredLate` checks the sign and the
mechanism. A jump ends on the floor it started from, so the gap between where the
arc finished and where the body was observed to land measures how wrong the
launch velocity was — measured, the arc finishes **0.062 m low** and real flight
outlasts the arc's prediction by **0.074 s**. Both are the signature of an
under-measured launch, and both are available in a deployment with no
instrumentation.

The fix is a better takeoff instant, which is exactly what a learned contact
classifier (issue #5) provides: it says *when* takeoff happened, and with what
confidence, instead of leaving it to a threshold that is least reliable at that
transition. Issue #6 already lists it as a prerequisite; this quantifies what it
would be worth.

## What this does not do

- **Proposal 2, ground-reaction-force plausibility.** Not attempted. Worth
  recording what was found while nearby: `LegTweaksBuffer` already has a GRF
  path — `getPressurePrediction`, `findForceVectors`, `detectOutsideForces` —
  and it compares `centerOfMassAcceleration` against `GRAVITY`. But
  `centerOfMassAcceleration` is a per-frame *delta of a delta*, never divided by
  the timestep, so it is in metres and is being compared against 9.81 m/s². At
  100 Hz the gravity term outweighs it by orders of magnitude, which makes
  `detectOutsideForces` very nearly a test of whether the foot force vectors
  cancel gravity, independent of how the body is actually accelerating. That is
  a unit mismatch in existing code and is left alone here — it is proposal 2's
  territory and wants its own measurement.

- **Proposal 3, full physics-based optimisation.** Issue #6 puts it inside the
  joint-estimator proposal (#4), which is where it belongs.

- **Retire `CONSTANT_ACCELERATION`, the anti-flyaway clamp, or the floor clamp's
  velocity reset.** The arc bypasses all of them in flight, but they still run
  on the ground, where they are entangled with each other and with walking. That
  is a separate change and wants the recordings.

- **Compare against the existing path on real data.** Needs issue #15. What is
  here is a comparison on input whose answer is known exactly, which bounds the
  method; a recording bounds the method plus the sensor.
