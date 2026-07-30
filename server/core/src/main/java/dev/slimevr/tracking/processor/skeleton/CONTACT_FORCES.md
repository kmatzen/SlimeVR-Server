# Contact-force plausibility

Issue #6 proposal (2): reject or damp translation estimates that would require
forces the contacts cannot supply. Off by default behind
`localizer.useContactForceLimits`.

Like everything in `Localizer`, this only affects setups with **no positional
head tracking**. `Localizer.update()` returns on its first line when the head
tracker has a position, so for a user in a headset none of this is in the path.

## The constraint

The whole body's centre of mass obeys `m·a = Σ F_external` exactly. Swinging an
arm or tucking the legs moves the CoM *within* the body, but the internal forces
that do it cancel in pairs and contribute nothing to the sum. So with the feet
as the only contacts, the CoM acceleration is fixed by gravity plus what the
floor pushes with, and by nothing else.

Writing the required contact force per unit mass as `f = a − g`:

```
f_y ≥ 0                     the floor pushes up, it does not pull down
√(f_x² + f_z²) ≤ μ · f_y    friction cannot exceed μ times the normal load
```

which is a second-order cone. Worth writing it this way rather than as two
rules, because the airborne case then needs no special handling: with nothing on
the floor the available normal load is zero, the cone collapses to the single
point `f = 0`, and the constraint says the body accelerates at exactly `g` in
every axis. *"A body with no foot on the ground cannot accelerate sideways"* is
not an extra rule; it is what the cone already says.

`ContactForceLimit` implements the full cone — `isPlausible`, `project`. What
`Localizer` actually applies is `limitHorizontal`, a restriction of it, for the
reason in the next section.

## Why only the horizontal channel is constrained

The cone couples the two channels: how much sideways force is available depends
on how hard the floor is being pushed. That coupling is only meaningful if the
vertical acceleration fed into it is a measurement of the body. In `Localizer`
it is not. The vertical channel is a torso accelerometer plus a constant
downward `CONSTANT_ACCELERATION` fudge, deliberately unphysical, whose job is to
stop the skeleton drifting upward. Sizing a physics constraint from a number that
exists to compensate for the absence of physics gives the constraint no meaning.

So the vertical load comes from the contact state instead, which is an
independent measurement, and the vertical channel is left alone for the
ballistic arc (`BALLISTIC_FLIGHT.md`) to own. Measured: enabling this moves the
ballistic apex by less than 0.1 mm.

The contact state is read from the previous frame's `worldReference`, not from
`isFootOnGround()`. That function is a bare inequality on foot height against a
calibrated floor, and floor clipping routinely leaves a planted foot a few
millimetres above it, so a standing body reads as intermittently unsupported —
and is then told it cannot be holding itself up.

## What it does

Synthetic motion at 100 Hz, three seconds after a 1.5 s lead-in.

| sequence | frames corrected |
| --- | --- |
| stand | 0 / 300 |
| squat | 0 / 300 |
| walk-in-place | 0 / 300 |
| lean | 0 / 300 |
| jump | 42 / 300 |

Silent on everything but the jump, and on the jump it fires only while airborne.
That is the shape a plausibility gate should have: a constraint that fires on
most frames of ordinary motion is not enforcing physics, it is low-pass
filtering, and it would be better described and tuned as one.

Horizontal drift over the frames where nothing anchors the estimate:

| | unanchored drift rate | unanchored frames |
| --- | --- | --- |
| without | 0.3826 m/s | 55 |
| **with** | **0.3291 m/s** | 55 |

14% less drift, over an identical frame count. Measuring per unanchored frame
rather than as a total is deliberate — a planted foot pins horizontal
translation, so a configuration that wrongly believes the feet are down reports
less total drift for that reason alone. The frame counts being equal is what
makes this comparison like-for-like.

## Why this is not built on the existing force machinery

`LegTweaksBuffer` already has ground-reaction-force code — `getPressurePrediction`,
`findForceVectors`, `detectOutsideForces` — and the audit on #6 pointed at it,
noting that it compares a centre-of-mass term against `GRAVITY` without ever
dividing by the timestep. That is accurate. The obvious conclusion, that
dividing by the timestep repairs the test, is wrong.

The fields are now named `centerOfMassDelta` and `centerOfMassDeltaChange`
rather than `...Velocity` and `...Acceleration`, because metres is what they
are. At 100 Hz they are 1e-2 and 1e-4 times the corresponding rates, so a real
25 m/s² push-off reaches `detectOutsideForces` as 2.5e-3 next to a gravity term
of 9.81. The acceleration term is four orders of magnitude too small to affect
any comparison it appears in. What those functions actually test is whether the
two foot force vectors happen to cancel gravity — a question about where the
feet are relative to the centre of mass. **They read as a dynamics test and
behave as a geometry test.**

Dividing by the timestep does not fix that, because `FORCE_ERROR_TOLERANCE_SQR`
is 4 m/s² and a second difference of a mass-weighted sum of eight segment
positions blows straight through it. Correctly scaled, on **noise-free**
synthetic motion:

| sequence | p50 | p99 | max |
| --- | --- | --- | --- |
| stand | 0.00 | 0.00 | 0.00 |
| squat | 1.93 | 39.01 | 42.24 |
| walk-in-place | 1.38 | 125.06 | 125.06 |
| lean | 0.10 | 15.17 | 16.36 |
| jump | 0.00 | 351.18 | 692.97 |

m/s². The spikes are phase boundaries in the profile and leg-correction snaps,
not motion. Real sensor noise only makes it worse: a second difference amplifies
position noise by `1/dt²`, which is 1e4 at this rate, so a millimetre of jitter
is 10 m/s² by itself.

So the scale error is load-bearing. Making that one-line change and running the
suite: `detectOutsideForces` fires on nearly every moving frame, `isStanding`
drops, the foot-lock pressure scalars collapse to their fallback, and flight
detection in `Localizer` goes from 34/39 frames to **0/39**. Walk-in-place foot
slide goes from 0 to 8 mm/s.

The tolerance was calibrated against a quantity 1e-4 of its nominal value, and
no tolerance makes that test both meaningful and non-disruptive while its input
is a raw second difference. Using CoM acceleration dynamically needs a smoothed
estimate first — which changes how foot locking behaves, and belongs with
whoever owns that (#4, #5), not with this issue. `ContactForceLimit` therefore
derives its own acceleration by differencing `Localizer`'s already-windowed
`comVelocity`, which is smooth enough to use.

`ContactForceReplayTest.theRawSecondDifferenceIsTooNoisyToUseAsADynamicsSignal`
pins this, so if it ever stops being true the fix becomes available.

## What this cannot see

Every external force that is not the floor: leaning on a desk, a hand on a wall,
sitting on furniture, holding a rail. Issue #6 raises exactly this, and it is
why the gate damps rather than rejects and why it is off by default. A user
pushing off a wall genuinely accelerates outside the foot-contact cone, and the
honest reading of that is not "the estimate is wrong" but "the contact model is
incomplete".

Whether the drift this buys is worth that risk cannot be settled on synthetic
motion, because synthetic motion contains no furniture. It needs recordings of
people bracing, kneeling and sitting — #15.

## Proposal (3)

Not attempted, and deliberately. Issue #6 says of it:

> Realistically this only makes sense as part of the joint-estimator proposal,
> where the physics terms become additional residuals in an objective that
> already exists, rather than a separate system.

That objective does not exist. `JOINT_SPACE.md` opens by saying so in as many
words — the joint-space work from #27 is groundwork for #4 and explicitly not
the sliding-window estimator. Building a standalone real-time dynamics solver is
the alternative the issue rules out in the same sentence, so proposal (3) stays
open, blocked on #4 rather than on #15.
