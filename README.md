# MachineVision toolchain


https://github.com/user-attachments/assets/e9e0e93a-b026-40aa-af4a-73a470dc6e0d



# RoboCup field localisation (`nubots/`)

A recursive Bayesian field localiser for a RoboCup humanoid, built on a
square-root information Gaussian framework. It replaces the NUbots 
per-frame NLopt point estimate with a proper filter that carries calibrated
uncertainty, and it beats that baseline on both ground-truthed recordings:

| recording | truth | ours | NUbots baseline |
|---|---|---|---|
| `data2` (lab, OptiTrack) | mocap, 2681 samples | **0.093 m / 5.41°** | 0.506 m / 11.33° |
| `data4_webots` (sim, two real falls) | simulator, 971 shared samples | **0.115 m / 3.82°** | 0.265 m / 5.07° |

Over all 1779 of its own frames — including both eight-second falls, where the
baseline reports nothing — `data4_webots` comes to 0.135 m / 8.55°. Updates cost
~1.0 ms each on a laptop CPU (max 3.2 ms on `data2`).

## State and belief representation

The state is 18-dimensional (`SystemLocalisation.h`):

| indices | meaning |
|---------|---------|
| 0–2     | torso position `rBFf` in the field frame `{f}` [m] |
| 3–6     | attitude quaternion `q` (w, x, y, z), `Rfb = quat2rot(q)` |
| 7–9     | body-fixed linear velocity `vBb` [m/s] |
| 10–12   | body-fixed angular velocity `omegaBb` [rad/s] |
| 13–15   | gyroscope bias `bGyro` in `{b}` [rad/s] |
| 16–17   | camera-mount attitude bias (roll, pitch) [rad] |

Pose in `{f}`, velocity in `{b}`: the Fossen vehicle-dynamics convention
`(eta, nu)` rather than the strapdown-INS one. The velocity states are a random
walk, and "the robot keeps walking forward at this speed" is a far better model
of a turning robot than "its field-frame velocity vector is constant" — the
latter needs a PSD sized for the turn even when walking straight. Every velocity
measurement also arrives body-fixed (the gyroscope directly, the walk odometry as
a body-frame finite difference), so `h(x)` is the identity rather than `Rfb'`. A
pleasant consequence: the 180° field mirror leaves both velocity blocks alone
(`v_b' = (Rz(π)Rfb)' Rz(π) v_f = v_b`), so `mirrorState()` stays a
position-and-quaternion operation.

The belief is a Gaussian in square-root information form
(`GaussianInfo.hpp`), so covariance never loses positive-definiteness and
marginals/conditionals are cheap. Every vision update is a MAP optimisation
(trust-region Newton, `funcmin.hpp`) whose Laplace approximation becomes the
posterior — i.e. an iterated-EKF-style update rather than a single
linearisation (`Measurement.cpp`).

The camera-bias states absorb the systematic ~1–3° mount/kinematics pitch
error discovered in the recorded data (field-line/landmark residuals growing
with range²). They are a slow random walk applied on the camera side of the
extrinsic, `Tfc = Tfb(x) · Tbc · R(δc)`, and are estimated jointly with the
pose (`SystemLocalisation.h`).

## Prediction: nothing is a known input

The process model is autonomous — the rigid-body kinematics plus a random walk
on the rates — and is integrated through an SDE with RK4
(`SystemEstimator.cpp`):

```
d(rBFf)/dt = Rfb*vBb      dq/dt = 0.5*Xi(q)*omegaBb
d(vBb)/dt  = dw_v         d(omegaBb)/dt = dw_omega
d(bGyro)/dt = dw_b        d(deltaC)/dt  = dw_c
```

The gyroscope and the walk-engine odometry used to be a known *input* to this
model. They are now **measurements of the velocity states**
(`MeasurementBodyRates.h`), which is the point of carrying `vBb` and `omegaBb`:
an input asserts its value as truth, so its noise can only be expressed as
process noise on whatever it drives and any bias it carries is unmodelled,
whereas a measurement carries its own sigma, can be gated or suppressed, can be
contradicted by the rest of the belief, and can have a bias that is itself a
state.

- **`MeasurementGyroscope`**: `y = omegaBb + bGyro + v`, σ = 0.02 rad/s
  (`SIGMA_GYRO`). It runs unconditionally, including through a fall — it is the
  one sensor that measures a topple honestly, whatever the posture.
- **`MeasurementBodyVelocity`**: `y = vBb + v`, from finite-differencing the
  odometry stream `Htw` (`twistFromOdometry`) at σ = 0.15 m/s
  (`SIGMA_ODOM_VEL`) — loose but real information, present as evidence and never
  as truth. Suppressed while not upright, where the same class supplies a
  **zero-velocity update** instead (σ = 0.02 m/s lying still, 0.30 m/s while
  toppling or being levered upright).

The odometry's *attitude* is not used at all. The walk-engine attitude slips
badly while turning — on `data2` the odometry alone loses ~150° of yaw by
t = 40 s, mocap-verified — while the gyroscope tracks every turn and only drifts
at a smooth bias-like rate. That drift used to be removed by a heuristic that
averaged "quiet" samples; the bias is now a state, made observable because the
gyroscope sees the sum while the landmarks pin `omegaBb` through the attitude
they constrain. It is unobservable to the upstream Mahony filter, whose bias
integrator is driven by the cross product of two near-vertical vectors and so has
no component about the vertical — which is precisely the axis that turns into
heading drift. On `data2` it converges to `[+0.31, +2.40, −0.78] deg/s` at
σ = 0.09 deg/s, and the old heuristic's estimate of the z bias was ~0.75 deg/s.

Process noise is a diagonal PSD (`Parameters` in `SystemLocalisation.h`). The
rate states now carry the dominant terms — 0.35 m/s/√s on `vBb` (a walking robot
changes forward speed over a step) and 1.50 rad/s/√s on `omegaBb` — while the
pose PSDs (0.02 m/√s horizontal, 0.01 rad/√s attitude) are a floor against
collapse in weakly-observable directions rather than the main source of growth:
uncertainty reaches position by integrating velocity uncertainty, which is where
the real ignorance lives. The gyroscope bias walks at 2e-4 rad/s/√s (slow thermal
drift only, so vision can average it out over many frames) and the camera bias at
3e-4 rad/√s. The tuning target is *consistency*: the mocap truth lies inside the
3-sigma position bound on 98.7% of `data2` samples (see *Consistency* below for
the one direction where the belief is not consistent, and why that is a bias
rather than a tuning failure).

### Event ordering

`predict()` refuses to integrate backwards. A negative `dt` is not merely
inaccurate: RK4 runs with a negative step and the process-noise square-root
information goes as `1/(σ√dt)`, so the belief turns to NaN on the spot and every
subsequent measurement inherits it. The old guard was an `assert`, which does
nothing — `CMakeLists` defines `NDEBUG` for every non-Debug build and Release is
the default. An out-of-sequence event now leaves the belief and the clock alone
and is counted.

Rejecting is not the same as handling it: the measurement that follows is applied
at the filter's current time rather than its own, which is a real modelling error
whose size scales with the lag. `backwardPredicts()` and `maxBackwardDt()` exist
so that error is visible rather than assumed absent, and the run prints a warning
when it happens. Doing better needs an out-of-sequence measurement update, which
this is not. All four recordings report zero rejections, as an offline replay of a
correctly ordered stream should.

## Initialisation: first-frame grid solve + start-half prior

No baseline pose is used. `solveInitialPose()` (`fieldLocalisation.cpp`)
grid-searches (x, y, yaw) on the first usable vision frame (z/roll/pitch come
from kinematics), scoring each cell with the robust landmark log-likelihood
(~170 ms once). On-field landmarks are invariant under the field's 180°
rotation, so the global maximum has an equally-likely mirror; the
GameController-style start-in-own-half prior (`ownHalfXSign`) picks the side.
The filter then starts from a deliberately loose prior and sharpens
recursively.

The remaining states start where a robot at kick-off is: velocity at zero with a
prior wide enough to cover a walk if it is not (0.30 m/s, 0.50 rad/s), and the
gyroscope bias at zero with a prior covering a few deg/s, which is the scale of
the drift it exists to absorb. The four attitude entries all carry the loose yaw
figure (0.25 per component — a body rotation of `s` maps to `0.5*s` on the
components), because yaw uncertainty is not separable across them; the tight
roll/pitch prior is re-established within a frame by gravity.

This is the one place the side is decided in the shipped configuration, and it is
a prior that is true exactly once, at kick-off. On `data3_webots` it lands on the
mirrored half — the run ends 4.4 m and 164° from that log's NUbots baseline with
no flips — and nothing downstream corrects it (see *Side disambiguation*).

## Measurement models

Applied per vision frame through `system.process()` (which also drives the
optional Gaussian-mixture hypothesis bank):

- **Field landmarks** (`MeasurementFieldLandmarks`): YOLO detections arrive
  as calibrated unit rays in the camera frame (the lens model — equidistant
  fisheye on the robot, rectilinear under webots — is already applied upstream),
  classes = goal posts and L/T/X line intersections.
  Rays are pre-gated (20°) and associated against the known field map at the
  prior mean by **surprisal nearest neighbour**: pairs are ranked and accepted
  by surprisal in the tangent plane of the predicted bearing, relative to the
  clutter crossover, rather than by raw angle. The likelihood is a robust
  inlier-Gaussian + uniform-clutter mixture on the unit sphere (σ = 0.25 rad),
  so gross mis-detections are absorbed rather than fought.
  The inlier weight is **per-detection, taken from the YOLO confidence**
  (w ≈ confidence, capped at 0.95): confidence is a statement about whether the
  box is a true positive, which is what the mixture weight means — inflating σ
  instead would claim the landmark is certainly real but poorly measured. As
  w → 0 the term flattens towards clutter, contributing to neither the gradient
  nor the Hessian, so weak detections cannot sharpen the posterior.
- **Gravity** (`MeasurementGravity`): accelerometer direction observes roll
  and pitch. Gated on the *specific force* rather than on posture:
  `| ‖a‖ − g | < 3 m/s²`. That is the actual validity condition — true of a robot
  lying still, false in free fall or on impact — and a better test than "upright"
  while walking too (455 frames of `data2` fail it).
- **Kinematic height** (`MeasurementKinematicHeight`): torso height from the
  leg kinematic chain observes z. The one model a fall genuinely invalidates,
  so it is the one suppressed by posture.
- **Body rates** (`MeasurementGyroscope`, `MeasurementBodyVelocity`): see
  *Prediction* above. These go in before the no-detections gate, because neither
  has anything to do with whether YOLO found something — and a fallen robot's
  camera is in the carpet, so those are exactly the frames where an unmeasured
  velocity state would integrate the pre-fall gait straight off the field.
- **Quaternion norm** (`MeasurementQuaternionNorm`): unit-norm pseudo-measurement.
  The four attitude states carry three degrees of freedom and `quat2rot`
  normalises, so `|q|` is invisible to every other model; without this the MAP
  Hessian is singular along it and the Newton step has a flat direction to wander
  down.
- **Field lines** exist (`MeasurementFieldLines`) but ship disabled: their
  per-frame errors are strongly correlated and multi-line capture degraded
  accuracy on the recorded data.
