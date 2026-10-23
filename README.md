# Gimbal tracker

[![CI](https://github.com/gambrelcolby63/gimbal-tracker-ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/gambrelcolby63/gimbal-tracker-ros2/actions/workflows/ci.yml)

<img src="docs/demo/flyby.gif" alt="Top-down flyby of the simulated tracker. The turret sits at the origin, its barrel swings onto a crossing target, orange dots are noisy sensor fixes, the purple trace is the Kalman estimate, and the star is the intercept the gimbal leads." width="360">

The portrait clip is the shipped flyby, slowed to one-third speed so the motion is easy to follow. It is a picture of the simulator: no weapon, seeker, or platform.

A simulated two-axis gimbal that estimates a moving target and points at the intercept of a constant-speed projectile. The control, estimation, and lead math live in a ROS-free C++17 library. A ROS 2 Humble package is a thin wrapper around that library.

This is a closed-loop controls exercise: a motor model, cascaded PID, a Kalman filter, and vacuum kinematics. There is no weapon, fuze, seeker hardware, or platform integration. The "projectile" is a point mass with a speed you set in YAML.

## Problem

A turret at the origin has to keep a pan/tilt gimbal on a target it cannot measure cleanly.

1. A trajectory generator flies the target: constant velocity, constant acceleration, a circle, or a sequence of straights, coordinated turns, speed changes, and weaves.
2. A sensor adds noise, either in Cartesian position or in azimuth, elevation, and range.
3. A Kalman filter estimates the target. The shipped flyby uses constant velocity. A coordinated-turn model and an interacting multiple-model filter are there for maneuvering paths. Bearing/range measurements use an EKF update.
4. A lead solver finds the launch direction whose projectile meets the target. Gravity is optional and, when enabled, is vacuum drop only.
5. Cascaded position and rate loops drive each gimbal axis. The plant includes motor electrical dynamics, inertia, viscous and Coulomb friction, and voltage, torque, and rate limits.

```mermaid
flowchart LR
  traj["Target trajectory"] --> truth["Truth state"]
  truth --> sensor["Sensor noise"]
  sensor --> ekf["CV / CA Kalman filter<br/>bearing EKF optional"]
  ekf --> lead["Lead solver"]
  lead --> pid["Cascaded PID<br/>anti-windup and feedforward"]
  plant["2-axis gimbal plant"] --> pid
  pid -->|"voltage"| plant
  plant --> bore["Boresight"]
```

The two axes are independent. That is the usual model for a mass-balanced gimbal: no inertial coupling, and no gravity torque on the axes themselves. Pan wraps. Tilt stops at mechanical limits.

## Repository layout

| Path | What it is |
| --- | --- |
| `include/gimbal_tracker/`, `src/` | C++17 library. No ROS headers. |
| `config/gimbal_tracker.yaml` | Plant, gains, filter, lead, and trajectory tuning. |
| `tests/` | GoogleTest for the controller, Kalman filter, coordinated turn, IMM, and lead solver. |
| `apps/` | Step response, Monte Carlo consistency, closed-loop flyby, and the CV-versus-IMM study. |
| `scripts/` | Bode / step metrics (`python-control`), NEES / RMSE plots, and the flyby and maneuver animations. |
| `docs/` | Generated plots, metric JSON, and `docs/demo/` (MP4 and GIF). |
| `ros2_ws/` | Humble messages, five nodes, launch file, RViz config. |
| `Dockerfile`, `.devcontainer/` | Humble dev container for local use or Codespaces. |

## Gimbal plant

Each axis is a DC motor on a rigid inertia. State is angle, rate, and current:

\[
L \dot i = v - R i - K_b \omega, \qquad
J \dot\omega = \mathrm{sat}\big(K_t i - b\omega - \tau_c \tanh(\omega / \omega_s)\big), \qquad
\dot\theta = \omega
\]

Voltage, torque, and rate are saturated. The electrical pole \(R/L\) is near 1000 rad/s, so the integrator substeps inside each control period. `measure()` adds encoder and gyro noise; the integrated state stays clean. Shipped parameters are in `config/gimbal_tracker.yaml` (pan inertia 0.05 kg·m², tilt 0.04 kg·m², 24 V bus).

## Cascaded controller

Position is an outer P loop. Rate is an inner PI loop. Feedforward adds the commanded rate and, on the voltage, a back-EMF / viscous term plus an acceleration term:

\[
\omega_\mathrm{cmd} = \mathrm{sat}\big(K_{p,\theta}\, e_\theta + K_{\mathrm{ff},\theta}\, \omega_\mathrm{ff}\big)
\]
\[
v = \mathrm{sat}\big(K_{p,\omega}\, e_\omega + \xi + K_{\mathrm{ff},\omega}\, \omega_\mathrm{cmd} + K_{a,\omega}\, \alpha_\mathrm{ff}\big)
\]

Pan angle error is wrapped to \((-\pi, \pi]\). The integrator state \(\xi\) uses back-calculation so a saturated voltage does not wind up:

\[
\dot\xi = K_{i,\omega}\, e_\omega + K_\mathrm{aw}\,(v - v_\mathrm{unsat})
\]

`rate_kff` is set to \(K_b + b R / K_t\) and `rate_kaff` to \(J R / K_t\), which is the steady voltage for a commanded rate and the inertial voltage for a commanded acceleration. Position \(k_\mathrm{ff} = 1\) passes the reference rate straight through, so a ramp does not need a constant position lag to produce that rate.

## Kalman filter

### Constant velocity

State \(\mathbf{x} = [p^\top, v^\top]^\top\) in the turret frame. Over a step \(\Delta t\) the mean motion is

\[
F = \begin{bmatrix} I & \Delta t\, I \\ 0 & I \end{bmatrix}
\]

Process noise is piecewise-constant white acceleration, independent on each axis, with standard deviation \(\sigma_a\):

\[
Q_i = \sigma_a^2
\begin{bmatrix}
\Delta t^4 / 4 & \Delta t^3 / 2 \\
\Delta t^3 / 2 & \Delta t^2
\end{bmatrix}
\]

The Monte Carlo truth uses that same draw, \(a \sim \mathcal{N}(0, \sigma_a^2)\), held over the step. That is what makes the NEES check meaningful.

### Constant acceleration

State appends acceleration. Process noise is piecewise-constant jerk with standard deviation \(\sigma_j\). The discrete input vector on one axis is

\[
G = \begin{bmatrix} \Delta t^3 / 6 & \Delta t^2 / 2 & \Delta t \end{bmatrix}^\top, \qquad Q_i = \sigma_j^2\, G G^\top
\]

### Update

Cartesian measurements use \(H = [I\ 0]\). The covariance update is the Joseph form

\[
P^+ = (I - KH)\, P\, (I - KH)^\top + K R K^\top
\]

so \(P\) stays symmetric when \(R\) does. Normalized estimation error squared and normalized innovation squared are

\[
\mathrm{NEES} = \tilde{\mathbf{x}}^\top P^{-1} \tilde{\mathbf{x}}, \qquad
\mathrm{NIS} = \mathbf{y}^\top S^{-1} \mathbf{y}
\]

For a consistent linear filter the average NEES sits near the state dimension (6 for CV, 9 for CA) and the average NIS near 3.

### Bearing / range EKF

The nonlinear measurement is azimuth, elevation, and range:

\[
h(p) = \begin{bmatrix} \operatorname{atan2}(y, x) \\ \operatorname{atan2}(z, \sqrt{x^2+y^2}) \\ \|p\| \end{bmatrix}
\]

The update linearizes \(h\) at the predicted state. The azimuth innovation is wrapped to \((-\pi, \pi]\) so a target crossing the negative x-axis does not look like a \(2\pi\) jump. On the boresight axis, where azimuth is undefined, that row is dropped.

### Coordinated turn

The turn model flies a horizontal arc at a nearly constant rate \(\omega\). Altitude stays on the constant-velocity model. With state \([p^\top, v^\top, \omega]^\top\),

\[
\begin{aligned}
p_x^+ &= p_x + \big(v_x \sin\omega\Delta t - v_y (1 - \cos\omega\Delta t)\big) / \omega \\
p_y^+ &= p_y + \big(v_x (1 - \cos\omega\Delta t) + v_y \sin\omega\Delta t\big) / \omega \\
v_x^+ &= v_x \cos\omega\Delta t - v_y \sin\omega\Delta t \\
v_y^+ &= v_x \sin\omega\Delta t + v_y \cos\omega\Delta t
\end{aligned}
\]

A tiny \(\omega\) uses the straight-line limit, so the mean and the Jacobian agree. Process noise is the same white acceleration as the constant-velocity model, plus a random walk on \(\omega\).

### Interacting multiple model

One Kalman filter has to pick a motion model before the target has shown it. An interacting multiple model runs three filters at once — straight flight (CV), accelerating flight (CA), and a coordinated turn (CT) — and keeps a probability for each.

Each step does four things:

1. **Mix.** The filters trade a probability-weighted position and velocity. Acceleration stays inside the CA filter, and turn rate stays inside the CT filter. Writing those extra states in as zeros would wipe the turn rate on every straight-flight cycle.
2. **Predict and update.** Each filter sees the same measurement.
3. **Reweight.** The filter that expected the measurement gains probability. The weights are the Gaussian likelihoods, normalized with a log-sum-exp.
4. **Combine.** The reported position and velocity are the weighted average. Reported acceleration is the CA acceleration plus the centripetal acceleration \(\omega \times v\) of the turn model.

A mode is expected to last its sojourn time (8 s, 3 s, and 4 s in the shipped file). Whatever probability that leaves is split across the other two modes. Select `model: imm` or `model: ct` in the YAML. The flyby above stays on `cv`.

## Lead solver

The projectile leaves the muzzle at constant speed \(s\). With no drag and no gravity, the intercept time is the smallest positive root of

\[
\| \mathbf{r} + \mathbf{v} t \| = s t
\]

which is the quadratic \(a t^2 + b t + c = 0\) with

\[
a = \|\mathbf{v}\|^2 - s^2, \quad b = 2\, \mathbf{r}\cdot\mathbf{v}, \quad c = \|\mathbf{r}\|^2
\]

\(\mathbf{r}\) is the target position relative to the muzzle. If the target is faster than the projectile and is not closing, there is no positive root and the solver says so. A target with acceleration is iterated: predict the position at the candidate time, then set the next time to range over \(s\).

With gravity \(g > 0\) the shot is a vacuum parabola. For a predicted intercept at horizontal range \(R\) and height \(z\), elevation \(\theta\) solves

\[
z = R \tan\theta - \frac{g R^2}{2 s^2 \cos^2\theta}
\]

Let \(u = \tan\theta\) and \(c = g R^2 / (2 s^2)\). Then

\[
c u^2 - R u + (z + c) = 0
\]

The smaller positive time of flight is kept (the low trajectory). Because the target is moving, that elevation is wrapped in a fixed-point iteration on the intercept time. The shipped config uses \(s = 220\,\mathrm{m/s}\) and \(g = 9.80665\,\mathrm{m/s}^2\). Set `gravity_m_s2` to 0 for a pure straight-line lead.

## Results

Regenerated by `scripts/run_analysis.sh` from the shipped YAML. The pan step is 0.25 rad. Margins are for the unsaturated linearization (Coulomb friction removed). The nonlinear simulation and that linear model differ by **0.00024 rad RMS**.

| | No Coulomb | With Coulomb | Linear model |
| --- | ---: | ---: | ---: |
| Rise time, 10–90% | 0.261 s | 0.266 s | 0.262 s |
| Overshoot | 9.7% | 8.9% | 9.5% |
| Settling, 2% band | 1.88 s | 1.22 s | 1.88 s |
| Steady-state error | \(4.5\times 10^{-5}\) rad | \(\sim 0\) | \(4.7\times 10^{-5}\) rad |

| Loop | Gain margin | Phase margin | Gain crossover |
| --- | ---: | ---: | ---: |
| Inner rate | no phase crossover | 52.9° | 7.85 rad/s |
| Outer position | 28.1 dB | 53.1° | 5.78 rad/s |

![Pan step response](docs/step_response.png)

![Bode plots of the pan loops](docs/bode.png)

The rate loop's phase approaches −180° from above inside the plotted band and never crosses it, so the linear gain margin is unbounded. The outer loop is the one that sets the stability limit, at about 26 rad/s.

The consistency run is 50 Monte Carlo trials of the CV filter with the process noise the filter assumes. Mean NEES is **6.07** (state dimension 6) and mean NIS is **3.02** (measurement dimension 3). Every time sample of the averaged NEES sits inside the 95% chi-squared band for 50 runs, \([5.08, 7.00]\). Final RMSE is **0.25 m** and **0.37 m/s**.

![NEES and NIS](docs/nees.png)

![Position and velocity RMSE](docs/rmse.png)

The flyby is the shipped target: it starts at \((50, -40, 3)\,\mathrm{m}\) and crosses in front of the turret at 12 m/s. After the one-second acquisition, pan tracking error is **0.050 rad RMS** and tilt error is **0.0011 rad RMS**. Every lead solution on that pass is valid. Mean estimated position error after one second is **0.083 m**. The command sits about **0.049 rad** ahead of the line of sight in azimuth, and gravity adds about **0.0046 rad** of super-elevation.

![Closed-loop flyby](docs/tracking.png)

### Maneuvering targets

A constant-velocity filter lags as soon as the target stops flying straight. The comparison below is the `maneuver` block in the YAML, not the flyby. The target starts at \((30, -45, 4)\,\mathrm{m}\) going north at 14 m/s, coasts for 4 s, turns for 5 s at 0.35 rad/s (a 40 m radius), coasts for 3 s, speeds up to 22 m/s, then weaves for 6 s. Forty Monte Carlo runs share the same 1 m position noise. `scripts/run_analysis.sh` writes the plots.

<img src="docs/demo/maneuver.gif" alt="Side-by-side of one maneuver. The constant-velocity estimate cuts the corner of the turn and the weave. The interacting multiple-model estimate stays on the path. A strip chart shows the straight, acceleration, and turn probabilities." width="360">

The clip is one run of that scenario, slowed slightly. The left map is the constant-velocity filter. The right map is the mixture. The strip is that run's model probability.

| Leg | CV position RMSE | IMM position RMSE |
| --- | ---: | ---: |
| Straight | 5.55 m | 0.70 m |
| Coordinated turn | 6.91 m | 1.15 m |
| Speed change | 3.61 m | 1.40 m |
| Weave | 6.26 m | 1.41 m |

On the turn, velocity RMSE is 8.33 m/s for the constant-velocity filter and 2.50 m/s for the IMM. Averaged over the forty runs, the turn model holds probability **0.75** during the turn, the acceleration model holds **0.62** during the speed change, and the constant-velocity model holds **0.40** on the straight legs. That last number is pulled down by the coast that follows the turn: the turn model is slow to hand probability back. On the single run in the clip, constant velocity leads the opening straight, the turn model takes the turn and the weave, and the acceleration model takes the speed change.

Mean NEES on the six kinematic states is **3.14×10³** for the constant-velocity filter and **49** for the IMM. The 95% band for forty runs is \([4.97, 7.12]\). The mixture is much less wrong, and it is not chi-squared consistent.

![Position and velocity RMSE through the maneuver](docs/imm_rmse.png)

![NEES of the IMM and of the constant-velocity filter](docs/imm_nees.png)

![Mean IMM model probabilities](docs/imm_probabilities.png)

Limits worth keeping in view: the turn is horizontal only. Model-specific states are not mixed across modes. The combined covariance is a moment-matched Gaussian, which is why the NEES test is not clean. Sojourn times are tuning knobs. This study is still the simulator: no seeker, weapon, or platform.

## How to run

Dependencies for the library: CMake, a C++17 compiler, Eigen3, yaml-cpp. Analysis also wants Python 3 with the packages in `requirements.txt` (`numpy`, `scipy`, `matplotlib`, `PyYAML`, `control`).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
python3 -m venv .venv && . .venv/bin/activate && pip install -r requirements.txt
PYTHON=python BUILD_DIR="$PWD/build" bash scripts/run_analysis.sh
```

The phone-friendly flyby above is regenerated with the same library and YAML. It needs `ffmpeg` as well as the Python packages:

```bash
PYTHON=python BUILD_DIR="$PWD/build" bash scripts/make_flyby_demo.sh
```

That writes `docs/demo/flyby.mp4` and `docs/demo/flyby.gif` (about 24 seconds, portrait). The maneuver comparison is the same idea:

```bash
PYTHON=python BUILD_DIR="$PWD/build" bash scripts/make_imm_demo.sh
```

That writes `docs/demo/maneuver.mp4` and `docs/demo/maneuver.gif`. The RMSE, NEES, and probability plots come from `scripts/run_analysis.sh`, which also fails if the IMM turn-position RMSE is not below the constant-velocity filter.

`ctest` covers the cascaded loop (step specs, wrap, rate saturation, anti-windup, ramp feedforward), the CV and CA filters (scalar update, process-noise \(Q\), NEES/NIS, bearing Jacobian, azimuth branch cut), the coordinated-turn model and the IMM mix, and the lead solver (closed-form intercepts, unreachable targets, vacuum drop).

### ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch gimbal_tracker_ros gimbal_tracker.launch.py rviz:=true
```

`rviz:=false` is the default, so the same launch works over SSH. Switch the sensor with `measurement:=bearing_range`. The launch file reads `config/gimbal_tracker.yaml` from the package share directory; pass `config:=/path/to/gimbal_tracker.yaml` to try a different tune without reinstalling.

| Topic | Message | Role |
| --- | --- | --- |
| `/target/truth` | `TargetTruth` | Noise-free target from `target_sim_node` |
| `/sensor/measurement` | `SensorMeasurement` | Cartesian or bearing/range |
| `/target/estimate` | `TargetEstimate` | Filter mean, 6×6 covariance, NIS |
| `/gimbal/state` | `GimbalState` | Noisy encoder and gyro |
| `/gimbal/truth` | `GimbalState` | Integrated plant state |
| `/gimbal/command` | `GimbalCommand` | Pan and tilt voltage |
| `/gimbal/setpoint` | `TrackSetpoint` | Lead angles, time of flight, intercept |

Frames are `map` → `turret` → `boresight` and `turret` → `target`. RViz also shows the target sphere, the boresight arrow, and the lead line.

### Docker and Codespaces

`Dockerfile` is `ros:humble-ros-base` plus the C++ and Python dependencies. The devcontainer builds the library, runs `ctest`, and `colcon build`s the workspace on first open.

```bash
docker build -t gimbal-tracker .
docker run --rm -it -v "$PWD":/workspace gimbal-tracker bash
```

RViz needs a display forwarded into the container. The library tests and the analysis scripts do not.

## Tuning

All of the numbers above come from `config/gimbal_tracker.yaml`. The loops were shaped so the inner crossover (7.9 rad/s) sits above the outer one (5.8 rad/s) with about 53° of phase margin on both. Coulomb friction is small next to the available torque; it trims overshoot and shortens the 2% settling time. To retune, edit the YAML and rerun `scripts/run_analysis.sh`. The unit test `Controller.ShippedYamlTuningSettlesWithFriction` fails if that file no longer settles a 0.25 rad step.

## What I learned

Replace the prompts below with your own notes before using this as an application piece.

- **Plant versus model:** Where did friction or the voltage limit show up once the linear Bode plot was no longer the whole story?
- **Loop shaping:** The shipped margins are about 53°. What broke when the rate loop was slowed toward the position loop?
- **Estimator consistency:** Mean NEES landed on 6.07. Which noise term would you change if a real sensor ran hot or cold?
- **Lead geometry:** When did the solver report no intercept, and how much did gravity move the elevation command on a slower projectile?
- **ROS boundary:** What belonged in the C++ library, and what did the nodes only route?

## License

MIT. See [LICENSE](LICENSE).
