# Control — the study file for section 7 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at
"can hold a technical conversation" depth, the sentence an interviewer is fishing for, and
an honest **`piros2` line**. Section 7 carries no priority note in the syllabus — it is
breadth, not one of the two sections that matter (C++ and SLAM). Per the honest-claim rule:
reading this does not move anything into skills.md, and **`piros2` has no control loop at
all** — nothing in the repo commands an actuator. The `piros2` lines below therefore mostly
say "not touched" and name the nearest thing (the depth estimator's `max_rate` pacing, the
usb_cam poll timer beating against the camera's cadence), which are *timing* lessons, not
control ones. The Hovermap tie-in is the one that matters for the room: **Cortex sends
velocity/position commands over DJI's Onboard SDK; DJI's flight controller keeps the inner
attitude and rate loops** ([01_emesent-company-domain.md](01_emesent-company-domain.md) §3),
so Emesent's control problem is the *outer* loops of a cascade fed by SLAM instead of GPS.

## Mental model to carry through the whole file

```
 reference ──►(+)──► CONTROLLER ──► ACTUATOR ──► PLANT ──► output
              (−)        ▲            (limits)   (dynamics)     │
               │         │ feedforward from the reference/model │
               └─────────┴──────────── SENSOR ◄─────────────────┘
                                   (noise, delay, sample rate)
```

Every item below is a question about one box: PID is the controller; cascades stack the
loop several deep; feedforward is the dotted path that bypasses the error; state space/LQR/
MPC are other controllers; saturation lives in the actuator box; timing lives in the sensor
and the loop period. For a multirotor the cascade is
**position → velocity → attitude → body rate → motor mixer**, and the interviewer's
question is always "which loop, how fast, and what feeds it".

## 1. PID: tuning, windup, derivative filtering

- **The law:** `u(t) = Kp·e + Ki·∫e dt + Kd·de/dt`, `e = r − y`. P acts on the present error
  (stiffness; too much → oscillation), I on the accumulated past (kills steady-state error
  against a constant disturbance — gravity, a bias, a slope; too much → slow oscillation,
  overshoot), D on the predicted future (damping; fights the overshoot P+I create). The
  "standard form" writes `Kp(e + (1/Ti)∫e + Td·ė)` — Ti and Td in seconds are what tuning
  rules quote; `Ki = Kp/Ti`, `Kd = Kp·Td`.
- **Tuning, in the order an engineer does it:** (1) P only, raise until the response is
  brisk with a little overshoot; (2) add D to damp it; (3) add just enough I to remove the
  offset. Ziegler–Nichols closed-loop: raise P until sustained oscillation at ultimate gain
  `Ku` and period `Tu`, then `Kp = 0.6 Ku, Ti = 0.5 Tu, Td = 0.125 Tu` — aggressive
  (quarter-amplitude decay), a starting point not an answer. Rule of thumb: for a
  second-order-ish plant, aim for phase margin ~45–60°, gain margin ~6 dB; a system that
  rings at a fixed frequency when you bump it has too little margin.
- **Integrator windup:** when the actuator saturates (§7) the error stays large, the
  integrator keeps summing, and when the error finally reverses the stored integral has to
  be *unwound* before the output leaves the rail — the classic big overshoot after a large
  step. Fixes: **clamping** (stop integrating when the output is saturated *and* the error
  has the same sign), **back-calculation** (feed `Kb·(u_sat − u)` back into the integrator
  so it tracks the saturated output), integrator limits, and integrating only inside a
  band around the setpoint. Every flight stack carries one of these on every axis.
- **Derivative filtering:** the pure derivative has gain proportional to frequency, so it
  amplifies sensor noise without bound — a raw D term on a noisy gyro or a quantised
  encoder is a motor-heating buzz. Standard cure: low-pass the derivative, `Kd·s/(1 +
  s·Td/N)` with `N ≈ 8–20`, or an explicit first-order filter (PX4's rate loop filters the
  D-term separately from the gyro). Second trick: **derivative on measurement, not error**
  — differentiating the *setpoint* produces a spike ("derivative kick") whenever the
  reference steps; `d(−y)/dt` avoids it and is what most real controllers do. Discretise
  with backward Euler or Tustin at a fixed `dt`, and use the *measured* `dt` if the loop
  period jitters (§8).
- **Classic mistakes:** tuning I before D; no anti-windup; D on the error; running the loop
  at a variable rate with a fixed `dt`; forgetting the units (a gain tuned in degrees is
  57× wrong in radians).
- **Interviewer's target sentence:** "P for stiffness, I for steady-state error, D for
  damping; the two things that bite in practice are windup under saturation — clamp or
  back-calculate — and the derivative amplifying noise, so you filter it and take it on the
  measurement rather than the error."
- **`piros2` line:** not touched — no PID anywhere. The nearest *idea* is the
  `ScaleAligner` in `depth_align.py`: a full "conform depth to the map" correction was
  unstable (a feedback loop with the TSDF's ~1.25-voxel ray-cast bias walked walls away),
  so it was rebuilt as a high-pass — correct only the deviation from a rolling median, so
  the loop has no DC gain and cannot drift. That is an anti-windup-shaped decision, arrived
  at the hard way, but it is not a controller.

## 2. Cascaded control loops

- **Structure:** the outer loop's output is the inner loop's *setpoint*. Motor drive: current
  (torque) loop → velocity loop → position loop. Multirotor: position → velocity →
  attitude → body rate → mixer (§6). Each inner loop takes a fast, well-measured state and
  turns a messy actuator into a well-behaved one the next loop out can treat as ideal.
- **Why:** (1) *disturbance rejection where the disturbance enters* — a gust changes body
  rate long before it changes position, and the rate loop rejects it before the position
  loop notices; (2) *linearisation* — the inner loop hides actuator nonlinearity
  (motor/ESC curve, thrust vs RPM²); (3) *decoupling* of time scales — you tune one loop
  at a time; (4) *saturation handling* — limits are applied at each layer as setpoint
  limits (max tilt, max rate) instead of one big rail (§7).
- **The rule of thumb:** each inner loop should have roughly **5–10× the bandwidth** of the
  loop outside it, so the outer loop can treat the inner as instantaneous. Tune inside-out:
  the rate loop first with the attitude loop open, then attitude, then velocity, then
  position. If an outer loop is pushed to a bandwidth close to the inner's, the two
  interact and the whole stack oscillates — the usual symptom of "I raised the position
  gain and now it wobbles".
- **The cost:** more loops = more delay in series and more sensors needed (a good gyro for
  the rate loop, an attitude estimate for the next, a velocity estimate for the next, a
  position source — GPS or SLAM — for the outermost).
- **Interviewer's target sentence:** "Cascade so the fast inner loop rejects disturbances
  and linearises the actuator, keep each inner loop 5–10× faster than the one outside it,
  and tune from the inside out — the outer loop's output is just the inner loop's setpoint."
- **Hovermap tie:** this is exactly the seam Emesent sits on. DJI's flight controller owns
  rate + attitude (the fast inner loops with the gyro on the FC), and Cortex sits *outside*
  it, issuing velocity/position commands over the Onboard SDK with SLAM as the position
  source — so Emesent's control tuning is the outer loops, and its latency budget is the
  SLAM pose (odometry ~15 Hz per the Wildcat paper) feeding a velocity loop.
- **`piros2` line:** not touched. The nearest structural analogue is the *pipeline*
  pacing chain — the depth estimator's `max_rate: 5` sets the tempo and everything
  downstream (projector, mesher, odometry) inherits it — one clock at the front rather
  than each stage running its own, which is the "outer sets the inner's setpoint" shape
  applied to data flow rather than control.

## 3. Feedforward vs feedback

- **Feedback** acts on the measured error: it needs no model, rejects unmodelled
  disturbances, but is always *reactive* — the error must exist before it acts, and gain is
  bounded by stability (delay and phase margin, §8).
- **Feedforward** acts on the *reference* or a *measured disturbance* through a model of
  the plant, before any error appears: hover thrust (`m·g` split by tilt), gravity/friction
  compensation on a robot arm, velocity and acceleration feedforward in trajectory tracking
  (`u = u_ff(r, ṙ, r̈) + K·e`), a feedforward yaw rate when following a curved path. It
  cannot destabilise (open loop) but is only as good as the model, so it always sits *with*
  a feedback term that mops up the residual.
- **The rule:** feedforward carries the bulk of the effort in a known manoeuvre; feedback
  is sized for the *uncertainty*, not the whole task. A trajectory tracker with good
  feedforward runs low feedback gains and is smoother and more robust than one where the
  P gain does all the work. PX4's position/velocity controller takes velocity and
  acceleration setpoints as feedforward alongside the position error for exactly this reason.
- **Interviewer's target sentence:** "Feedforward is model-driven and proactive, feedback
  is error-driven and reactive; use feedforward for what you can predict (gravity, hover
  thrust, the planned trajectory) so the feedback loop only has to handle the surprises."
- **`piros2` line:** not touched. The nearest thing is a *feedforward-shaped* choice in the
  perception chain: the tape-measure `depth_scale: 2.69` and the spec-derived intrinsics
  (`fx = fy ≈ 907 px`) are fixed model corrections applied open-loop, and the
  `ScaleAligner` high-pass is the feedback that handles the per-frame residual — the same
  "model carries the bulk, feedback trims" split, on a scale factor rather than a thrust.

## 4. State space, LQR

- **State space:** `ẋ = A x + B u`, `y = C x + D u` — the plant as a first-order vector ODE
  in a state `x` (position, velocity, attitude, rates, biases…). Everything a transfer
  function can say, plus MIMO for free, plus the internal state. Discrete form
  `x[k+1] = A_d x[k] + B_d u[k]` via zero-order hold. Two questions come with it:
  **controllability** (`rank[B, AB, …, Aⁿ⁻¹B] = n` — can `u` steer every state) and
  **observability** (`rank[C; CA; …]` — can `y` reveal every state). A quadrotor's
  linearised hover model is controllable with four inputs and twelve states; a
  featureless-tunnel SLAM is an *observability* failure in the same language (the SLAM
  file's §17).
- **Full-state feedback:** `u = −K x` places the closed-loop poles of `A − BK`. Pole
  placement picks them by hand; **LQR** picks `K` to minimise
  `J = ∫ (xᵀQx + uᵀRu) dt` — `Q` weights state error, `R` weights control effort — giving
  `K = R⁻¹BᵀP` with `P` from the continuous algebraic Riccati equation
  `AᵀP + PA − PBR⁻¹BᵀP + Q = 0` (DARE for discrete). Tune by scaling `Q`/`R` (Bryson's rule:
  diagonal entries `1/max_acceptable²`), not by moving poles. LQR is optimal for the
  linear model and, in the SISO case, has guaranteed **≥ 6 dB gain margin and ≥ 60° phase
  margin** — but those guarantees evaporate once you put a state estimator in front (LQG
  has no guaranteed margins — Doyle 1978), which is why LQG designs are checked with loop
  transfer recovery or just tested.
- **In practice:** LQR needs the full state, so it pairs with a Kalman filter (LQG); it is
  linear, so on a nonlinear plant you linearise about an operating point (hover) or
  re-linearise along a trajectory (time-varying LQR — the workhorse of aggressive quadrotor
  tracking in the research literature). It has no notion of constraints — that is MPC's
  job (§5).
- **Interviewer's target sentence:** "State space gives you MIMO and the internal state;
  LQR turns the tuning problem into choosing Q and R, solves a Riccati equation for the
  gain, and gives you an optimal linear controller — but you need the whole state, so in
  practice it's LQG, and the margins are no longer guaranteed."
- **`piros2` line:** not touched as control. The state-space *vocabulary* is used:
  `se3.py` builds and inverts SE(3) transforms, the keypoint detector composes rotations
  as a state, and the pose is a state estimate — but no dynamics model, no `A`, no `B`, no
  gain.

## 5. MPC

- **The idea (receding horizon):** at every control step, solve a finite-horizon optimal
  control problem — predict `N` steps ahead with a model, minimise a cost (tracking error
  + effort + terminal cost) subject to **explicit constraints** (actuator limits, rate
  limits, state limits, obstacle keep-out) — apply *only the first input*, then re-solve
  next step with fresh measurements. The re-solving is the feedback.
- **Why people pay for it:** constraints handled *by design* rather than by clamping after
  the fact (§7); MIMO coupling and previewed references handled naturally; the same
  framework covers path following, obstacle avoidance and energy limits. **What it costs:**
  a solver in the loop. Linear model + quadratic cost + linear constraints = a QP (OSQP,
  qpOASES, HPIPM); nonlinear model = NMPC, an NLP solved by SQP or interior point (acados,
  ACADO, CasADi + IPOPT) — small quadrotor NMPC problems solve in well under a millisecond
  on a laptop-class CPU and at 50–100 Hz on embedded ARM, which is why NMPC quadrotor
  racing exists. Explicit MPC precomputes the piecewise-affine law offline for tiny problems.
- **The knobs and the failure modes:** horizon `N` (too short → myopic, unstable; too long →
  slow), the terminal cost/set (what makes it provably stable), model mismatch (the plant
  isn't the model — robust/tube MPC adds a margin), and **infeasibility** — if the
  constraints can't all be met the solver returns nothing, so real implementations soften
  constraints with slack variables and always have a fallback controller.
- **Interviewer's target sentence:** "MPC solves a constrained finite-horizon optimisation
  every step and applies the first move; you use it when constraints matter — actuator
  limits, obstacles — and you pay with a solver in the loop, a model to maintain and an
  infeasibility story."
- **Hovermap tie:** collision avoidance and Explore-mode path following are the natural MPC
  candidates in a Cortex-like stack, but I have no evidence Emesent uses MPC rather than
  a planner plus a tracking controller — a question to ask, not a claim to make.
- **`piros2` line:** not touched. The nearest concept is the offline TSDF pipeline's
  "rebuild the TSDF from frame memory when the pose graph moves" (slam-plan P3) —
  recompute from a horizon of stored inputs rather than trust the integrated state — but
  that is mapping, not control.

## 6. Drone flight control: attitude, rate and position loops

- **Why a cascade at all:** a multirotor is *underactuated* — four (or six/eight) thrusts
  give you total thrust plus three torques; to move sideways it must *tilt*. So position
  control necessarily goes through attitude: position error → desired acceleration vector →
  desired thrust magnitude and desired tilt (attitude) → attitude error → desired body
  rates → rate error → torques → motor mixer → PWM/DShot. Yaw is decoupled from
  translation and driven by the weaker drag-torque difference between CW/CCW props, so it
  gets lower authority and slower tuning.
- **The loops and their rates (PX4 as the concrete reference; ArduPilot's structure is the
  same with a 400 Hz main copter loop):**
  - **Rate loop** — innermost, PID on angular velocity from the gyro, runs at the gyro
    sample rate (hundreds of Hz to ~1 kHz, or higher on fast racing stacks). Its D-term
    filtering and its notch filters on prop-frequency vibration are where flight quality
    is won or lost. This is the loop that lives on the flight controller and never on the
    companion computer.
  - **Attitude loop** — a proportional law on the *quaternion error* (tilt-prioritised:
    correct roll/pitch first, yaw as budget allows), outputs a body-rate setpoint,
    ~250 Hz.
  - **Velocity loop** — PID on the velocity estimate, outputs acceleration/thrust-vector
    setpoint, with feedforward from the trajectory; ~50 Hz.
  - **Position loop** — usually just P on position error → velocity setpoint, ~50 Hz.
  - A **state estimator** (EKF fusing IMU with GPS/baro/mag/vision — PX4's EKF2) feeds all
    of them; the innermost loops need only the gyro, the outer loops need the fused
    velocity and position.
- **What GPS-denied changes:** nothing in the inner loops. The position/velocity loops need
  a position source; indoors that is a SLAM/VIO pose injected as an external-vision-style
  measurement, at ~10–30 Hz with tens of ms of latency and occasional jumps (loop closure).
  The estimator has to time-align it (delay compensation with a buffer of IMU states),
  down-weight or reject a jump, and the outer loop gains must respect its latency. **This
  is Hovermap's job on a DJI M300/M350:** SLAM gives relative pose → Cortex commands
  velocity/position over the Onboard SDK ("Enable API Control", the RC's flight-mode switch
  has a Hovermap position; flip out and DJI takes back Atti mode without GPS) → DJI keeps
  the attitude and rate loops. Emesent's own manual names **"SLAM slip"** as the failure
  that cancels position hold and RTH — a degenerate SLAM estimate must not be fed to the
  velocity loop, so the honest response is to drop out of position control, not to fly
  on it.
- **Interviewer's target sentence:** "Rate loop on the gyro at up to a kHz, attitude above
  it, velocity and position outside at ~50 Hz; a quadrotor has to tilt to translate so
  position control goes through attitude; and going GPS-denied only swaps the position
  source — Hovermap feeds SLAM pose into DJI's outer loops over the Onboard SDK and never
  touches the inner ones."
- **`piros2` line:** not touched. What the repo *has* built is the piece Cortex would hand
  over: an orientation estimate with a measured recovery behaviour when
  tracking breaks (`just gate flick|occlude`, 65.3° correction / 0.48° tail; 18.4° snap /
  0.95° tail) and a `was_tracking` "count a blank as lost" rule — i.e. it knows when its
  estimate is *not* good enough to hand to a controller, which is the SLAM-slip
  contract in miniature. Nothing consumes it as a setpoint.

## 7. Actuator saturation and rate limits

- **Saturation:** every actuator has a magnitude limit (motor max thrust, servo travel,
  a velocity command cap) and the plant is nonlinear the moment you hit it. Consequences:
  integrator windup (§1); loss of authority on one axis stealing from another (a
  multirotor at full collective thrust cannot also produce roll torque — the **mixer**
  must prioritise: PX4's control allocation gives roll/pitch priority over yaw and scales
  thrust down to keep attitude authority — the "airmode" idea keeps a rate-control margin
  even at zero throttle); and **limit cycles** — a loop that is stable in the linear region
  can oscillate at a fixed amplitude once it spends time on the rail (describing-function
  analysis predicts it).
- **Rate (slew) limits:** actuators also have a maximum *rate of change* — servo slew,
  ESC/motor spin-up time, and deliberate command rate limits (max tilt rate, max
  acceleration, jerk-limited trajectories). A rate limit is a phase lag that grows with
  amplitude, so a large step that looks fine in simulation can go unstable on hardware.
- **The disciplined answers:** anti-windup on every integrator; **setpoint shaping** —
  limit and smooth the *reference* (max velocity/acceleration/tilt at each cascade layer,
  jerk-limited trajectory generation) so the inner loops are asked only for what the
  actuators can deliver; explicit constraint handling if you can afford MPC; and
  fail-safes for persistent saturation (a drone that needs >90 % thrust to hover is
  overloaded or has a failed motor).
- **Interviewer's target sentence:** "Saturation turns a linear design nonlinear — windup,
  cross-axis authority loss, limit cycles — so you anti-windup every integrator, shape the
  reference so the loops are never asked for more than the actuator has, and let the mixer
  prioritise attitude over yaw and thrust when it runs out."
- **`piros2` line:** not touched. The nearest analogue is a *rate limit on a source*: the
  depth estimator's `max_rate` (`depth_estimator.py`, `on_frame` returns early if less
  than `1/max_rate` s has passed since the last processed frame) throttles the whole
  pipeline to 5 Hz because that is what `rgbd_odometry` sustains — "ask the downstream
  only for what it can deliver" — and the 2026-08-16 finding that shrinking the sync
  queues instead made things *worse* is the same lesson as shaping the reference rather
  than clamping at the actuator: pace the source, don't starve the sink.

## 8. Control loop timing and sample rates

- **Sample rate:** Nyquist is a floor, not a target — pick the loop rate at **10–30× the
  closed-loop bandwidth** so the discretisation and the half-sample delay of the
  zero-order hold don't eat the phase margin. Rate loops at 500 Hz–1 kHz for a bandwidth of
  tens of Hz; position loops at 50 Hz for a bandwidth of a fraction of a Hz.
- **Delay is phase:** a pure delay `τ` costs `ω·τ` radians of phase at frequency `ω`. A
  loop with 30° of spare phase margin at 10 Hz (63 rad/s) can afford ~8 ms of extra
  latency before it rings. Sensor latency, filter group delay, transport delay (a pose
  arriving over a serial link or DDS) and compute time all add up in series — which is why
  the fast loop lives next to the gyro on the FC and the slow loops can tolerate a
  companion computer.
- **Jitter is worse than delay:** a constant delay can be compensated (predict forward,
  Smith predictor, buffered IMU states in the estimator); a *variable* one cannot, and a
  fixed `dt` in the discrete integrator/derivative under a jittering period injects error
  proportional to the jitter. Rules: trigger the loop from the *sensor sample* (PX4's
  rate loop runs on gyro arrival), not from a free-running timer that beats against it;
  measure and use the actual `dt`; timestamp at acquisition, not at receipt; and keep the
  loop's worst-case execution time inside its period (the real-time file, section 9). A
  loop that occasionally takes 2× its period is a loop with a 2× delay some of the time —
  a stability margin you cannot see on the average.
- **Multi-rate discipline:** outer loops run at sub-multiples of the inner rate; a slow
  sensor (SLAM at 15 Hz) feeding a fast loop must be interpolated or run through the
  estimator, never zero-order-held straight into a high-gain loop.
- **Interviewer's target sentence:** "Sample at ten to thirty times the bandwidth, count
  every millisecond of latency as phase, treat jitter as worse than delay, and drive the
  loop from the sensor sample rather than a timer that beats against it."
- **`piros2` line:** this is the one item where the repo has a real, measured story,
  though from a data pipeline rather than a control loop:
  - **Timer beating** — `usb_cam` grabs frames on a ROS timer at the requested rate; at
    `framerate:=30` its 33 ms poll timer beat against the camera's own 33 ms cadence and
    delivered a steady **24.0 fps** while a raw V4L2 capture got 30 at the same moment;
    polling at 60 (16 ms) caught every frame — 29.72 fps, and 42–60 fps once the camera's
    control baseline was fixed (`docs/info/camera.md`, `camera.launch.py` defaults
    `framerate` to 60.0). Two unsynchronised clocks at the same nominal rate is exactly
    the "trigger from the sensor, not a timer" mistake, seen from the data side.
  - **Timestamps at acquisition vs receipt** — `/image_raw` header stamps lag wall clock by
    a steady ~0.73 s (a UVC/driver fault); a first version of the edge detector gated on
    stamp age and silently dropped 100 % of frames, so every span in the repo is measured
    on the receiving process's own clock. Any control loop fed by this camera would have to
    do the same or add 0.73 s of unmodelled delay.
  - **Pacing** — `max_rate: 5` on the estimator sets one tempo for the chain; measured
    per-cloud TF wait went from p50 813 ms with 30 % dropped to p50 15 ms with none.
  - No loop closes on any of it: there is no actuator, and milestone 7's pan/tilt servo
    (the roadmap's "introduces actuators, control loops, and the visual-servoing feedback
    path") was never built.

## What to say if asked "have you done control?"

"Not on this project — `piros2` is a perception pipeline with no actuator; my control
experience is embedded firmware loops, not a flight stack. What I do have is the timing
side: I've measured a poll timer beating against a sensor's cadence, a 0.73 s timestamp
fault that broke a freshness gate, and I paced a pipeline at its slowest stage's rate
because starving the sync made it worse. I know the multirotor cascade — rate loop on the
gyro, attitude, velocity, position — and that Hovermap's seam is the outer loops over the
Onboard SDK with SLAM as the position source, so the interesting question for me is how
Cortex decides its pose is good enough to hand to that loop." Then stop.
