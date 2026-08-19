# Navigation and planning — the study file for section 6 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at conversation depth, the
sentence an interviewer is fishing for, and an honest **`piros2` line**. The syllabus gives
this section no priority note (breadth, not a named gap), but it is the layer Emesent's
Cortex sells — waypoints on a live cloud, bounding-box Explore, Shield, return-to-home
along the flown path (§3 of [01](01_emesent-company-domain.md)) — so the concepts need to
be held even though `piros2` **contains none of it**: no planner, no costmap, no controller,
no `cmd_vel`, no Nav2; the Pi is a sensor head and the "planner" for every recording was a
human arm. Per the honest-claim rule, the `piros2` lines below are short and say exactly
that, naming the nearest real thing where one exists.

## Mental model to carry through the whole file

```
 goal / explore-box
        │
        ▼
 GLOBAL PLANNER  ── path on the (cost)map, ~1 Hz, A*/Dijkstra/sampling ──┐
        │                                                                │ replan on
        ▼                                                                │ map change
 LOCAL PLANNER / CONTROLLER ── trajectory over the next 1–5 s, 10–50 Hz, │ or failure
   respects dynamics, dodges fresh obstacles (DWA / TEB / MPPI / MPC)    │
        │                                                                │
        ▼                                                                ▼
 vehicle controller (attitude/velocity loops — section 7)      BEHAVIOUR LAYER
        │                                                      (behaviour tree: recoveries,
        ▼                                                       RTH, failsafes, rally point)
 world ──► sensors ──► SLAM/localisation ──► map (occupancy / ESDF / cloud) ──► back to the top
```

The recurring trade is **completeness/optimality vs compute vs reactivity**: a global
search is optimal on a stale map, a local optimiser is fast on fresh data but myopic, and
the behaviour layer decides what to do when either fails. On a drone with no GPS and no
comms (Cortex AL2), *all* of it runs onboard, on the same box doing SLAM.

## 1. Global vs local planning

- **Global:** start → goal on the current map, ignoring dynamics beyond a footprint; runs
  once per goal and again on a timer (Nav2 default 1 Hz) or when the path is blocked; graph
  search (§2) or sampling (§3). **Local:** follow the global path over a short horizon
  (1–5 s) using the *live* sensor data and the robot's velocity/acceleration limits,
  10–50 Hz; DWA/TEB/MPPI (§5). The global map is the whole world; the local map is a
  rolling window around the robot.
- Why two layers: one search that is both globally optimal and dynamically feasible at
  50 Hz does not fit in the compute budget, and the local layer must react to things the
  map hasn't absorbed yet. Failure modes at the seam: a global path through a gap the
  local planner can't take (§12); local minima where the local optimiser oscillates while
  the global path stays valid.
- Drone flavour: the global layer plans in 3D (OctoMap/ESDF, corridors), the local layer
  is a trajectory optimiser (§6) tracked by the flight controller — Cortex's AL2 plans on
  the live point cloud and *issues velocity/position commands while DJI keeps the inner
  attitude loop*.
- **Interviewer's target sentence:** "Global gives you a route on the map at low rate,
  local makes it feasible and safe on fresh data at high rate; the interesting bugs live
  where they disagree."
- **`piros2` line:** neither exists. The nearest structural analogue is the same
  rate-splitting instinct — TSDF integration at frame rate, meshing on a 10–15 s timer,
  and RTAB-Map's odometry-vs-graph split — but nothing here plans motion.

## 2. A*, Dijkstra, D* Lite, hybrid A*

- **Dijkstra:** uniform-cost search from the start, priority queue on `g`; optimal,
  explores in every direction, `O(E log V)`. **A\*:** `f = g + h`; with an *admissible* `h`
  (never overestimates — Euclidean or octile on a grid) it is optimal; *consistent* `h`
  means no node is reopened. Weighted A* (`f = g + ε·h`) trades optimality for speed by a
  bounded factor. Jump Point Search prunes uniform grids. Resolution is the cost driver —
  a 3D grid at 10 cm over 100 m³ is 10⁶ cells and A* still copes; at 2 cm it doesn't.
- **D\* Lite** (Koenig & Likhachev 2002): incremental replanning — search from the goal,
  and when edge costs change (new obstacle seen), repair only the affected part of the
  previous search instead of starting over. The right tool when the map keeps changing
  under a moving robot; the LPA* lineage.
- **Hybrid A\*** (Dolgov et al., Stanford DARPA Urban Challenge): search over continuous
  `(x, y, θ)` using kinematically feasible motion primitives (arcs; Dubins/Reeds-Shepp
  for the heuristic) so the result is *drivable* by a car-like robot; Nav2's
  `SmacPlanner` ships 2D, Hybrid-A* and state-lattice variants. Costs: cell cost, turning,
  reversing, and a smoothing pass afterwards.
- Classic mistake: an inadmissible heuristic that quietly makes A* suboptimal, or a
  heuristic so weak the search degenerates to Dijkstra; ignoring the tie-break, which
  decides whether A* is fast on open grids.
- **Interviewer's target sentence:** "A* is Dijkstra with an admissible heuristic; D* Lite
  is A* that repairs itself when the map changes; hybrid A* searches in the vehicle's
  configuration space so the path is one it can actually drive."
- **`piros2` line:** no graph search over space. The only shortest-path-shaped code is
  the pose graph optimiser's search over *poses* — a least-squares problem, not a route.

## 3. RRT, RRT*, PRM, sampling-based planning

- **Why sampling:** grids explode with dimension (an arm has 6–7 DoF; a drone with
  velocity in the state has 6+); sampling-based planners never build the grid. **RRT**
  (LaValle 1998): sample a state, find the nearest tree node, steer toward the sample a
  bounded step, keep it if collision-free; probabilistically complete, *not* optimal,
  jagged paths. **RRT\*** (Karaman & Frazzoli 2011): choose the best parent within a
  shrinking radius and *rewire* neighbours through the new node → asymptotically optimal,
  slower per iteration. RRT-Connect grows two trees; Informed RRT* samples inside the
  ellipse that could still improve; BIT* batches. **PRM** (Kavraki 1996): multi-query —
  sample many states, connect each to k nearest with a collision-checked local planner,
  then run graph search on the roadmap; good when the map is static and many queries come.
- Ingredients you must supply: a sampler (with goal bias), a distance metric, a steering
  function (trivial for holonomic, a two-point BVP for kinodynamic — §12), a collision
  checker (§9); and post-processing (shortcutting, spline smoothing) because raw
  sampling paths are ugly. OMPL is the library (MoveIt uses it).
- **Interviewer's target sentence:** "RRT finds *a* path fast, RRT* converges to the
  optimal one with rewiring, PRM builds a reusable roadmap; sampling wins when the state
  space is high-dimensional or free space is 3D volume, grid search wins when the map is a
  2D grid you already have."
- **`piros2` line:** not touched. RANSAC — sample, fit, count inliers, keep the best
  (`room_layer.py`, `solvePnPRansac`) — is the repo's only randomised algorithm, and it
  is estimation, not planning.

## 4. Costmaps, inflation, obstacle layers

- **`nav2_costmap_2d`:** a 2D grid of `uint8` cost per cell — 0 free, 1–252 rising, 253
  `INSCRIBED_INFLATED` (centre here → footprint definitely collides), 254 `LETHAL`
  (obstacle), 255 `NO_INFORMATION` — built from **layers** in order: *static* (the map
  server's grid), *obstacle* (marks cells from LaserScan/PointCloud2 hits within
  `obstacle_max_range`, clears along the ray to `raytrace_max_range` — the free-space
  carving of section 5), *voxel* (3D obstacle voxels projected down — the 2.5D answer to
  overhangs), *inflation* (from every lethal cell: 253 out to the robot's inscribed
  radius, then an exponential decay `exp(−cost_scaling_factor · (d − r_inscribed))` out to
  `inflation_radius`), plus plugin layers (range sensors, denoise, keepout zones,
  speed-limit zones). Two instances: the **global** costmap over the whole map for the
  planner and a **local** rolling window (~3–5 m) for the controller.
- Why inflate: it turns a point-robot planner into one that respects the footprint
  cheaply, and the decaying skirt makes A* prefer the middle of corridors. Classic
  mistakes: `inflation_radius` smaller than the robot → plans that clip walls;
  overinflated → doorways vanish; not clearing (marking without raytrace) → ghost
  obstacles that never go away; treating `NO_INFORMATION` as free.
- 3D drones use the same idea as an **ESDF** (Voxblox, nvblox, FIESTA): distance to
  the nearest obstacle per voxel, gradient for free — clearance and repulsion in one lookup.
- **Interviewer's target sentence:** "A costmap is a layered occupancy grid with the
  robot's footprint pre-baked as inflation, so a point planner is safe; the layers mark
  and clear from sensors, and most navigation bugs are inflation and clearing settings."
- **`piros2` line:** no costmap. The confirmation-count voxel map (`min_weight`) and the
  TSDF weight threshold are occupancy-*like* accumulators without a free state; a
  clearance/inflation notion appears only in Emesent's Shield, which is §9's story.

## 5. Local planners: DWA, TEB, MPPI

- **DWA** (Fox, Burgard, Thrun 1997): sample `(v, ω)` in the *dynamic window* — the
  velocities reachable within one control step given acceleration limits — forward-simulate
  each for ~1–2 s, score by progress to goal, clearance, and speed, pick the best. Simple,
  robust, poor in tight spaces and doorways (short horizon, oscillation). Nav2's `DWB` is
  its plugin-scored descendant.
- **TEB** (Rösmann, Timed Elastic Band): treat the local path as a band of poses *with time
  intervals* and optimise it (g2o) for minimum time subject to obstacle distance,
  velocity/acceleration and kinematic (car-like) constraints; handles non-holonomic
  robots and produces smooth, feasible trajectories; sensitive to tuning and local minima,
  ROS 2 port is community-maintained.
- **MPPI** (Williams et al. 2016; Nav2's default controller since Iron): sample hundreds
  to thousands of noisy control sequences, roll each out through the vehicle model,
  weight by `exp(−cost/λ)`, and average — a sampling MPC that handles non-differentiable
  costs (costmap, footprint), vectorises well, and re-plans at 20–50 Hz. Also in the family:
  Regulated Pure Pursuit (a path *follower*, not a planner — slows in corners and near
  obstacles), Graceful controller.
- **Interviewer's target sentence:** "DWA samples velocities and scores rollouts, TEB
  optimises a timed band of poses, MPPI samples whole control sequences and averages by
  cost — the trend is toward sampling-MPC because it takes arbitrary costs and dynamics."
- **`piros2` line:** none. The nearest thing to a "controller" is the pacing parameter
  `max_rate: 5` on the depth estimator that keeps the pipeline synchronous with what
  RTAB-Map's odometry can absorb — a rate decision, not a motion decision.

## 6. Trajectory optimisation and smoothing

- **Path vs trajectory:** a path is geometry; a trajectory is time-parametrised (position,
  velocity, acceleration at every t). Smoothing turns a jagged planner output into
  something a controller can track: shortcutting, spline fits, or an optimisation with a
  smoothness cost + obstacle cost + dynamic limits.
- **The methods:** *CHOMP* (covariant gradient descent over waypoints, obstacle cost from
  a signed distance field), *STOMP* (stochastic version), *TrajOpt* (SQP with convex
  collision constraints); for quadrotors, **minimum-snap** (Mellinger & Kumar 2011 — the
  quadrotor is *differentially flat*, so a piecewise polynomial in position and yaw with
  minimised 4th derivative is a QP and maps directly to thrust/attitude), and the
  B-spline-in-ESDF line (Fast-Planner, Ego-Planner, HKUST): kinodynamic path → safe
  corridor → B-spline optimised for smoothness, clearance and feasibility, replanned at
  ~10 Hz. Continuity matters: C² (continuous acceleration) is the minimum for a smooth
  attitude command; time allocation between segments is its own optimisation.
- Classic mistake: smoothing a path *into* an obstacle (smoothing must see the collision
  cost), or optimising in a corridor too narrow for the dynamics so the QP is infeasible.
- **Interviewer's target sentence:** "Planners give paths, controllers need trajectories;
  the drone standard is a polynomial or B-spline optimised for snap/jerk and clearance
  inside a safe corridor, exploiting differential flatness — and it must be re-solved as
  the map changes."
- **`piros2` line:** not touched. The one continuous-time trajectory object in the repo
  is a *record*, not a plan — `/world/trajectory` (`nav_msgs/Path`, the pose graph's
  optimised keyframe poses).

## 7. Frontier exploration and autonomous exploration

- **Frontier** (Yamauchi 1997): a free cell adjacent to an unknown cell. Exploration =
  detect frontiers → cluster them → pick one by a utility (distance cost vs expected
  information gain, frontier size, heading change) → plan to it (§2/§3) → move → update the
  map → repeat until no reachable frontier remains. It only exists on a map with an
  explicit *unknown* state (section 5 §7).
- **3D / drone versions:** receding-horizon **next-best-view** (Bircher et al. 2016 —
  grow an RRT, score each branch by unmapped volume its sensor would see, execute the
  first edge, regrow), FUEL (Zhou et al. 2021, frontier information structure + fast
  planning), **GBPlanner** (graph-based, the CERBERUS SubT team's underground explorer —
  local volumetric-gain graph + a global graph for homing), and TARE. Underground the
  practical knobs are gap size (Emesent's Explore fits gaps down to 2.4 m horizontal ×
  1.75 m vertical on an M300), a **bounding box** to keep exploration finite, minimum
  frontier size (ignore slivers), blacklisting unreachable frontiers, and — the one that
  decides missions — a **battery budget** with return cost folded into the utility so the
  robot turns for home while it still can (Cortex "budgets battery" and returns along the
  flown path).
- Termination and honesty: "explored" means no frontier is *reachable* under the
  constraints, not that the volume is complete; report coverage as a number.
- **Interviewer's target sentence:** "Frontier exploration is a loop of pick-a-boundary,
  go, re-map; the engineering is in the utility — information gain against travel and
  return cost under a battery budget — and in a bounding box and gap limits so it stops."
- **`piros2` line:** no exploration; recordings were hand-held sweeps and the "where next"
  was a person. The nearest concept is honest: `mesh_fill.py` labels each mesh
  component's largest boundary loop as the *frontier* (the edge of what the camera has
  seen) and refuses to fill it — that is the unknown boundary an explorer would fly toward,
  represented on a surface instead of a grid.

## 8. Coverage planning

- **Area coverage:** decompose free space into cells (boustrophedon / trapezoidal
  decomposition), sweep each with a lawnmower pattern sized to the sensor or tool
  footprint, order the cells (TSP-like); spanning-tree coverage (STC) for grids; complete
  coverage vs a target percentage; overlap for photogrammetry (front/side lap 70–80 %).
- **Surface / inspection coverage** (what a mapping payload cares about): choose *views*
  such that every surface patch is seen within range and at an acceptable angle and
  density — view planning on a mesh or coarse model, then a tour through the views;
  standoff distance for a wall inspection, and for LiDAR mapping the constraint is
  point density and range (Emesent's KB says features must be within ~40 m for SLAM
  health). Explore is volume coverage with a 360° × 290° LiDAR footprint; a stope scan is
  "cover the walls of a void with enough density for a volume calculation" — the coverage
  target is the deliverable's accuracy, not the flight.
- Classic mistake: planning coverage on the *prior* model and not replanning when the
  live map shows the model was wrong (a stope is never the design shape).
- **Interviewer's target sentence:** "Coverage is decomposition plus a sweep sized to the
  sensor footprint; for inspection it becomes view planning against a surface at the
  density the deliverable needs, and it has to replan against the live map."
- **`piros2` line:** not touched. The `just record` sweeps and the fixed-view offscreen
  renders in `tools/verify/render_mesh.py` (origin, top, oblique) are a human's coverage,
  chosen for a check, not planned.

## 9. Collision checking

- **Discrete:** test the robot's footprint (polygon rasterised onto the costmap; a
  sphere/ellipsoid or a set of spheres in 3D) at sampled poses along the trajectory, at a
  spacing ≤ the cell size — sampling only the waypoints misses the segments between them.
  **Distance-based:** an ESDF/SDF gives clearance and its gradient in O(1) per query, so
  the check becomes "clearance > radius" and the same field drives repulsion in the
  optimiser. **Continuous / swept:** FCL (BVH, GJK) for meshes and swept volumes; MoveIt
  uses it. Cost per check decides which planner is affordable — RRT* makes ~10⁴–10⁶ checks.
- **Unknown space is not free:** a collision checker must have a policy for
  `NO_INFORMATION` (treat as lethal or as free-with-penalty); "passive" checkers only know
  what the map/last scan shows and cannot dodge a moving object.
- **Emesent's Shield** is a collision *bubble*: a virtual ellipsoid with configurable
  clearance that grows forward with speed (stopping distance scales with `v²/2a`) and
  shrinks beside structures so the drone can work close to walls; it is a velocity
  limiter/veto layer, passive by their own description ("won't dodge moving objects") —
  i.e. it checks the live cloud, not a predicted world.
- **Interviewer's target sentence:** "Collision checking is footprint-vs-map at a spacing
  finer than the cells, or clearance from a distance field; a speed-dependent safety
  bubble like Shield is the drone form, and the policy for unknown space is a design
  decision, not a default."
- **`piros2` line:** none — no footprint, no field. The repo's clearance-shaped number
  is `max_range`/`depth_max` (points beyond 6 m are dropped as unreliable), which is a
  sensor-trust cut, not a safety margin.

## 10. Nav2 architecture and behaviour trees

- **The servers** (each a lifecycle node, brought up by `lifecycle_manager`, plugins via
  `pluginlib`): `bt_navigator` (runs the tree, exposes `NavigateToPose` /
  `NavigateThroughPoses` actions), `planner_server` (global costmap + planner plugins:
  NavFn, SmacPlanner, Theta*), `controller_server` (local costmap + controller plugins:
  MPPI, DWB, RPP; progress checker, goal checker), `behavior_server` (spin, backup,
  drive-on-heading, wait), `smoother_server`, `waypoint_follower`, `velocity_smoother`,
  `collision_monitor` (a last-line safety stop between controller and base), plus
  `map_server` and AMCL for localisation. Everything talks over ROS 2 actions —
  cancellable, with feedback.
- **Behaviour trees** (BehaviorTree.CPP v4 in Jazzy, XML files you can swap per robot):
  *Sequence* (all children in order), *Fallback* (first that succeeds), decorators
  (`RateController` replans at 1 Hz, `RecoveryNode` retries a child N times with a
  recovery in between), `PipelineSequence` for the plan-then-follow flow. The default tree
  is "compute path (replanned at 1 Hz) → follow path", wrapped in a recovery fallback:
  clear costmaps → spin → wait → backup. Why BTs over FSMs: modular, reactive (ticked, not
  event-driven), readable, and the same leaf nodes recombine into different behaviours
  without recompiling. Nav2 also has the "route" server, docking, and (from Jazzy) a
  developing 3D story — but it remains a ground-robot 2D/2.5D stack; drone autonomy
  stacks (PX4 + custom planners, Aerostack2, MRS) mostly don't run Nav2, and Cortex is
  its own stack of the same shape.
- **Interviewer's target sentence:** "Nav2 is a set of lifecycle servers — planner,
  controller, behaviours, smoother — orchestrated by a behaviour tree over ROS 2 actions;
  the tree is where recovery policy lives, and its plugin design is why you swap MPPI for
  DWB in a YAML line."
- **`piros2` line:** Nav2 has never run here, and the repo's nodes are plain (not
  lifecycle) rclpy nodes. What *does* exist is what Nav2 assumes: a REP-105 frame tree
  (`map → odom` from the pose graph, `odom → base_link` from odometry, `base_link →
  camera_link → camera_optical_frame` static), services with `Trigger` (`~/reset`,
  `~/save`, `~/clear`), latched (TRANSIENT_LOCAL) markers, and launch files that compose
  the stack — the substrate, without any of the servers on top. Section 10 of the
  syllabus (lifecycle, rclcpp) is the named gap this exposes.

## 11. Recovery behaviours and failsafes

- **Nav2 recoveries:** clear costmap (ghost obstacles), spin (re-perceive), backup, wait,
  then abort — ordered cheapest first, retried a bounded number of times by the tree.
- **Drone failsafes** (PX4/ArduPilot vocabulary, and Cortex's): RC loss, datalink loss,
  low battery (warn → RTL → land at thresholds), geofence breach, EKF/position-estimate
  failure (→ altitude hold or land — you cannot RTL without a position), and *what
  Cortex adds for GPS-denied*: navigation source auto-selects **SLAM → GPS → INS**;
  INS-only is tolerated ~10 s before RTH; **RTH retraces the previously-flown safe path**
  ("may not be the shortest") because the flown corridor is the only space known to be
  free — a shortest path through unmapped rock is not a recovery; a **rally point** is
  the fallback on error/comms loss; and "SLAM slip" cancels position hold and RTH — a
  failsafe that *disables* itself when its own estimate is untrustworthy, so the pilot
  takes over (flip the RC's mode switch and DJI regains control in Atti mode).
- **Design rules:** an escalation ladder (each rung cheaper and tried first; the last
  rung guarded), every rung testable on the bench, evidence logged, and no silent
  degradation — a robot that keeps flying on a bad estimate is worse than one that lands.
- **Interviewer's target sentence:** "Recoveries are cheap-first ladders bounded by the
  behaviour tree; drone failsafes degrade the mission (hold → RTH along known space →
  land) and must be conditioned on estimate health — Cortex's RTH-along-the-flown-path
  and rally point are exactly 'only trust the space you've seen'."
- **`piros2` line:** no vehicle, so no vehicle failsafe — but the same ladder was built
  for infrastructure: the Ansible `wifi` role's watchdog escalates *reassociate → driver
  reload → guarded reboot* (10-minute uptime floor, 1-hour cooldown), each rung with journal
  evidence, drilled to unaided recovery; camera launchers reap themselves on link death;
  and every camera consumer fails loudly rather than idling. Same shape, different layer.

## 12. Kinodynamic constraints, holonomic vs non-holonomic

- **Holonomic** platform: can move in any direction of its configuration space
  instantly (omni-wheel base; a quadrotor's *position* path is effectively holonomic —
  it can hover and translate any way, but subject to dynamic limits). **Non-holonomic:**
  a constraint on *velocity* that can't be integrated into a constraint on position —
  a differential drive can't slide sideways, a car has a minimum turning radius (Dubins
  paths: forward-only arcs and lines; Reeds–Shepp: with reversing). Consequence: a
  geometric path with corners is unfollowable; the planner must search in `(x, y, θ)`
  with feasible primitives (hybrid A*, state lattices) or the controller must cut corners.
- **Kinodynamic planning:** the state includes velocities, and edges must respect
  acceleration/jerk bounds — kinodynamic RRT (needs a two-point boundary-value steering
  function), motion-primitive search, or plan geometry then time-parametrise with
  limits. For a quadrotor: velocity, acceleration (tilt) and jerk limits, yaw-rate
  limits, and the underactuation that couples translation to attitude — which is why
  minimum-snap works and why "5 m/s max in Explore" is a planner constraint, not just a
  pilot preference. Legged robots (Spot) are nearly holonomic in the plane, which is one
  reason Autowalk missions are simple to author.
- Classic mistake: planning for a point that can stop and turn in place, then flying a
  vehicle that can do neither at speed — the local layer then oscillates or overshoots.
- **Interviewer's target sentence:** "Holonomic means any direction now, non-holonomic
  means velocity constraints you can't integrate away; kinodynamic planning puts velocity
  and acceleration limits into the search itself, and a drone's are tilt-coupled — so
  its planners work in flat outputs with snap or jerk costs."
- **`piros2` line:** no vehicle model — the only motion in the repo is a hand-held
  camera, and the *odometry* side of kinematics is what got built (rotation-only Kabsch,
  then 6-DoF via RTAB-Map and the pose graph). Nothing constrains or commands motion.

## What to say if asked "have you done navigation or planning?"

"No — `piros2` has no planner, costmap, controller or Nav2; the Pi is a sensor head and
every recording was a hand-held sweep. What I do have is the layer under it: a REP-105
frame tree with `map → odom` from my own pose graph, latched map/trajectory topics,
services and launch composition, and a fail-loud/escalation-ladder habit from the Wi-Fi
watchdog. I can talk through the stack — A*/hybrid A* vs sampling, costmap inflation,
MPPI, frontier exploration under a battery budget, and why Cortex's RTH follows the flown
path — as an engineer who's read it and hasn't shipped it." Then stop.
