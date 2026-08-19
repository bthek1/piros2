# Software engineering practice — the study file for section 17 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at conversation depth, the
sentence an interviewer is fishing for, and an honest **`piros2` line**. The syllabus's
priority note: **"Section 17's diagram items are the named Andromeda feedback and are already
promoted to short-term in [goals.md](goals.md)"** — so §3 (C4 / sequence / container /
component diagrams) gets a worked sketch of the `piros2` `world_mesh` session, not just a
definition. Per the honest-claim rule, reading ≠ holding: the `piros2` lines say what the repo
*actually does* — and this section is unusual in that the repo's docs are strong, verifiable
evidence for several items (decision records with dates and rejected alternatives, plans as
stable phases, symptom→cause troubleshooting, a verification doctrine, an incident record
with journal evidence). Where the repo has nothing — support escalation, customer
requirements — the line says so and names the nearest thing.

## Mental model to carry through the file

```
   decide ──► record ──► draw ──► build ──► observe ──► break ──► learn ──► record
   (design    (ADR,      (C4,     (module   (logs,      (field    (post-    (the doc the
    review)    plan)      seq)     bounds)   telemetry)  issue)    mortem)   next person reads)
```

Every item below is a point on that loop. The thing an employer like Emesent is really
checking is whether decisions, drawings, and failures *leave a written trace someone else
can act on* — because a field-robotics company has software people in Brisbane, hardware
people in a workshop, and customers in a mine three time zones away, and the only thing that
travels between them reliably is a document with a date on it.

## 1. Design reviews, and how to run one

- **What it is:** a structured critique of a design *before* it is built (or before it is
  merged, for a large change), aimed at the decisions that are expensive to reverse:
  interfaces, data flow, failure modes, concurrency, dependencies, operational cost. Not
  code style — that is code review, later and cheaper.
- **How to run one that works:**
  1. **A written design doc first, circulated 2–3 days ahead**: problem, constraints,
     options considered, the proposal, the interfaces (message/API/frame definitions),
     failure modes, testing plan, open questions. No doc, no meeting.
  2. **Reviewers read before, comment in the doc**; the meeting is for the unresolved
     comments, not for the presenter to read slides.
  3. **Scope the questions**: "does this solve the stated problem, is the interface right,
     what breaks it, what does it cost to operate/test/undo?" A facilitator keeps it off
     bikeshedding.
  4. **Decisions and actions written down at the end** — accept / accept-with-changes /
     rework — and the significant decisions become ADRs (§2).
  5. **Timebox** (45–60 min), 3–6 reviewers including one from the team that will
     *operate* or *integrate* it (firmware, field, QA), and one who did not write it.
- **Rules of thumb:** review the *rejected* alternatives as hard as the chosen one — a
  design with no alternatives was not designed; ask for the failure-mode table ("what
  happens when the LiDAR drops out for 2 s / the link dies / the disk fills"); prefer a
  small prototype's numbers to an argument.
- **Classic mistakes:** review-as-approval theatre; the meeting is the first reading;
  reviewing the code instead of the design; no record, so the same debate recurs.
- **Interviewer's target sentence:** "A design review is a pre-read document, a timeboxed
  discussion of the unresolved comments on interfaces and failure modes, and a written
  decision — and the alternatives get reviewed as hard as the proposal."
- **`piros2` line:** the repo's plans are the design docs — every plan in `docs/plans/`
  opens with the problem, honest scope, and phases each ending in something runnable, and
  several carry an explicit options table (docs/info/setup.md: reflash vs Docker vs
  build-from-source vs RoboStack, each with a verdict; the SLAM plan's "two honest routes
  — and why this plan takes both"). The "review" was a one-person-plus-agent loop, not a
  team meeting — say that plainly — but the artefacts a review needs were written *before*
  building, with dates.

## 2. Architecture documentation, ADRs

- **Architecture docs** answer "why is it shaped like this?" — the reasoning, the
  constraints, the trade-offs — not just "what is where" (which the code and a diagram
  answer). They rot unless they are small, dated, and near the code.
- **ADR (Architecture Decision Record, Nygard 2011):** one short file per significant
  decision — **Title · Status (proposed/accepted/superseded by ADR-n) · Context (forces) ·
  Decision · Consequences (good and bad)** — numbered, append-only: you never edit an
  accepted ADR, you supersede it with a new one that links back. Lives in the repo
  (`docs/adr/0007-use-cyclonedds.md`), reviewed like code. The value is that a newcomer (or
  you in a year) can find *why* the DDS is pinned, why the frame is called that, why the
  Docker option lost — and can tell whether the forces have changed.
- **What deserves an ADR:** anything hard to reverse or often re-litigated: middleware,
  languages, message contracts, frame conventions, build/deploy shape, third-party
  dependencies, "we will not do X".
- **Classic mistakes:** ADRs written after the fact from memory; a giant architecture wiki
  nobody updates; recording *what* without the rejected alternatives; no dates.
- **Interviewer's target sentence:** "Record decisions, not just designs: one small
  dated ADR per hard-to-reverse choice — context, decision, consequences, the alternatives
  that lost — append-only and superseded rather than edited, kept in the repo next to the
  code it explains."
- **`piros2` line — held in practice, if not by the name:** the repo does not have a
  `docs/adr/` folder, but it applies the discipline verifiably: (a) `docs/plans/README.md`
  fixes the rule that phases P0…Pn keep their number and scope forever and progress is
  recorded by *annotating* the phase (dates, ✓, "what actually happened") — the append-only
  rule; (b) decisions are recorded as decisions, with dates and reasons — the world-mesh
  plan says "rebuilt as a full package fork **by decision** 2026-08-15", "closed by decision
  … live gates unrun, moved to todo.md"; the perception plan "closed 2026-07-29 by decision,
  before P3's map was built"; (c) rejected alternatives are tabled with verdicts
  (setup.md's four-option table; CLAUDE.md: "Docker was considered for the Pi and rejected
  — do not reintroduce container instructions"); (d) docs are split by kind — reference in
  `docs/info/`, plans in `docs/plans/{in-progress,completed}/`, and *moving the file is the
  status change*; (e) constraints carry their evidence and date ("BEST_EFFORT receives zero
  large frames — 2.7 MB messages fragment into ~1800 UDP datagrams", "the 30 fps ceiling
  fell on 2026-08-04"). Ask me why CycloneDDS is pinned to `enp6s18` or why the SD card is
  pinned by PARTUUID and the answer is in a dated paragraph, not in someone's head.

## 3. **Sequence, container and component diagrams (C4)**

- **C4 (Simon Brown):** four zoom levels of one system, each a *static structure* diagram
  with a consistent notation (box = thing, arrow = labelled dependency, every box says
  what technology it is):
  1. **System Context** — the system as one box, the people who use it, and the external
     systems it talks to. Audience: everyone.
  2. **Container** — the separately deployable/runnable units inside the system
     (services, processes, databases, apps, file stores) and how they communicate
     (protocol on the arrow). *"Container" means runtime unit, not Docker.* Audience:
     engineers and architects.
  3. **Component** — inside one container: the major modules/classes and their
     responsibilities and dependencies.
  4. **Code** — UML class diagrams; usually generated, rarely drawn.
  Plus **supplementary** diagrams: **Deployment** (containers mapped onto machines/nodes)
  and **Dynamic** (a numbered-arrow collaboration or a UML **sequence diagram** — lifelines
  per participant, time downwards, messages as arrows, activation bars, `alt`/`loop`
  frames — for *one* scenario).
- **Rules that make them useful:** a title that says which level and which scope; a
  legend; every arrow labelled with *what* and *how* (`/depth`, RELIABLE, 5 Hz); one
  scenario per sequence diagram; keep them in the repo as text (Mermaid/PlantUML/
  Structurizr DSL) so they diff and don't rot; draw the boundary you're deciding about,
  not everything.
- **Classic mistakes:** one diagram at three zoom levels at once; unlabelled arrows;
  boxes without technology; a sequence diagram of "everything"; drawing what you wish
  rather than regenerating from what runs.
- **Interviewer's target sentence:** "C4 is four zoom levels — context, container,
  component, code — plus deployment and dynamic/sequence views for a scenario; the
  discipline is one level per diagram, technology on every box, protocol on every arrow,
  kept as text in the repo."
- **`piros2` line — held, and here is the sketch.** The repo already carries
  docs/info/just_world_mesh_diagrams.html (Mermaid, redrawn 2026-08-16 with the topic table
  *regenerated from the live graph* — which caught a node still pulling a second Wi-Fi
  copy): a node/topic dataflow (≈ container), a two-machine deployment, a TF-ownership
  view, and a recipe-lifecycle **sequence diagram** (`You → just world_mesh → Pi → launch →
  rviz2`). What it lacks is the C4 *layering* and it predates the SLAM plan's `map → odom`;
  below is the same session drawn as C4, current as of 2026-08-19 (default arguments:
  `odom:=rgbd slam:=own`).

**Level 1 — System Context**

```
 [Person] Engineer at the dev box ──runs `just run` / reads RViz──► [System] piros2 world_mesh
                                                                        │ owns
                                                            [Hardware] Logitech C922 on the Pi (UVC)
 [External data] rosbag2 MCAP bags, TUM RGB-D sequences ──replayed into──► (same system, no Pi)
 [Output] meshes/*.ply · maps/room_*.npz · captures/verify/* (files a person or script reads)
```

**Level 2 — Containers** (each row is a runnable process; arrows are DDS topics unless said)

| Container | Technology | Talks to | Protocol / topic |
| --- | --- | --- | --- |
| `usb_cam` | C++ ROS 2 node, Pi, owns `/dev/video0` | `camera_relay` (only) | `/image_raw/compressed` JPEG, RELIABLE, the *one* stream over Wi-Fi; `/camera_info` |
| `static_transform_publisher` ×2 | Pi | every TF listener | `/tf_static` latched: `base_link → camera_link → camera_optical_frame` |
| `camera_relay` | Python node, dev box | 6 local readers | `/camera_relay/compressed`, loopback fan-out |
| `depth_estimator` | Python, venv `python -m`, onnxruntime CUDA | projector, mesher, rgbd | `/depth` 32FC1 + `/depth/rgb` twin (identical stamps), paced `max_rate 5` |
| `keypoint_detector` | Python node | RViz, dashboard, `/tf` | `/keypoints/*`, `/camera/orientation`, `/world/keyframes`, `/world/trajectory`, **`map → odom`** on `/tf` (slam:=own) |
| `rgbd_odometry` | RTAB-Map C++ node | `/tf` | `odom → base_link`, 6-DoF, 2–3 Hz; services `/reset_odom_to_pose` (called by the detector) |
| `cloud_projector` | Python node | RViz | `/points`, exact-sync of `/depth` + relay RGB, posed in `odom` via latest TF |
| `tsdf_mesher` (+ `mesh_worker` process) | Python, venv, open3d CUDA | RViz; disk | `/world/mesh_live` latched Marker every 15 s; `~/save` → `meshes/live_*.ply` |
| `dashboard` | Python node | RViz | `/world/stats/compressed` 10 Hz |
| `rviz2` | C++ GUI, X11 pin | — | subscribes everything; the *viewer, not the evidence* |
| Data stores | files | — | `maps/room_<stamp>.npz`, `meshes/*.ply`, `bags/*` (MCAP), `captures/verify/*` |

**Deployment** (C4 supplementary): Pi (`192.168.2.17`, `wlan0`, ROS_DOMAIN_ID 42, CycloneDDS
pinned to `wlan0`, wifi-watchdog timer) ↔ dev box (`192.168.2.109`, `enp6s18`, GTX 1660
SUPER; two of the containers run inside `~/.venvs/piros2-perception`). Exactly one image
stream crosses the edge; every other topic is loopback.

**Level 3 — Components inside `keypoint_detector`** (verifiable in
`src/piros2_world_mesh/piros2_world_mesh/`)

```
 /camera_relay/compressed ─► [ORB front-end + Hamming matcher + Kabsch on bearing rays]  ── update_orientation
                                    │ points, descriptors
                                    ▼
                          [KeyframeStore]  keyframe_store.py — novelty-gated, ≤100 keyframes,
                                    │       descriptors + rays + 3D landmarks
                     ┌──────────────┼───────────────────────┐
                     ▼              ▼                       ▼
        [Relocaliser]        [Loop detector]          [Map persistence]
        attempt_relocalization  maybe_detect_loop        save_map / load_map → .npz
        Kabsch/Umeyama snap;    + solvePnPRansac verify   (plain arrays, no pickle)
        rgbd: /reset_odom_to_pose      │
                                       ▼
                             [PoseGraph]  pose_graph.py — SE(3) Gauss-Newton, robust edges
                                       │ optimised keyframe poses
                                       ▼
                       [TF + trajectory publisher]  publish_map_tf (map → odom) · /world/trajectory
        shared maths: se3.py (quaternions, make_transform/invert, BASE_FROM_OPTICAL)
```

**Sequence — one camera frame in the default mode**

```
usb_cam        camera_relay     depth_estimator    rgbd_odometry    cloud_projector     rviz2
   │ /image_raw/compressed │                │                │               │             │
   ├──────────────────────►│ /camera_relay/compressed        │               │             │
   │                       ├───────────────►│ (infer, ≤5 Hz) │               │             │
   │                       ├────────────────┼────────────────┼──────────────►│             │
   │                       │                ├─ /depth ───────►│ exact sync    │             │
   │                       │                ├─ /depth/rgb ───►│ (stamps ==)   │             │
   │                       │                ├─ /depth ───────┼──────────────►│ exact sync  │
   │                       │                │                ├─ /tf odom→base_link ─────────►
   │                       │                │                │   (latest TF) │──/points in odom──►
   │                       │                │                │               │             │ draw
```

The point of drawing it this way in the room: the container view makes the Wi-Fi edge and
the single-reader rule visible; the component view shows where the SLAM backend lives and
what it depends on; the sequence shows *why* `/depth/rgb` exists (exact sync pairs every
frame by construction) and why `/points` is posed upstream (RViz cannot lose a TF race for a
cloud already in the fixed frame). That is the Andromeda feedback answered with a diagram
that names its evidence.

## 4. API and module boundary design

- **Principles:** a boundary hides a decision (Parnas — modules around things likely to
  change: the sensor driver, the optimiser, the transport); **high cohesion, low coupling**;
  depend on interfaces/messages, not on internals; make the boundary the unit of testing;
  keep pure computation separate from I/O so it can be unit-tested without hardware.
- **In ROS 2 terms** the boundary *is* the interface definition — topics/services/actions
  with typed messages, QoS as part of the contract, frame conventions (REP-105 `map → odom
  → base_link`, one authority per frame), parameters as the configuration surface, and
  node = one responsibility. Composable nodes are the reward for getting it right (same
  boundary, zero-copy inside one process). Versioning a message is a real cost — add
  fields, don't repurpose them; keep custom messages few.
- **API rules of thumb:** name the unit and the frame in the field (`range_m`, `pose in
  odom`); make defaults safe; fail loudly on misuse; keep the destructive path explicit;
  document the *contract* (rate, latency, what happens on missing data) not just the type.
- **Classic mistakes:** a god node that grew every feature; hidden coupling through shared
  globals or a shared YAML key; leaking a library type (an Open3D mesh) across a boundary;
  two publishers on one frame; "temporary" direct calls across modules that become the
  architecture.
- **Interviewer's target sentence:** "Draw the boundary around the decision that will
  change, make the message/QoS/frame contract explicit and testable, keep pure computation
  out of the I/O node so it tests without hardware — and in ROS 2 that boundary is the
  interface definition, one node per responsibility, one owner per frame."
- **`piros2` line:** both the good and the debt are visible. Good: pure modules with no
  ROS import — `se3.py`, `pose_graph.py`, `keyframe_store.py`, `depth_align.py`,
  `mesh_fill.py` — carry the maths and are the ones with the densest unit tests
  (`test_pose_graph.py`, `test_keyframe_store.py`, `test_se3.py`; the fork's suite is 123
  tests as of the SLAM plan's close); the message contracts are explicit (all image topics
  RELIABLE/KEEP_LAST-1 by measurement, `/depth/rgb` stamps identical to `/depth` by
  construction, `output_frame` on the projector); one owner per TF frame is a written rule
  the launch enforces (`publish_tf` flips off in rgbd mode; `slam:=own|rtabmap|off` picks
  exactly one owner of `map → odom`); the fork boundary itself (`piros2_world_mesh` vs the
  frozen `piros2_world`) is a decision to let one side drift. Debt: `keypoint_detector.py`
  is 1,350 lines — compass, keyframe store, relocaliser, loop detector, pose graph, TF and
  marker publisher in one node class — the god-node smell, mitigated by the extracted pure
  modules but not resolved. Say that unprompted; it is the credible answer.

## 5. Managing technical debt in a growing stack

- **Definition:** shortcuts whose cost is paid later, with interest — deliberate (a known
  hack to hit a demo) or accidental (the design didn't foresee the load). Not all debt is
  bad; *unmanaged* debt is.
- **Manage it like money:** make it **visible** (a debt register or tagged issues, with
  the *interest* named — "every new topic costs an hour of Wi-Fi tuning"), **decide**
  per item (pay now / schedule / accept and record why), **budget** a share of each cycle
  (10–20 %) to pay-down tied to work you're doing anyway (the boy-scout rule, but on the
  files you're already in), and **stop the bleeding** first (a lint rule, a test, a
  freeze) before refactoring. Prefer *strangler* migrations to rewrites: a new component
  beside the old, traffic moved gradually, the old one frozen and deleted when unused.
- **Signals it's due:** the same bug class recurring, onboarding time growing, "don't
  touch that file", the test suite as the only person who understands a module.
- **Robotics-specific debt:** parameter YAMLs nobody can explain, per-robot special
  cases in code, hardware workarounds baked into algorithms, calibration files with no
  provenance, and message types that outgrew their fields.
- **Classic mistakes:** a big-bang rewrite; refactoring without tests first; letting
  "temporary" become load-bearing without ever writing it down.
- **Interviewer's target sentence:** "Debt is fine when it's visible and chosen: name the
  interest, decide per item, budget pay-down against work you're already touching, freeze
  or lint before you refactor, and migrate strangler-style rather than rewriting."
- **`piros2` line:** verifiable practice — `todo.md` is the register (open gates and
  named levers moved there when plans close "by decision"); the fork/freeze is a
  deliberate strangler: `piros2_world` is "the frozen known-good fallback — don't backport
  the fork's changes for parity", and its two accepted consequences are *written down*
  (its `odom:=rgbd` mode still carries the raw-topic Wi-Fi collision; it predates the
  transport rework); dead things were removed rather than left (`/world/dashboard/compressed`
  publishing 2×2 mosaics to nobody at 10 Hz — removed 2026-08-12; `mesh_marker` lived one
  day; the redundant `world3d.launch.py` deleted after the merged session was proven);
  "provisional until measured" values are labelled as such (the TSDF quality knobs).
  Unpaid and admitted: the 1,350-line detector node, and the approximate intrinsics that a
  checkerboard run would replace.

## 6. Debugging methodology for field-reported issues

- **The shape of the problem:** the report is second-hand ("the map doubled in the
  decline"), the environment is gone, and the robot is 2,000 km away. So the method is
  **capture first, reason second**, and it starts *before* the incident: what a field unit
  records by default decides whether the bug is debuggable at all (§7).
- **Method:**
  1. **Get the artefacts**: logs, the bag/flight recorder, config and firmware/software
     versions, calibration, the operator's account with times, photos of the site.
  2. **Reproduce** — replay the bag through the same version on a bench; if it reproduces
     you have a unit of work, if not the difference between bench and field *is* the clue
     (timing, load, sensor, thermal, network).
  3. **Localise**: bisect in time (when did the pose first diverge from the mapping) and
     in space (which node/stage) — compare healthy vs failing runs of the *same* input;
     change one thing at a time; keep a log of what you tried.
  4. **Form a hypothesis that predicts something**, then test the prediction — the fix
     is only believed when toggling it on/off flips the symptom.
  5. **Write it down as symptom → cause → fix** where the next person will look.
- **Rules of thumb:** never trust the report's diagnosis, only its observation; "what
  changed?" (deploy, config, firmware, weather, operator) solves half; measure against
  the process's own clock when sensor stamps are suspect; the second bug hides behind the
  first.
- **Classic mistakes:** fixing on the field unit without capturing; changing three
  things; declaring "can't reproduce" without asking what the bench lacks; no record, so
  it comes back with a different name.
- **Interviewer's target sentence:** "Field bugs are debugged from artefacts: capture
  everything first, reproduce by replay on the bench, bisect in time and by stage against
  a healthy run, prove the cause by toggling it, and write the symptom-to-cause down —
  which means the real work is deciding *before* deployment what the robot records."
- **`piros2` line — held on a two-machine 'field':** the repo debugs by replay (`just
  run-bag`, gate bags, TUM sequences — no Pi needed) and by toggle-proof (the blackout
  fix: FAIL 19.7° → fix → PASS 0.41°, and toggling the fix off reproduces 19.7°). The
  2026-08-16 crawl was diagnosed the field way — measure the link (14–17 MiB/s of
  retransmit storm for ~2 frames/s), find the change (`rgb/image` remapped to the *raw*
  `/image_raw`, silently pulling 2.7 MB frames over Wi-Fi for five days), fix, re-measure
  (1.3–3.4 MiB/s single reader). The Wi-Fi incidents were read from `journalctl -b -1`
  after recovery ("the truth is in the previous boot"), and CLAUDE.md's rule "never
  diagnose an unreachable Pi as crashed without evidence: ping first" is the *don't trust
  the report's diagnosis* rule. `docs/info/troubleshooting.md` is 40+ symptom → cause
  entries, each with the fix and often the date it bit.

## 7. Logging, telemetry and observability on robots

- **Three signals:** **logs** (discrete events, levelled, structured — `rosout` /
  `rcutils` levels DEBUG…FATAL, journald for services), **metrics** (numbers over time —
  rates, latencies, queue depths, CPU/GPU/temperature/battery, disk), and **traces/
  recordings** (bags — the robot's flight recorder). Robots add a fourth: **health/
  diagnostics** — `diagnostic_msgs/DiagnosticArray` on `/diagnostics` via
  `diagnostic_updater`, aggregated by `diagnostic_aggregator` into OK/WARN/ERROR per
  subsystem so an operator (or an autonomy stack) can act on it.
- **Rules of thumb:** *rate-limit* anything per-frame (`throttle_duration_sec`,
  `RCLCPP_WARN_THROTTLE`) or the log becomes the CPU load; log the *decision and its
  evidence* ("chose CUDA provider", "tracking lost for N frames"), not the loop; include
  IDs and stamps and units; keep two clocks straight (sensor stamps vs receipt time; NTP/
  PTP between machines) or you cannot correlate; **budget disk** — ring-buffer bags with a
  trigger-to-keep on fault (the crash-recorder pattern), MCAP with compression, and know
  the write bandwidth of the SD/SSD; log versions and config at start-up so every log is
  self-describing; make health a topic, not a print.
- **Observability** = can you ask a *new* question of a past run without redeploying? A
  bag with the raw sensor topics + TF + params + logs says yes; a screenshot says no.
- **Classic mistakes:** logging at 60 Hz; unthrottled warnings on a hot path; a
  latency measured against a lagging sensor stamp; recording every topic until the disk
  fills mid-mission; time not synchronised between the payload and the vehicle.
- **Interviewer's target sentence:** "Logs for events with their evidence, throttled;
  metrics for rates and resources; a bag as the flight recorder with a disk budget and a
  keep-on-fault trigger; health on `/diagnostics` so it's actionable — and one consistent
  clock, because on a robot the timestamp is half the observation."
- **`piros2` line:** partly held. Nodes log the *decision*: `depth_estimator` logs
  `inference provider: …` precisely because the CUDA→CPU fallback is silent;
  `camera_relay` logs its relay rate every 5 s (the live view of what the camera actually
  delivers); `cloud_projector` throttles its warnings (`throttle_duration_sec=5.0`); the
  detector's `tracking lost for … / relocalized against keyframe … / snapping odometry`
  lines are grep-able evidence the gates assert on; the dashboard's stats panel measures
  rates and STALE against its own receipt clock, never `header.stamp` (the camera's
  0.73 s stamp fault — the two-clocks rule learned the hard way); the Wi-Fi watchdog is a
  flight recorder (`logger -t wifi-watchdog`, read with `journalctl -t wifi-watchdog`);
  `just camera` prints every V4L2 control current-vs-default; `just wifi`/`just status`
  are the health views; bags are MCAP with `/tf_static` QoS preserved. Not touched:
  `/diagnostics`/`diagnostic_updater`, metrics export, ring-buffer recording — the
  honest gap for a real robot.

## 8. Root cause analysis and postmortems

- **Postmortem:** a blameless written record after an incident — **timeline** (with
  clock times and who saw what), **impact**, **detection** (how did we find out; how long),
  **root cause and contributing factors** (there is rarely one), **what went well / badly /
  where we got lucky**, **action items** with owners and dates. Circulated, reviewed, and
  the actions tracked to closure. **Blameless** means the question is "why did the system
  let this happen?" not "who". Severity classes decide which incidents get one.
- **RCA techniques:** 5 Whys (stop when the answer is a *system* property, not a person);
  fishbone/Ishikawa categories (hardware, software, environment, process, people); fault
  trees for safety; "what changed" diff. Distinguish the **trigger** (the Wi-Fi AP rejected
  re-association) from the **cause** (no recovery mechanism existed) from the
  **contributing factors** (a bare `ssh` with no timeout hung the cleanup).
- **Rules of thumb:** write it within days while the evidence exists; put the evidence
  in (log excerpts, numbers); an action item that is "be more careful" is not an action
  item — it must change a system, a test, a check, or a doc; drill the fix if it's an
  operational one.
- **Classic mistakes:** naming a person; stopping at the first why; actions with no
  owner; a postmortem nobody can find later.
- **Interviewer's target sentence:** "A postmortem is a blameless timeline with evidence,
  the trigger separated from the root cause and contributing factors, and action items
  that change the system — then track them closed and, if the fix is operational, drill
  it."
- **`piros2` line — held, small scale, verifiable:** the Wi-Fi watchdog plan opens with
  "The incident record (why this exists)": two incidents with Pi-time stamps, journal
  evidence (`CTRL-EVENT-ASSOC-REJECT status_code=16`, `auth_failures=87`, 100 rejections;
  15 h of NTP timeouts with the OS healthy — load 0.03, `throttled=0x0`), what recovered
  each (reboot; power cycle), the suspect pairing (`brcmfmac` × mesh band-steering), and
  the **collateral** on the dev box (the trap's `ssh pi pkill` hung ~2 min; the orphaned
  `usb_cam` held the device). Actions changed the system: power-save off, an
  escalation-ladder watchdog, `ConnectTimeout` on every scripted ssh, `ssh -tt` +
  keepalives + sshd ClientAlive so outages reap sessions — and the fix was **drilled**
  (reproduced the `status_code=16` rejection, recovered unaided at T+426 s via the
  driver-reload rung, proving reassociation alone cannot clear it). The 2026-08-16
  transport crawl is written up the same way in CLAUDE.md and troubleshooting. Missing:
  a formal severity scheme and a postmortem template — one-person scale.

## 9. Working with hardware and field teams

- **What is different:** hardware has lead times and revisions (the software must know
  which board it's on); field teams have a mission window and no debugger; the sensor
  bench-tests fine and fails at 40 °C in dust; the operator's word is your only log if you
  didn't record. Respect runs both ways: the field tech knows what actually happens in a
  decline; the software engineer knows what the log means.
- **What works:** shared vocabulary and a shared **test plan** with acceptance criteria
  written before the trial; **checklists** for setup/teardown; a diagnostic bundle button
  ("collect everything") so a field tech can hand back evidence without knowing ROS;
  version pinning and a "known-good" release the field can always fall back to;
  hardware-in-the-loop and replay so software is tested before it meets the hardware;
  join a field day — nothing teaches like carrying the unit; feedback loops (a field
  issue tracker triaged with the software team weekly); document *how to operate*, not
  just how to build.
- **Classic mistakes:** software assuming lab conditions (Wi-Fi, a screen, a keyboard,
  a person watching); blaming "hardware" or "user error" without evidence; changing
  behaviour without telling the field; a fix that needs an engineer on site.
- **Interviewer's target sentence:** "Write the test plan and the acceptance numbers
  together before the trial, give the field a one-button evidence bundle and a
  known-good release to fall back to, and make software assume the field — no screen,
  no link, no one watching — then go and carry the unit yourself."
- **`piros2` line:** the repo is one person's lab, so "teams" is not held — say so. What
  *is* held is field-shaped software discipline: the Pi is headless, on Wi-Fi, and
  provisioned by Ansible so a reflash reproduces the machine (`changed=0` on rerun);
  camera consumers **fail loudly** (pre-flight the device, name the PID that holds it,
  `on_exit=Shutdown()`, recipes verify the launch survived warm-up); the link self-heals
  and sessions reap themselves without a person; hardware facts are measured over SSH and
  dated in `docs/info/hardware.md` (the "30 fps ceiling" fell on 2026-08-04, quoted with
  the exposure mode); a known-good fallback session (`just world`) is kept frozen; and the
  camera's persistent control state is treated as inspectable machine state
  (`just camera` / `just camera-reset`) — the "it works on the bench" trap in miniature.

## 10. Translating customer requirements into software requirements

- **The gap:** customers state outcomes in their vocabulary — "a survey-grade map of the
  stope in one flight", "it must not fly into the mesh", "the surveyor wants it in
  Deswik by Monday" — and software needs testable statements: accuracy at a range in an
  environment, latency, failure behaviour, formats, interfaces, operating envelope.
- **Method:** *why* before *what* (what decision does the customer make with the map?);
  restate as **measurable acceptance criteria** ("relative accuracy ≤ X cm over Y m in a
  Z-lit tunnel, verified against ground-control targets"); separate **functional** from
  **non-functional** (accuracy, latency, robustness, power, ops effort); prioritise
  (MoSCoW / must-should-could); write down **assumptions and out-of-scope** explicitly;
  trace each software requirement back to the customer statement so a change request can
  be costed; validate the translation *with the customer* using a prototype or a sample
  deliverable, early.
- **Rules of thumb:** every requirement needs a verification method (test, analysis,
  demo, inspection); "user-friendly" and "robust" are not requirements until they carry a
  number or a scenario; the environment *is* a requirement (dust, dark, GPS-denied,
  temperature); the deliverable format is a requirement people forget.
- **Classic mistakes:** solutioneering in the requirement ("use a 128-beam LiDAR");
  accuracy without a range or environment; ignoring the operator's workflow; no
  out-of-scope list, so scope creeps silently.
- **Interviewer's target sentence:** "Ask what decision the output feeds, restate the
  need as measurable acceptance criteria with the environment in them, split functional
  from non-functional, write the out-of-scope down, give every requirement a verification
  method, and check the translation against the customer with something they can hold."
- **`piros2` line:** no customer — but the translation step is on record twice. "Can
  Claude check the output like Playwright checks a web page?" (2026-08-18) became a
  requirement set with verification methods: a topic-to-file snapshot, replay without the
  Pi, gate bags with thresholds (5°, 0.2 m) and required log lines, offscreen renders —
  and an explicit "still needs a human" list (an unrecorded motion, the tape-measure
  scale, real exposure, taste). "What's needed to make `world_mesh` a SLAM project?"
  became a five-piece definition table with an honest ✓/◐/✗ grade per piece and per-phase
  numeric gates against RTAB-Map and TUM ground truth (slam-plan.md). Requirements with
  numbers and a verification method each, out-of-scope written down.

## 11. Communicating trade-offs to non-technical stakeholders

- **Method:** lead with the decision they need to make and the *consequence they care
  about* (time, cost, risk, what the customer sees), not the mechanism; give **2–3 options
  in a table** with the same columns (what it gets, what it costs, what it risks, when);
  put a **recommendation** and say why; use one concrete number and one analogy, not
  ten; state confidence and what would change your mind; write it down after the meeting
  in three lines.
- **Rules of thumb:** translate technical risk into schedule/quality/safety risk; never
  hide the trade-off you'd rather not discuss (it surfaces later at higher cost); "we
  can have it fast, cheap, or good — pick two" is true and unhelpful — offer the specific
  version ("ship rotation-only in two weeks and label it, or six weeks for full 6-DoF");
  a demo or a picture beats a paragraph.
- **Classic mistakes:** jargon; presenting one option as inevitable; over-precise
  estimates; forgetting to say what happens if nothing is done.
- **Interviewer's target sentence:** "Decision first, options in a table with the same
  columns, a recommendation with the reason, one number and one picture, and the risk
  stated in their currency — schedule, cost, quality — then three lines in writing."
- **`piros2` line:** the repo's docs are written for exactly that reader in one place —
  `docs/info/project-overview.md` is a single-page account for someone who wasn't there;
  the diagrams page's captions each end with "what to notice"; and the trade-off tables
  exist (setup.md's options table; the SLAM plan's Route A "cheap, teaches almost
  nothing, and the TSDF still cannot be corrected" vs Route B). The honest-scope
  paragraphs ("a panorama, not a walkable map"; "monocular depth, one room, no IMU") are
  the practice of not hiding the trade-off. Not held: presenting to an actual executive
  or customer — say so, and offer the written form as the evidence.

## 12. Support escalation handling

- **Structure:** tiers — L1 (support desk: known issues, checklists, collect the bundle),
  L2 (field/application engineers: reproduce, configuration, workarounds), L3
  (engineering: code fix, root cause) — with **severity classes** (S1 mission-blocking /
  safety, S2 degraded, S3 cosmetic) that set response targets and who is paged; a
  **ticket** that carries the diagnostic bundle, versions, and the customer's timeline;
  a **known-issues/KB** so L1 can close repeats; a **runbook** per common failure.
- **Engineering's part:** make the bundle collectable by L1 (§7, §9); triage on a
  cadence with support; give **workarounds first, fixes second** and say which is which;
  feed every L3 ticket into the backlog and, for S1/S2, a postmortem (§8); close the loop
  back to the customer with what changed and in which release; watch the *pattern* across
  tickets — three sites with the same symptom is a design problem, not three tickets.
- **Rules of thumb:** reproduce before you theorise; never ask the customer for the same
  thing twice (the bundle exists so you don't); over-communicate status on S1; the
  workaround must be safe, not just effective.
- **Classic mistakes:** engineers pulled into every ticket (no L1/L2 filtering); no
  severity, so everything is urgent; a fix shipped without telling support what changed;
  no KB, so the same 20 tickets recur.
- **Interviewer's target sentence:** "Tiers with severity classes, a diagnostic bundle so
  the first responder collects the evidence, workaround-then-fix with the difference
  stated, every escalated ticket into the backlog and the bad ones into a postmortem — and
  look for the pattern across sites, because three identical tickets is a design issue."
- **`piros2` line:** not touched — no users, no tiers. Nearest thing: the shape of the
  artefacts an escalation needs is there — `docs/info/troubleshooting.md` as the KB
  (symptom-first headings, cause, fix, date), `just status` / `just wifi` / `just camera` /
  `just stragglers` as the L1 runbook commands with `clean`/non-zero-exit outcomes, `just
  snap` as the evidence bundle, and workarounds distinguished from fixes ("until the next
  reboot", "provisional until the sweep measures them"). Say that as "I know what the
  bundle and the KB have to contain, because I built both for a robot I was the only
  support engineer for."

## What to say if asked "tell me about your engineering practice — diagrams, ADRs, reviews"

"The diagram feedback is fair and I've acted on it: `piros2` carries a Mermaid page of the
live session — dataflow, deployment, TF ownership, a sequence diagram of the recipe — with
the topic table regenerated from the running graph, and I can draw the same system as C4
context, container, component and a per-frame sequence on a whiteboard now. Decision records
I do by habit rather than by the ADR name: plans with fixed phases annotated with dates,
decisions written as decisions with the rejected alternatives and reasons, a
symptom-to-cause troubleshooting doc, and an incident record with journal evidence and a
drilled fix. What I haven't done at team scale is run a design review with six reviewers,
tiered support, or presented a trade-off to a customer — I know the shape and I'd say so
rather than pretend." Then stop.
