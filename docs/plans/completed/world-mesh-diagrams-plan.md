# World mesh diagrams plan — redraw the session anatomy page

**Done 2026-08-16, same day as P0** — all four diagrams and the tables
redrawn for the transport rework; kept as the build log. The redraw's
live-graph rule earned its place immediately: see P3's annotation.

**Goal:** bring [docs/info/just_world_mesh_diagrams.html](../../info/just_world_mesh_diagrams.html)
back in step with the session it documents. The page was adapted from
`just-world-diagrams.html` on 2026-08-15 (the old file is deleted; the
world session's own diagrams live on only in git history), and the very
next day the 2026-08-16 transport rework changed the session's shape —
so every diagram on the page now shows an architecture that no longer
exists.

**Why it matters:** the page's own contract (CLAUDE.md documentation
map) is "keep it in step with the session when nodes, topics, or
measured figures change". Right now it is the most detailed — and most
wrong — description of `just world_mesh` in the repo.

## What drifted (audited 2026-08-16)

The rework this page predates, in one paragraph: the camera stream now
crosses the Wi-Fi exactly once (`camera_relay`, the session's single
Wi-Fi reader, fans it out locally on `/camera_relay/compressed`); the
`image_transport republish` node is gone — the depth estimator
republishes the exact frame it inferred on as raw `/depth/rgb`
(`publish_rgb`, stamps identical to `/depth`, so rgbd's exact sync
pairs every depth frame) and paces the whole pipeline (`max_rate: 5`);
and `cloud_projector` publishes `/points` already posed in `odom` via a
latest-TF lookup (`output_frame`), so RViz's Depth3D does no dynamic
transform at all. Backstory and measurements:
[troubleshooting.md](../../info/troubleshooting.md#a-live-session-crawls-at-2-fps-while-the-pis-wi-fi-is-saturated).

Specific claims now wrong on the page:

- **Header prose:** "seven processes (five with `odom:=kp`)" — the
  relay replaced the republisher and runs in both modes, so it is
  seven either way minus rgbd in kp mode: seven (six with `odom:=kp`).
- **Diagram 1 (dataflow):** shows `image_transport republish` producing
  a local `/image_raw`, and `/image_raw/compressed` fanning out to
  seven subscribers. Reality: `camera_relay` is the only subscriber on
  the Pi topic; everything else reads `/camera_relay/compressed`; rgbd
  reads the estimator's `/depth/rgb` twin; `/points` leaves the
  projector already in `odom`.
- **Diagram 2 (deployment):** the "apt-installed rgbd pair
  (republish + rgbd_odometry)" box, and the dotted "`/image_raw` (raw)
  ~83 MB/s never crosses the link" edge — which the old
  `rgb/image:=/image_raw` remap was *silently violating* (that is what
  saturated the link at 14–17 MiB/s). The violation-and-fix is the
  page's best lesson; the redrawn edge should carry the incident date
  rather than pretend the rule was never broken.
- **Diagram 3 (TF ownership):** `cloud_projector` is now a TF
  *consumer* (latest-only lookups, same rule as `tsdf_mesher`), and
  RViz no longer walks TF to pose Depth3D — the cloud arrives in the
  fixed frame.
- **Diagram 4 (lifecycle):** process count, and the recipe trap now
  pkills `camera_relay` and no longer needs the republisher's
  `out:=/image_raw` pattern.
- **Reference tables:** missing `/camera_relay/compressed` and
  `/depth/rgb`; `/points` frame is `odom`; the per-node cost table
  predates every 2026-08-16 measurement (single-copy Wi-Fi 1.3–3
  MiB/s, estimator paced at 5 Hz, rgbd 2.2–2.9 Hz with `delay=`
  ~1.1–1.6 s headless / 1.6–2.6 s under RViz load, per-cloud TF wait
  p50 15 ms · max 551 ms — the figures behind the odom-frame decision).

## Phases

### P0 — stop the page misleading anyone today ✓ 2026-08-16

Immediate, before any redrawing: a dated stale-notice banner at the top
of the page naming the rework and pointing at the troubleshooting
entry; CLAUDE.md's documentation-map row, README's plans table, and
todo.md's diagrams bullet annotated to match (and this plan linked from
all three). Ends with: nothing in the repo implies the page is current.

### P1 — redraw dataflow and deployment ✓ 2026-08-16

The two outright-wrong diagrams. Dataflow: relay as the single Wi-Fi
reader with the local fan-out, the estimator's twin topics, the paced
5 Hz spine, `/points` in `odom`. Deployment: republisher box gone, the
one-copy link annotation with real MiB/s, and the raw-topic-collision
incident recorded on the "never crosses the link" edge. Ends with: the
page renders in a browser (mermaid via CDN) and neither diagram
mentions `republish` or a dev-box `/image_raw`.

Done as written; the deployment diagram's dotted "never crosses the
link" edge now carries the violation dates and the fix, per the plan.

### P2 — TF ownership, lifecycle, header prose ✓ 2026-08-16

Diagram 3 gains the projector as a latest-TF consumer and loses RViz's
Depth3D transform arrow; diagram 4 gets the new process count and trap
patterns (verify against the justfile trap, not memory); the header
paragraph gets the new process arithmetic. Ends with: every node and
pkill pattern in the diagrams exists in `world_mesh.launch.py` /
`justfile` by inspection, and vice versa.

Done as written. The TF-tree diagram's edges were already correct
(ownership never changed); the caption gained the consumer story —
latest-only lookups, and no display left that walks TF at a stamp.

### P3 — reference tables and measured figures, banner off ✓ 2026-08-16

Topic/service tables regenerated from the live graph (`ros2 topic list`
/ `ros2 topic info -v` during a bounded session, not from memory); the
cost table refilled with the 2026-08-16 measurements, each figure dated
and traceable to troubleshooting.md or a fresh bounded run; the P0
banner removed. Ends with: a grep over the page for `republish`,
`image_raw` (outside the Pi-side compressed topic and the incident
note), and the old process counts comes back empty — then this plan
moves to `docs/plans/completed/` and the CLAUDE.md/README/todo
annotations from P0 are lifted.

Done — and the regenerate-from-the-live-graph rule caught a real bug
before a single diagram was drawn: `ros2 topic info` showed **two**
subscriptions on `/image_raw/compressed` where the design says one.
`cloud_projector` had been missed in the relay migration and was still
pulling its own second Wi-Fi copy; fixed (remapped to
`/camera_relay/compressed` in `world_mesh.launch.py`) and re-verified
live before the tables were written — subscription count 1, relay
fanning out to 5 local readers (6 with RViz). Fresh figures landed in
the cost table: relay 11–18 fps in a dim room at 1.3–3.4 MiB/s
single-copy, estimator 86–130 ms/frame in-session, rgbd 2.2–2.9 Hz at
quality ~420, mesher 28–107 ms/frame integrating at 1.5 cm. Banner
removed, stale notes lifted from CLAUDE.md/README/todo, page checked
well-formed.
