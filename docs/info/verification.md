# Verification without a person watching

Every plan in this repo used to end the same way: "the live gate needs a
human" — someone to run `just run`, wave the camera, and watch RViz. That
was honest, but it was also the reason gates stayed open for days after
the code was done. This page is the alternative, built and proven
2026-08-18: **the RViz window is a viewer, not the evidence.** Everything
it draws is on a topic; the motions the gates wanted a hand for are edits
to a bag we already have; and when a picture really is the evidence, a
window or a mesh can be rendered to a PNG that anyone — or an assistant —
can open.

The web analogue is Playwright: drive the app, read the DOM, take a
screenshot. Here the "DOM" is the ROS graph, the "fixture" is a bag, and
the "screenshot" is a topic frame, an X-window dump, or an offscreen
render.

## The tools (`just`, group `verify`)

| Recipe | What it does | What comes out |
| --- | --- | --- |
| `just snap [name]` | One snapshot of a *running* session: a JPEG from every image topic (the bytes on the wire are already JPEG), keypoint counts, `/points` size, live-mesh triangle count, keyframe strokes, the newest `odom → base_link`, plus every X window titled rviz/rqt/Open3D/`.ply` dumped via `xwd` → ffmpeg | `captures/verify/<name>_<stamp>/` — `summary.txt`, `summary.json`, `*.jpg`, `window_*.png`. Missing topics are reported, not fatal (`just orient` honestly has empty slots) |
| `just run-bag [bag] [args]` | The `world_mesh` session fed by a bag instead of the Pi: plays it once, runs `world_mesh.launch.py` (the relay reads the bag's `/image_raw/compressed`), opens `world_mesh.rviz`. Same nodes, topics, services — `just snap` and `just mesh-save` work against it. No camera, no Wi-Fi | A live session you can snapshot; default bag `bags/sweep3` (a real 44 s hand sweep) |
| `just gate-bags [sweep]` | Cuts a recorded sweep into the two relocalization *gate bags* (`tools/verify/make_gate_bag.py`) — see below | `bags/gate_flick`, `bags/gate_occlude`, each with a `gate.json` |
| `just gate flick` / `just gate occlude [bag]` | Headless: launches the pipeline in the gate's odom mode, records `/tf` and the frames while the gate bag plays, compares the return pass against the reference pass, greps the launch log for the lines the plan promised, exits 0/1 | `captures/verify/gate_<which>_<stamp>/` — `report.json`, `poses.csv`, `poses.png` (yaw / position / error against bag time with segments shaded), `launch.log` |
| `just mesh-views [mesh]` | Renders a saved PLY from three fixed viewpoints with Open3D's offscreen renderer — from the camera's start pose, straight down (walls as lines: doubling and drift show here), oblique | `captures/verify/mesh_<name>/{origin,top,oblique,sheet}.png` |
| `just gate-loop [own\|rtabmap\|off] [bag]` | The SLAM loop gate (SLAM plan P0/P2): the session headless in rgbd mode with the chosen backend on `bags/gate_loop` (a *palindrome* — `sweep3` out and back — so every return frame's reference is its own outbound pose); `traj_record.py` writes odom → base_link, map → odom and the optimised path to files, `traj_check.py loop` scores the BACK-vs-OUT drift of raw and corrected trajectories. PASS = the correction closes the loop tighter than odometry | `captures/verify/gate_loop_<stamp>/` — `report.json`, `poses.png`, `odom.txt`, `path_world_trajectory.txt` (or `graph_rtabmap_slam.txt` from RTAB-Map's DB), `launch.log` |
| `just gate-tum [own\|rtabmap\|off] [sequence]` | Ground truth: `tum_player.py` plays a TUM RGB-D sequence (fr1/desk by default) as camera + real depth + static TF with `depth_source:=external`; `traj_check.py ate` scores stamp-associated, Umeyama-aligned ATE against `groundtruth.txt` for the odometry and the corrected trajectory | `captures/verify/gate_tum_<stamp>/` — `ate_*.json`, `ate_*.png`, `odom.txt` |
| `just gate-mesh [bag]` | Map correction (P3): one headless run under the fork's backend with the mesher remembering frames; `~/save` → PLY + `frames.npz`; `mesh-views`, `mesh_planes.py` (RANSAC plane thickness — printed, not decisive on this scene) and `mesh_split.py` (every BACK frame paired with its mirrored-stamp OUT twin, the pairs re-integrated at odom vs corrected poses; surface-to-surface gap). PASS = corrected halves coincide better | `captures/verify/gate_mesh_<stamp>/` — `mesh.ply`, `frames.npz`, `views/sheet.png`, `mesh_split.json` |
| `just gate-map [bag]` | Persistence (P4): session one saves keyframes + graph (`~/save_map`), session two loads it, must relocalize cold, close loops against loaded keyframes and pass the loop check on its own trajectory | `captures/verify/gate_map_<stamp>/` — `launch_1.log`, `launch_2.log`, `room.npz`, `report.json`, `poses.png` |
| `just mesh-planes [mesh] [--compare other.ply]` | Wall-flatness numbers for a saved mesh: RANSAC dominant planes, inlier fraction, thickness (rms / p95 of vertex distance within a band) | printed table + `<mesh>_planes.json` |

`captures/` is git-ignored; the tools live in `tools/verify/` and run
under `/usr/bin/python3` (the PlatformIO venv shadows `python3` — the
recipes handle it) or the perception venv (`render_mesh.py`).

## Gate bags — a hand motion as a bag edit

The relocalization plan's two open gates were "flick the camera 90° away
and back" (kp mode) and "cover the lens, uncover on a known view" (rgbd
mode). Neither needs a hand once you notice both are *sequences of views
we already recorded*:

```
flick    A → gap → B → gap → A'      odom:=kp
occlude  A → gap → A'                 odom:=rgbd
```

`A`, `B`, `A'` are windows cut from a real sweep (`bags/sweep3` by
default: `A` = 0–8 s wardrobe view, `B` = 32–40 s the drawer close-up),
timelines re-stitched so header stamps stay continuous and each frame
keeps its own header→receive offset (the camera's 0.73 s stamp fault,
preserved on purpose). The gap is three seconds of frames that break
tracking: `--fill noise` (default) is coarse random blobs — hundreds of
ORB keypoints that match nothing, not even each other, the way motion
blur behaves; `--fill black` is a near-black frame with faint noise —
*no* keypoints, the way a covered lens behaves (and not pure zeros: the
detector CRC-skips byte-identical frames whole, so pure black never even
reaches it).

**The trick that makes it a measurement:** `A'` repeats part of `A`.
Whatever pose the pipeline reported for a source frame during `A` is the
reference for the same frame during `A'`, so "did the camera come back to
where it was?" is a number — the rotation angle (and in rgbd mode the
translation) between the two passes — not a judgement about axes in
RViz. `A'` is chosen so the pose at the uncover view differs from the
pose at the cover moment by more than the detector's `min_correction`
(occlude runs `A` = 0–14 s, `A'` = 3–9 s); otherwise "recognised, no
snap" is the right answer and the gate tests nothing.

`gate_check.py` puts every transform on the bag's receive-time clock —
rgbd_odometry stamps TF with the image header stamp, the kp detector
with its own wall clock, and the two are told apart by whether the stamp
lands near a seen header — pairs `A'` poses with `A` by source time,
and reports median/p90/max over `A'`'s tail (after `--settle 3` s, the
time the recovery is allowed). **PASS** = the tail is under
`gate.json`'s thresholds (5°, and 0.2 m for rgbd) *and* every expected
log line appeared (`tracking lost for`, `relocalized against keyframe`,
`snapping odometry` for rgbd) — the numbers say the pose is right, the
log says it is right *for the reason the plan built*.

### Measured 2026-08-18

| Gate | Result | Evidence |
| --- | --- | --- |
| `just gate flick` (kp) | **PASS** | `A'` tail 0.48° median / 0.64° max from `A` (82 poses); log: lost after 10 frames, `relocalized against keyframe 0: orientation snapped, correction 65.3°` — the compass had carried `B`'s 60° pan into `A'` and snapped back within the first frames |
| `just gate occlude` (rgbd, noise fill) | **PASS** | tail 0.95° / 3 cm (13 poses); `snapping odometry (Δ 0.05 m, 18.4°)` — without the snap the pose would have sat 18° off (the pre-gap pose rgbd kept publishing through the gap) |
| `just gate occlude bags/gate_occlude_black` (rgbd, black fill) | **FAIL → fixed → PASS** | First run: 19.7° median error, *no* `tracking lost` line — a featureless view yields no descriptors, `could_estimate` was False, and the loss counter never moved; the detector woke up in rgbd's reset odometry believing it was healthy. Fix: a frame with nothing to match counts as lost once tracking has ever succeeded (`was_tracking`; two unit tests). Rerun: 0.41° / 2 cm, all three log lines. Toggling the fix off reproduces the failure (19.7°) |

The third row is the argument for this page in one line: a scripted
gate found, in ninety seconds, a failure the human gate would have hit
with a real hand over the lens — and its counter-check is a rerun, not
a memory.

`just run-bag bags/sweep3` + `just snap` + `just mesh-save` +
`just mesh-views` ran the *hand sweep* gate the mesh plans wanted: 44 s
of real motion through the full rgbd session, 120k live triangles, a
723k-triangle honest PLY (+ the Poisson-closed tier), rendered from
above as one far sheet with no visible doubling at plan scale
(`captures/verify/mesh_live_20260818-175522/sheet.png`). Reading that
picture is still a judgement — but now it is a judgement about a file
anyone can reopen, made after the fact, not a memory of a window.

## The SLAM gates — measured 2026-08-18 (night)

The SLAM plan's gates reuse the same shape — a bag we have, a recorder,
a checker with a threshold — and add two things: a **yardstick** (the
same replay through RTAB-Map's SLAM node, `slam:=rtabmap`) and a
**ground truth** (TUM fr1/desk through `tum_player.py`). Numbers from
the first night:

| Gate | Raw odometry | Fork's own backend (`slam:=own`) | RTAB-Map (`slam:=rtabmap`) |
| --- | --- | --- | --- |
| `gate-loop` — loop gap over the palindrome's last 5 s (translation / angle) | 6.1 cm / 1.9° (run-to-run 3.8–14.6 cm / 1.4–19° — the pipeline is not deterministic under load) | **2.3 cm / 0.85°** PASS | 0.7–1.4 cm / 0.6–1.6° PASS |
| `gate-tum` — fr1/desk ATE RMSE (SE(3)-aligned) | 0.163 m (0.212 m on RTAB-Map's run) | **0.089 m** PASS | 0.096 m PASS |
| `gate-loop off` | — | — | FAIL ("no loop closed") — the control |
| `gate-mesh` — paired OUT/BACK surface gap (119 twin pairs), odom vs corrected | median 7.8 cm, p90 40 cm | **5.7 cm, p90 36 cm** PASS (a few degrees of residual pose at 3–6 m range) | — |
| `gate-map` — a loaded 19-keyframe room, session two's loop gap | 16.4 cm / 7.5° (its odometry was snapped into the map's frame at relocalization) | **0.7 cm / 2.4°** PASS; 1 cold relocalization, 2 loops against loaded keyframes | — |

Three things the gates taught that a window would not have: the
mesher had been starving itself (a 12–21 s refresh inline, then a
thread that still blocked under Open3D's GIL — the frame memory of a
whole loop bag came out 90 outbound / 13 return frames until the
finishing moved into its own process); RTAB-Map's
`/mapPath` poses carry no per-pose stamps (its optimised graph is read
from `~/.ros/rtabmap.db` after the node is stopped), and its DB's
per-node *odometry* is re-based on every odometry auto-reset — a
correction must be `optimised ∘ tf_odom(t_k)⁻¹` from the recorded TF
(the first scoring used the DB column and read "corrected = raw").

## Writing a gate that a script can close

When a plan phase ends with a check, write it so the check names its
evidence:

- **A number on a topic or in a log line**, with a threshold — "`/tf`
  `odom → base_link` returns within 5° of the pre-flick pose", "the log
  prints `relocalized against keyframe`". `gate_check.py` is the model:
  compare against the pipeline's *own* earlier output rather than
  against a truth nobody has.
- **A bag, named** — if the check needs motion, name the recording
  (`just record 45 <name>` once, by a person, then forever replayable)
  or the derived bag (`make_gate_bag.py` if the motion is a re-ordering
  of views we have). One human recording session converts every future
  run of the gate into `just gate …`.
- **A picture, only when geometry is the question** — `just snap` for
  a live/replayed session, `just mesh-views` for a saved surface. Say
  which view answers the question (top-down for "walls stay put").
- **"Needs a human"** is reserved for the physical world: recording a
  motion that does not exist yet, the tape-measure scale check, camera
  exposure in a real room, deciding a mesh *looks* right. Say why, and
  say what one recording would turn it into.

## Mechanics and gotchas

- **Xwayland windows dump; the root does not.** `xwd -root` fails with
  `BadMatch` under rootless Xwayland; `xwd -id <window>` works for any
  X window — rviz2/rqt (the `QT_QPA_PLATFORM=xcb` pin) and Open3D's
  viewer (`XDG_SESSION_TYPE=x11`) all are. `window_snap.sh` walks
  `xwininfo -root -tree` for titles and skips the 1×1 helper windows.
  A window that is minimised or fully covered dumps whatever the server
  holds, which may be stale — read it as "what the window held".
- **No ImageMagick, no xdotool.** XWD → PNG goes through `ffmpeg`
  (installed, decodes `xwd`). Windows are not driven; they need not be —
  layouts come from `.rviz` files and behaviours from topics/services.
- **The bag replay must carry `/tf_static`'s TRANSIENT_LOCAL QoS.**
  A gate bag written without the recorded `offered_qos_profiles` plays
  `/tf_static` volatile and a tf2 listener never matches it — the
  builder copies the source topic metadata through.
- **Warm-up before play.** The depth model takes ~10 s to load; `gate`
  waits for the estimator's `inference provider` log line, `run-bag`
  sleeps 12 s. Frames played before that are simply lost.
- **rviz2 sometimes needs two SIGTERMs** (observed 2026-08-18: the first
  `pkill` logged the kill, the process stayed; the second ended it).
  Session recipes end when the window closes, so their traps never
  depended on it — but a scripted close should `pkill` again after a
  pause and confirm with `just stragglers`.
- **Don't name a node's path on the same command line as a session
  recipe.** The recipe's EXIT trap `pkill -f`s node patterns such as
  `piros2_world_mesh/[k]eypoint_detector`; a shell whose own command
  line contains `src/piros2_world_mesh/piros2_world_mesh/keypoint_detector.py`
  matches and gets killed with the session (exit 144). Bit twice while
  building this page.
- **A synthetic gap must not be byte-identical frames** — the detector's
  usb_cam duplicate CRC skip drops them before anything runs. Both fills
  carry per-frame noise.
