# World mesh plan — `just world_mesh`, the mesh-first session

> **In progress, started 2026-08-12.** Duplicate the `just world`
> session under a new name — `just world_mesh` — so it can diverge
> toward the goal the current session only gestures at: **a 3D surface
> as the final output**. The duplicate becomes the day-to-day target as
> `just dev`; `just world` (and its `just run` alias) stay exactly as
> they are, the known-good baseline to fall back to.
>
> **Status 2026-08-12 (same day):** P0–P3 built and unit-tested in one
> sitting, then **rebuilt as a full package fork by decision** —
> `src/piros2_world_mesh`, see the revised section below (160 tests
> green, both forks' suites). Open: the **live gates** — P2's hand
> sweep (which is also live-mesh P3's open gate) and P3's in-session
> `just mesh-save` → `just view-mesh` check — plus P4's bookkeeping
> once those pass. P2's mesh-first values are provisional until the
> sweep measures them.

The motivating decision (2026-08-12 discussion): for a surface,
`tsdf_mesher` beats `cloud_mapper` by construction — a point cloud is
never a surface — but the *live* mesh is deliberately compromised for
in-session viewing (2 cm voxels, the 60k-triangle cap, Marker-only
output that dies with RViz). A mesh-first session gets to change those
defaults, defaults `odom:=rgbd` for poses that survive translation, and
grows a way to keep the surface when the session ends.

## What "duplicate" means here (revised 2026-08-12, same day)

**A full package fork: `src/piros2_world_mesh`.** The plan's first
draft argued for duplicating only the session artefacts (recipe +
launch + rviz) around shared nodes, and the first build that day did
exactly that — an include-wrapper launch plus an `extra_params`
overlay threaded through `world.launch.py`. **Decision: rejected** in
favour of a real project the mesh work can freely diverge — nodes
included — without ever touching `piros2_world`. The wrapper bits were
unwound the same day (`world.launch.py` is back to its pre-fork
shape); this section records the final shape:

- `src/piros2_world_mesh` is a copy of `piros2_world` — the four nodes
  (`keypoint_detector`, `dashboard`, `cloud_mapper`, `tsdf_mesher`),
  `se3.py`/`depth_align.py`, and the full test suite, imports renamed.
  Node names, topics and services are unchanged: it is an alternative
  implementation of the same session, not a second one.
- `piros2_perception` stays shared (depth estimator + cloud projector
  belong to perception, not to either world fork).
- `world_mesh.launch.py` is a standalone copy of `world.launch.py`
  running this package's nodes with this package's full
  `world_mesh.yaml` (forked from `world.yaml`, deviations marked
  MESH-FIRST); `world_mesh.rviz` is a copy of `world.rviz`.
- The `world_mesh` recipe matches the `world` recipe shape end to end
  (SSH camera launch, warm-up check, RViz foreground, EXIT trap), with
  patterns for this package's process names.

**The cost is real and accepted:** a `dashboard` fix now exists twice,
and the forks will drift — that drift is the point. If a change is
wanted in both, make it in both; `just test` runs both suites.

**The two sessions are alternatives, never run together.** They start
identically-named nodes on the same topics; running both would
double-start everything and fight over the camera. Nothing enforces
this beyond the same convention that already governs `world` vs
`orient`.

## Phases

### P0 — the faithful duplicate ✓ runnable

**Built 2026-08-12, twice.** First as the include-wrapper this phase
originally sketched (an `extra_params` overlay argument on
`world.launch.py`); superseded the same day by the package-fork
decision — `src/piros2_world_mesh` copied from `piros2_world`, imports
renamed, its own standalone launch/config/rviz, the wrapper bits
reverted out of `piros2_world`. Launch validates (`ros2 launch
piros2_world_mesh world_mesh.launch.py --show-args`), the module
imports under the venv interpreter, and the fork's full test suite
runs in `just test`. **Found while copying the recipe:** `just world`
passes its args to the *camera* launch only — `just world odom:=rgbd`,
as README described, never reached `world.launch.py` (launch silently
accepts unknown args, so nothing errored). `world_mesh` routes args to
both launches; `world` is left as-is for now — its README claim was
the bug, and README now points at `just dev`.

Create the three artefacts, changing nothing behavioural:

- `src/piros2_world/launch/world_mesh.launch.py` — includes
  `world.launch.py`, passes `odom` through unchanged (still defaulting
  `kp` at this phase), loads `world_mesh.yaml` (empty overrides for
  now).
- `src/piros2_world/config/world_mesh.yaml` — created, initially just
  a header comment saying what it is.
- `src/piros2_world/config/world_mesh.rviz` — copy of `world.rviz`.
- justfile: `world_mesh *args:` recipe in the `test` group, copied
  from `world` with the two file names swapped. The rviz pkill pattern
  (`[r]viz2 -d src/piros2_world/config`) already covers both configs;
  every other trap pattern is identical.
- `colcon build --symlink-install` so the new launch/config install.

**Check:** `just world_mesh` runs a session indistinguishable from
`just world` (same nodes, same panels, same teardown); close RViz,
then `just stragglers` reports `clean` on both hosts. `just world`
still works, untouched.

### P1 — `just dev` points at it ✓ runnable

**Built 2026-08-12** exactly as below; `just --list` shows both, each
with a one-line doc (just uses the *last* comment line as the doc —
learned by getting it wrong).

- justfile: `dev *args: (world_mesh args)` — a one-line dependency
  alias in the same group, exactly how `run` aliases `world` today.
  `run` keeps meaning `world`; the plan does not touch it.
- Recipe comments and `just` group listing say which is which: `run` =
  the stable session, `dev` = the mesh-first session under active
  change.

**Check:** `just dev` opens the world_mesh session; `just run` still
opens the classic one.

### P2 — mesh-first defaults (the reason the fork exists) ✓ runnable

**File-side built 2026-08-12**: `odom` defaults `rgbd` in
`world_mesh.launch.py`, the recipe trap carries `rgbd_[o]dometry` and
`out:=/[i]mage_raw` patterns, and `world_mesh.yaml` (the fork's own
full config since the package decision) sets voxel_size 0.015 /
max_triangles 120000 / refresh_period 15.0, marked MESH-FIRST and
provisional in the file. CloudMap defaulted off in `world_mesh.rviz`
(the one line that differs from `world.rviz`). **The live sweep gate
is open** — run it, record the measured costs here, and settle the
provisional values.

Now diverge, in `world_mesh`'s own files only:

- `world_mesh.launch.py` defaults **`odom:=rgbd`** — 6-DoF poses so
  the surface stops smearing the moment a hand-pan carries real
  translation (RTAB-Map measured 0.9 m of arm-arc in a "rotation-only"
  sweep). The keypoint compass stays available with
  `just dev odom:=kp`.
- **Teardown contract:** rgbd mode adds two processes
  (`rgbd_odometry`, the image republisher), so the `world_mesh` trap
  gains their pkill patterns in the same change — the current `world`
  trap does not carry them, which is fine while rgbd is opt-in there
  but wrong the moment it becomes a default.
- `world_mesh.yaml` overrides `tsdf_mesher` for quality over glance:
  smaller `voxel_size` and a raised triangle cap, values chosen by
  measurement on the dev box (the 6 GB card OOMs marching cubes below
  ~8 mm offline; the live budget will land coarser — measure, don't
  guess). Re-mesh period revisited under the new cost.
- `world_mesh.rviz` becomes mesh-centric: LiveMesh and TF prominent,
  CloudMap defaulted off (the mapper still runs — toggling it back on
  is one click), image panels kept but small.

**Check (this is also live-mesh P3's open hand-sweep gate — closing
it here closes it there; annotate that plan):** a lit-room hand sweep
with real translation under `just dev`; walls stay put while the
camera moves, the mesh accumulates without radial smear, and one live
click of `/tsdf_mesher/reset` starts the surface over. Cost figures
(integrate ms/frame, re-mesh seconds, triangle count) recorded here.

### P3 — the surface survives the session ✓ runnable

**Built and unit-tested 2026-08-12**: `~/save` on `tsdf_mesher`
(extract + fill_holes shared with the refresh via a new
`extract_mesh_arrays()`), writing `meshes/live_<stamp>.ply` as
**hand-written ASCII PLY** (`ply_from_mesh`, a pure function — chosen
over `o3d.io.write_triangle_mesh` so the path needs no open3d and
tests on the system interpreter, the projector's hand-built-PointCloud2
lesson again); `just mesh-save` recipe in the recon group; three new
tests (PLY geometry/colour serialisation, coordinate round-trip, the
save-before-anything guard). **The live check is open**: save during a
real `just dev` session, open the file with `just view-mesh`.

The final output today dies with RViz. Fix that:

- `tsdf_mesher` gains a **`~/save`** Trigger service: extract now,
  write `meshes/live_<stamp>.ply` (PLY because RViz's assimp loads it
  and `view-mesh` exists; the offline pipeline already git-ignores
  `meshes/`). The service is additive and shared — `just world`
  inherits it harmlessly, same as `~/reset`.
- justfile: `mesh-save` recipe — one `ros2 service call` wrapper so
  nobody types the service path.
- Unit test: the extract-to-PLY path as a pure function against a tiny
  synthetic volume, matching the existing marker/alignment test style.

**Check:** during a `just dev` session, `just mesh-save`; after
teardown the file opens in `just view-mesh` and shows the swept scene.

### P4 — bookkeeping ✓ checkable

- CLAUDE.md current-state paragraph and docs map row for this plan;
  README session wording (`just dev` exists, what it is); the
  diagrams page (`docs/info/just-world-diagrams.html`) gains a note —
  or a fifth-figure variant — for the rgbd-default session once P2
  lands, per its "keep in step" contract.
- Move this file to `docs/plans/completed/` and fix inbound links —
  the move is the status change.

**Check:** `just test` green, docs agree with the justfile, no doc
implies `world_mesh` does something it doesn't.

## The honest scope

- **This plan does not improve mesh quality beyond poses + budget.**
  The measured next ceiling after 6-DoF poses is the depth model's
  per-frame scale wobble and its spatially-structured residual
  (affine alignment is the named todo) — out of scope here, same as
  in the live mesh plan.
- **The best surface is still the offline one.** RTAB-Map-posed
  `fuse-capture --trajectory` remains the quality ceiling; `just dev`
  narrows the gap live but P3's PLY is a convenience export, not a
  replacement for the offline pipeline.
- **Two sessions cost maintenance.** Every change to shared nodes now
  gets glanced at under both sessions; the include-not-copy launch
  keeps that cheap, the copied rviz config is the one artefact that
  can silently drift — if the layouts converge again, delete the
  duplicate rather than maintaining two identical files.
