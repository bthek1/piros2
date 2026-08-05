# World combined plan — dashboard + orientation + map, one command

> **Completed 2026-08-05**, authored and built the same day. P0–P2
> landed and were bag- then live-verified (three windows confirmed by
> hand); the deletions followed the live proof, and P3's bookkeeping
> moved this file to `completed/`. Kept as the build log.

> One session showing everything `src/piros2_world` can do, in three
> windows: the **2D dashboard mosaic** (the original piros2_world view),
> an **orientation window** (RViz: the TF axes turning with the camera,
> with the live cloud posed around them), and a **separate map window**
> (RViz: the accumulated `/world/map_points` panorama on its own). Today
> this takes two sessions (`just world` and `just world3d`) that cannot
> run at once — they would double-start the detector and depth estimator.
> The fix is a merge, not a third variant: `world.launch.py` becomes the
> one launch with all five dev-box nodes, and `world3d.launch.py` is
> retired. Stable phases, each ending with something runnable.

## The honest scope: a merge, not features

Nothing new is computed anywhere in this plan. Every node already exists;
the work is folding the projector and mapper into `world.launch.py`,
splitting one RViz config into two purpose-named ones, growing the
`just world` recipe to three windows, and then *deleting* the launch
file, config and recipe the merge makes redundant. Ending with less code
than it started with is the point — two overlapping sessions is a
maintenance trap the repo does not need.

## Preconditions, verified

| Have | Where |
| --- | --- |
| All five dev-box nodes working together in some session | `world.launch.py` (detector + depth + dashboard), `world3d.launch.py` (detector + depth + projector + mapper), both live-verified 2026-08-05 |
| The detector always estimates + broadcasts, regardless of session | world 3D plan P0/P1 — harmless when unconsumed |
| RViz env pins and the recipe skeleton (SSH camera, warm-up, `pkill -f`) | `just world3d`, [troubleshooting.md](../../info/troubleshooting.md) |
| The one-viewer-blocks, trap-cleans-all recipe pattern | every camera recipe since milestone 2 |

Constraints that shape the phases:

- **The detector and depth estimator must exist exactly once**, which is
  why merge-then-delete beats a third launch file: while both
  `world.launch.py` and `world3d.launch.py` exist, running both
  double-starts the shared nodes and two depth estimators fight for the
  GPU. Once the merge lands, the second file is a foot-gun and goes.
- **Delete only after the replacement runs.** `world3d.launch.py`, its
  recipe and `world.rviz` are removed in the same phase that proves the
  merged session live — never before, so the repo always has one working
  full session.
- **Two RViz instances are two independent processes.** Each needs its
  own config file and its own `QT_QPA_PLATFORM=xcb` pin; they do not
  share state, which is exactly what "a different graph" wants.
- **One window is the session's anchor.** The recipes block on a single
  foreground viewer and tear everything down when it closes; with three
  windows, the dashboard viewer keeps that role (it is the "original"
  view) and closing it ends the whole session. Closing an RViz window
  alone just loses that window.
- **`just world` gets heavier and that is accepted.** After the merge
  there is no dashboard-only session any more; `just orient` remains the
  lightweight alternative (detector + axes, no depth/cloud/GPU work).
- **Load is additive but small.** The mosaic + two RViz windows all
  subscribe to streams already being produced; the only duplicated work
  is JPEG decoding in each subscriber, which this machine absorbs.

## Changes to the package

```
src/piros2_world/
├── launch/
│   ├── world.launch.py            # P0: + cloud_projector + cloud_mapper (all five nodes)
│   └── world3d.launch.py          # P2: DELETED (redundant after the merge)
├── config/
│   ├── orient.rviz                # P1: NEW — TF axes + live /points + keypoint panel
│   ├── map.rviz                   # P1: NEW — /world/map_points alone, fixed frame odom
│   └── world.rviz                 # P2: DELETED (its displays live on in the two above)
└── (justfile)                     # P2: `world` grows to three windows; `world3d` recipe deleted
```

## P0 — The merge ✓ (2026-08-05)

> Landed and bag-verified: `world.launch.py` alone brings up all five
> nodes against the `bags/static1` replay — `/world/dashboard/compressed`
> at 10 Hz, `/points` at ~12.7 Hz, `/world/map_points` at 1 Hz,
> `/camera/orientation` at the bag's 15 Hz; five process starts, no
> duplicates.

Add the `cloud_projector` and `cloud_mapper` `Node` actions to
`world.launch.py`, exactly as `world3d.launch.py` declares them —
detector, venv depth estimator, dashboard, projector, mapper: the whole
dev-box side of the package in the one launch whose name says so. No
camera include (ownership rules). `world3d.launch.py` is untouched in
this phase — it only becomes redundant, not yet deleted.

**Runnable check:** against the `bags/static1` replay, `world.launch.py`
alone brings up all five nodes: `/world/dashboard/compressed`, `/points`,
`/world/map_points`, `/camera/orientation` all publishing (`ros2 topic
hz` on each), no duplicate-node warnings in the log.

## P1 — The two graphs ✓ (2026-08-05)

> `orient.rviz` and `map.rviz` split from the verified `world.rviz`
> displays; `just orient` repointed. One process lesson recorded: the
> repoint edit first anchored on the wrong of two identical recipe lines
> (`orient` and `world3d` ended in the same rviz2 command) — caught by
> checking the recipe→config mapping afterwards.

Split `world.rviz`'s displays into two purpose-named configs (the
original stays until P2 so `just world3d` keeps working during the
transition):

- `config/orient.rviz` — fixed frame `odom`; grid, TF display, the live
  `/points` cloud, and the `/keypoints/compressed` image panel. This is
  "where is the camera pointing, and what does it see" — the orientation
  graph. `just orient` is repointed here in this phase.
- `config/map.rviz` — fixed frame `odom`; grid and `/world/map_points`
  only, slightly larger point size. This is "what has been painted so
  far" — the map graph, uncluttered by the live cloud that would
  otherwise draw over it.

**Runnable check:** with the P0 launch running against the bag, opening
each config by hand (`QT_QPA_PLATFORM=xcb rviz2 -d …`) shows its own
content and nothing of the other's; `just orient` still works.

## P2 — One command, then the deletions ✓ (2026-08-05; live check passed, deletions done)

> The three-window `just world` session was confirmed live (all three
> windows, human-verified), and the deletions followed:
> `world3d.launch.py`, `config/world.rviz` and the `just world3d`
> recipe are gone. Suite green (69), justfile parses, and
> `grep -r world3d` over code and recipes finds nothing — only the
> historical plan/doc references remain, as predicted.

Grow the `just world` recipe: same skeleton (camera over SSH, warm-up
health check, `world.launch.py` in the background), then *three* viewers
— both RViz instances in the background (each with the xcb pin and its
own config) and `rqt_image_view` on the dashboard mosaic in the
foreground as the session anchor. The trap `pkill -f`s the launch, every
node pattern, and `rviz2`, so closing the dashboard window cleans up all
three windows and the Pi-side camera (bracket-pattern pkills, per the
orphaned-nodes lesson). `just run` already points at `world` and needs no
change.

Then, with the merged session proven live, delete the redundancy:
`launch/world3d.launch.py`, `config/world.rviz`, and the `just world3d`
recipe.

**Runnable check:** `just world` live — three windows: the mosaic
updating with all four panels, the orientation axes + live cloud turning
with a hand pan, the map window painting the panorama; closing the
dashboard window leaves no orphaned processes on either machine
(`pgrep -af ros2` clean on both). `just world3d` no longer exists;
`grep -r world3d` finds only historical plan/doc references.

## P3 — Bookkeeping ✓ (2026-08-05)

Docs map + current-state notes in `CLAUDE.md` and `README.md`, a line in
[roadmap.md](../../info/roadmap.md) (both still describe `just world3d`
as the way to see the 3D world — after P2 that recipe is gone), an
annotation on the completed
[world-3d-plan.md](../completed/world-3d-plan.md) noting its session was
merged into `just world` here (the build log itself stays as written —
it records its own era), suite still green, and this file moves to
`docs/plans/completed/` — the move *is* the status change; fix inbound
links.

## Out of scope, recorded so nobody wonders

- **Merging the graphs into one RViz window** — the split into two
  windows is the requirement, not a limitation.
- **Embedding RViz views in the dashboard mosaic** — same verdict as the
  world plan: 3D lives in RViz.
- **New computation of any kind** — throttling, translation, calibration
  all stay with their own todos and plans.
- **Retiring `just orient`** — it stays as the lightweight session
  (detector + axes, no GPU work); only the redundant full session goes.
