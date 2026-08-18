# TODO

- [ ] rewrite in C/C++
- [ ] world_mesh TSDF values (from the closed
      [world mesh plan](docs/plans/completed/world-mesh-plan.md)): the
      provisional 1.5 cm / 120k triangles / 15 s are still unmeasured —
      no hand needed any more: `just run-bag bags/sweep3` replays a
      real sweep, `just mesh-save` + `just mesh-views` show the result
      ([verification.md](docs/info/verification.md)); the sweep and
      the in-session save both ran that way 2026-08-18, the tuning
      loop over the replay is what's left
- [x] a wall-flatness number for the sweep gate — `just mesh-planes`
      (`tools/verify/mesh_planes.py`, 2026-08-18) reports RANSAC plane
      inlier fraction and thickness; measured useless on `sweep3` (a
      close-range wall + object, no dominant plane) — the loop-bag
      surface metric is `mesh_split.py` instead (see below)
- [ ] SLAM plan follow-ups ([docs/plans/in-progress/slam-plan.md](docs/plans/in-progress/slam-plan.md)):
      run `just gate-map` (P4, written, unrun); make `mesh_split.py`'s
      halves comparable so P3's PASS is credible (equal frame counts /
      region of interest / per-source-frame scoring); then flip the
      claims (README, project-overview, diagrams page, the `SLAM`
      GitHub topic, `docs/to_learn/emescent.md`); record a walked loop
      (`just record 60 loop1`) to test the translation axis
- [ ] the mesher's refresh cost: 0.7–1.6 M triangles at 1.5 cm on a
      close scene → 12–21 s per refresh even off-thread, ~160 ms/frame
      integration under contention — the provisional TSDF values above
      now have a measured cost against them
- [ ] diagrams page
      ([just_world_mesh_diagrams.html](docs/info/just_world_mesh_diagrams.html),
      redrawn 2026-08-16 for the transport rework —
      [world-mesh-diagrams-plan](docs/plans/completed/world-mesh-diagrams-plan.md)):
      replace the provisional TSDF quality figures with measured ones
      once the sweep runs



- [ ] project in C++