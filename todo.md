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
- [ ] SLAM follow-ups ([docs/plans/completed/slam-plan.md](docs/plans/completed/slam-plan.md),
      done 2026-08-19): record a walked loop (`just record 60 loop1`,
      ending on the start view) to test the translation axis the
      palindrome bag can't; redraw the diagrams page for the SLAM
      session (new node internals, `/world/trajectory[_odom]`,
      `/world/keyframe_graph`, the `map` frame, the mesher's worker
      process — it carries a dated addendum for now); a Sim(3) graph if
      the residual ever proves to be scale; store hygiene for the
      keyframe cap now that loop keyframes are forced in
- [ ] `just gate occlude` variance under the SLAM default: one run
      2026-08-19 FAILED at 6.8° tail (snap against the only keyframe A's
      14 s stores — a slow pan barely crosses the 18° novelty), the rerun
      PASSED at 0.59°; store more than one keyframe in A (novelty 12°?
      or a time-based keyframe) so the snap has choices, then re-measure
      a few runs
- [ ] the mesher's refresh cost: 1.4–2.6 M triangles at 1.5 cm on a
      close scene → 22–32 s per refresh in the worker process (the
      marker updates every ~30 s) — the provisional TSDF values above
      now have a measured cost against them; a coarser live voxel or a
      cheaper decimation is the lever
- [ ] diagrams page
      ([just_world_mesh_diagrams.html](docs/info/just_world_mesh_diagrams.html),
      redrawn 2026-08-16 for the transport rework —
      [world-mesh-diagrams-plan](docs/plans/completed/world-mesh-diagrams-plan.md)):
      replace the provisional TSDF quality figures with measured ones
      once the sweep runs



- [ ] project in C++