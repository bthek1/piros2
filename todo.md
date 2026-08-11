# TODO

- [x] cloud map isn't great, i need a better data fusion pipeline —
      done 2026-08-10 by the
      [world fusion plan](docs/plans/completed/world-fusion-plan.md):
      the live map fuses (weighted averages + min_weight), and the
      offline TSDF pipeline (`tools/recon/`) goes bag → capture → mesh
      → room.json
- [ ] per-frame depth-to-TSDF scale alignment — the next fusion lever:
      the neural depth wobbles ±4% frame to frame (measured), which is
      what shingles the swept TSDF even under RTAB-Map poses
