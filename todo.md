# TODO

- [x] cloud map isn't great, i need a better data fusion pipeline —
      done 2026-08-10 by the
      [world fusion plan](docs/plans/completed/world-fusion-plan.md):
      the live map fuses (weighted averages + min_weight), and the
      offline TSDF pipeline (`tools/recon/`) goes bag → capture → mesh
      → room.json
- [x] per-frame depth-to-TSDF scale alignment — done 2026-08-11 by the
      [live mesh plan](docs/plans/in-progress/live-mesh-plan.md) P2:
      `ScaleAligner` (high-pass against a rolling ratio baseline —
      conform-to-map drifts, learned the hard way) cuts per-frame
      placement spread 4.0% → 2.9%; the remainder is spatially
      structured model error a global scale can't touch
- [ ] affine / per-pixel depth alignment — the next fusion lever: the
      residual ~3% wobble is not a global scale (measured); a
      scale+shift or low-order fit against the ray-cast could take
      another bite
