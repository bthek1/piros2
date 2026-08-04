# TODO

- [ ] reduce compute

- [x] keypoint matching: don't just use the last frame — keep a record of
      the last 10 frames and match against it (done 2026-08-04:
      `match_window` parameter, default 10; frame-to-frame was ~75/100,
      lost mostly to detection flicker at the feature cap)