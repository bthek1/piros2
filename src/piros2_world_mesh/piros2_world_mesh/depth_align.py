"""
Per-frame depth scale alignment — the fusion-quality lever, pure numpy.

Live mesh plan P2, closing the todo the world fusion plan opened: the
neural depth's output scale wobbles ±4% frame to frame (measured on a
static scene), so consecutive frames disagree about where a wall is by
~±10 cm at 2.5 m — five 2 cm voxels — and a TSDF averages that into
layered shingles instead of one surface.

The measurement is the monocular-fusion community's standard move:
ray-cast the current TSDF from the frame's pose and take the median
ratio of expected over incoming depth across the overlap. Median, not
mean: the overlap includes genuinely new geometry and depth-model
failures, and the median ignores both.

The *application* is a high-pass, and that shape was learned the hard
way: conforming every frame to the map directly is unstable, because
the renderer itself reads systematically far (measured 2026-08-11:
VoxelBlockGrid's ray-cast surface sits ~1.25 voxels behind the
integrated one — voxel-proportional, truncation-independent, and it
shifts as the map accumulates, so a one-shot calibration also fails).
Any constant error in the loop compounds — the wall visibly walked
away at ~+1%/frame. So the aligner corrects only the *deviation* of
each frame's ratio from a rolling median of recent ratios: per-frame
wobble is fast and gets cancelled; renderer bias and slow drift are
absorbed into the baseline and never fed back. Absolute scale honesty
is unchanged — that belongs to the tape-measured depth_scale, not to
this loop.

Pure numpy and stdlib; the ray-cast stays with the caller (tsdf_mesher
live, tools/recon offline), so this logic is unit-testable on synthetic
planes.
"""

import numpy as np


def depth_ratio(expected, incoming, min_overlap=0.2):
    """
    Median expected/incoming over the valid overlap, or None.

    expected and incoming are same-shape depth images in metres; pixels
    that are 0/inf/nan in either are invalid. Below min_overlap valid
    fraction there is nothing to conform to (a mostly-new view) and the
    answer is honestly None, not a guess.
    """
    valid = (np.isfinite(expected) & np.isfinite(incoming)
             & (expected > 0) & (incoming > 0))
    overlap = float(valid.mean())
    if overlap < min_overlap:
        return None, overlap
    return float(np.median(expected[valid] / incoming[valid])), overlap


class ScaleAligner:
    """
    High-pass wobble corrector over a stream of depth ratios.

    Feed it (expected, incoming) per frame; it returns the scale to
    multiply the frame's depth by before integration. The rolling-median
    baseline absorbs renderer bias and slow drift (correction factors
    have median exactly 1 over the window — no net push on the map);
    only the fast per-frame deviation is corrected, clamped so a wrong
    pose or depth failure cannot fold the map.
    """

    def __init__(self, min_overlap=0.2, max_correction=0.15, window=50):
        self.min_overlap = min_overlap
        self.max_correction = max_correction
        self.window = window
        self.ratios = []

    def scale_for(self, expected, incoming):
        """Return (scale, overlap) for this frame; 1.0 when unalignable."""
        ratio, overlap = depth_ratio(expected, incoming,
                                     self.min_overlap)
        if ratio is None:
            return 1.0, overlap
        self.ratios = (self.ratios + [ratio])[-self.window:]
        baseline = float(np.median(self.ratios))
        scale = float(np.clip(ratio / baseline,
                              1.0 - self.max_correction,
                              1.0 + self.max_correction))
        return scale, overlap

    def reset(self):
        self.ratios = []
