"""
A room's worth of keypoint memory — the store behind relocalization.

The relocalization plan's P0. Both live pose sources are memoryless (the
compass matches consecutive frames; rgbd odometry resets to identity on
loss), so a camera that flicks away and back has no way to recognise
"I've seen this view before". This store is that memory: keyframes —
descriptors plus the geometry to turn recognition into a pose — captured
while tracking is healthy, queried when it breaks.

Design points, sized for the one-room assumption:

- **Novelty-gated**: a frame is stored only when its view direction is
  at least `novelty_deg` from every stored keyframe, so the store
  covers directions instead of hoarding near-duplicates. A room at ~18°
  spacing is a few dozen keyframes; the `cap` (default 100 ≈ 1.6 MB of
  descriptors) is a ceiling, not a target.
- **At the cap, the nearest view is replaced**, not the oldest: coverage
  is the asset, and replacing the most redundant viewpoint also
  refreshes stale imagery.
- **Brute-force matching is the honest choice at this scale**: ~100
  keyframes x 500 ORB descriptors cross-checked per query is tens of
  milliseconds — bag-of-words vocabularies earn their complexity at
  thousands of keyframes, not here. Callers rate-limit queries instead.
- **A wrong match must lose to no match**: `match` demands a minimum
  pair count and a margin over the runner-up keyframe before it names a
  winner — relocalizing to the wrong wall is worse than waiting.

Pure Python + numpy + cv2, no ROS imports: the geometry columns are
whatever frame the caller works in (the detector stores bearing rays in
the odom frame for orientation recovery, 3D landmark points for 6-DoF),
and unit tests drive the whole thing synthetically.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Keyframe:
    """One stored view: recognisers plus per-keypoint geometry."""

    descriptors: np.ndarray          # (N, 32) uint8 ORB
    view_dir: np.ndarray             # (3,) unit vector, caller's frame
    rays: np.ndarray = None          # (N, 3) unit bearing rays, or None
    points: np.ndarray = None        # (N, 3) 3D landmarks, or None
    pose: np.ndarray = None          # (4, 4) capture pose, or None


class KeyframeStore:

    def __init__(self, novelty_deg=18.0, cap=100):
        self.novelty_rad = np.radians(novelty_deg)
        self.cap = cap
        self.keyframes = []
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def __len__(self):
        return len(self.keyframes)

    def clear(self):
        self.keyframes = []

    def _nearest(self, view_dir):
        """(index, angle_rad) of the stored view closest to view_dir."""
        dirs = np.stack([kf.view_dir for kf in self.keyframes])
        cosines = np.clip(dirs @ view_dir, -1.0, 1.0)
        idx = int(np.argmax(cosines))
        return idx, float(np.arccos(cosines[idx]))

    def maybe_add(self, descriptors, view_dir, rays=None, points=None,
                  pose=None):
        """
        Store the frame if its view is novel; return the slot index or None.

        At the cap a novel view replaces its nearest stored neighbour —
        the most redundant slot — so coverage never shrinks.
        """
        if descriptors is None or len(descriptors) == 0:
            return None
        view_dir = np.asarray(view_dir, dtype=np.float64)
        view_dir = view_dir / np.linalg.norm(view_dir)
        keyframe = Keyframe(
            descriptors=np.asarray(descriptors),
            view_dir=view_dir,
            rays=None if rays is None else np.asarray(rays),
            points=None if points is None else np.asarray(points),
            pose=None if pose is None else np.asarray(pose))
        if not self.keyframes:
            self.keyframes.append(keyframe)
            return 0
        nearest, angle = self._nearest(view_dir)
        if angle < self.novelty_rad:
            return None
        if len(self.keyframes) >= self.cap:
            self.keyframes[nearest] = keyframe
            return nearest
        self.keyframes.append(keyframe)
        return len(self.keyframes) - 1

    def match(self, descriptors, max_distance=64, min_pairs=12,
              margin=1.3):
        """
        Recognise the view; the winner must be unambiguous.

        Returns (keyframe_index, kf_indices, query_indices) or None.
        Cross-checked Hamming matching against every stored keyframe;
        the winner must clear `min_pairs` good matches AND beat the
        runner-up by `margin` — an ambiguous room (two lookalike walls)
        must produce no answer, not a coin flip.
        """
        if descriptors is None or len(descriptors) == 0:
            return None
        counts = np.zeros(len(self.keyframes), dtype=int)
        pairs_per_kf = []
        for i, keyframe in enumerate(self.keyframes):
            matches = [m for m in
                       self.matcher.match(keyframe.descriptors,
                                          np.asarray(descriptors))
                       if m.distance <= max_distance]
            counts[i] = len(matches)
            pairs_per_kf.append(matches)
        if not len(counts) or counts.max() < min_pairs:
            return None
        best = int(np.argmax(counts))
        second = np.partition(counts, -2)[-2] if len(counts) > 1 else 0
        if second > 0 and counts[best] < margin * second:
            return None
        matches = pairs_per_kf[best]
        kf_idx = np.array([m.queryIdx for m in matches])
        query_idx = np.array([m.trainIdx for m in matches])
        return best, kf_idx, query_idx
