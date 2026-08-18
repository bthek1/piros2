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
    node_id: int = -1                # pose-graph node (SLAM plan P1), or -1


class KeyframeStore:

    def __init__(self, novelty_deg=18.0, cap=100, novelty_m=0.0):
        self.novelty_rad = np.radians(novelty_deg)
        # SLAM plan P1: a second novelty axis. View direction alone is
        # the right gate for a compass (rotation-only), but a camera
        # that walks 2 m sideways with the same heading sees a new place
        # and would store nothing. When novelty_m > 0 and the caller
        # gives a position, a view is also novel when every stored
        # keyframe *facing the same way* is at least novelty_m away.
        self.novelty_m = novelty_m
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
                  pose=None, force=False):
        """
        Store the frame if its view is novel; return the slot index or None.

        At the cap a novel view replaces its nearest stored neighbour —
        the most redundant slot — so coverage never shrinks. `force`
        skips the novelty gate (a verified loop closure stores its frame
        regardless — the graph needs a node exactly there).
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
        if not force and angle < self.novelty_rad \
                and not self._far_from_same_view(keyframe):
            return None
        if len(self.keyframes) >= self.cap:
            self.keyframes[nearest] = keyframe
            return nearest
        self.keyframes.append(keyframe)
        return len(self.keyframes) - 1

    def _far_from_same_view(self, keyframe):
        """Tell whether every same-heading keyframe is >= novelty_m away."""
        if self.novelty_m <= 0.0 or keyframe.pose is None:
            return False
        dirs = np.stack([kf.view_dir for kf in self.keyframes])
        same_view = np.arccos(np.clip(dirs @ keyframe.view_dir, -1.0, 1.0)) \
            < self.novelty_rad
        positions = [kf.pose[:3, 3] for kf, near in
                     zip(self.keyframes, same_view)
                     if near and kf.pose is not None]
        if not positions:
            return True
        dists = np.linalg.norm(np.stack(positions) - keyframe.pose[:3, 3],
                               axis=1)
        return bool(dists.min() >= self.novelty_m)

    def match(self, descriptors, max_distance=64, min_pairs=12,
              margin=1.3, exclude=None):
        """
        Recognise the view; the winner must be unambiguous.

        Returns (keyframe_index, kf_indices, query_indices) or None.
        Cross-checked Hamming matching against every stored keyframe;
        the winner must clear `min_pairs` good matches AND beat the
        runner-up by `margin` — an ambiguous room (two lookalike walls)
        must produce no answer, not a coin flip. `exclude` names slots
        left out of the contest (loop detection skips the keyframes just
        stored — matching your own last view is not a revisit).
        """
        if descriptors is None or len(descriptors) == 0:
            return None
        exclude = set() if exclude is None else set(exclude)
        counts = np.zeros(len(self.keyframes), dtype=int)
        pairs_per_kf = []
        for i, keyframe in enumerate(self.keyframes):
            if i in exclude:
                pairs_per_kf.append([])
                continue
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

    # ------------------------------------------------------------------
    # Persistence (relocalization plan P3): the room survives the session.
    # A .npz of plain arrays — no pickle, so a map file is data, never
    # code. Keyframes are ragged (N differs per frame), hence one array
    # group per keyframe plus a small manifest of the store's shape.

    def to_arrays(self):
        """Flatten the store into {name: ndarray} for np.savez."""
        arrays = {
            'manifest_count': np.array(len(self.keyframes)),
            'manifest_novelty_rad': np.array(self.novelty_rad),
            'manifest_novelty_m': np.array(self.novelty_m),
            'manifest_cap': np.array(self.cap),
        }
        for i, kf in enumerate(self.keyframes):
            arrays[f'kf{i}_descriptors'] = kf.descriptors
            arrays[f'kf{i}_view_dir'] = kf.view_dir
            arrays[f'kf{i}_node_id'] = np.array(kf.node_id)
            for column in ('rays', 'points', 'pose'):
                value = getattr(kf, column)
                if value is not None:
                    arrays[f'kf{i}_{column}'] = value
        return arrays

    def save(self, path):
        """Write the store to `path` (.npz)."""
        np.savez_compressed(path, **self.to_arrays())

    @classmethod
    def from_arrays(cls, arrays):
        """Rebuild a store from to_arrays() output (or a loaded npz)."""
        store = cls(novelty_deg=np.degrees(float(arrays['manifest_novelty_rad'])),
                    cap=int(arrays['manifest_cap']),
                    novelty_m=float(arrays.get('manifest_novelty_m', 0.0)))
        for i in range(int(arrays['manifest_count'])):
            def col(name, i=i):
                key = f'kf{i}_{name}'
                return np.asarray(arrays[key]) if key in arrays else None
            node_id = col('node_id')
            store.keyframes.append(Keyframe(
                descriptors=col('descriptors'), view_dir=col('view_dir'),
                rays=col('rays'), points=col('points'), pose=col('pose'),
                node_id=-1 if node_id is None else int(node_id)))
        return store

    @classmethod
    def load(cls, path):
        """Read a store written by save(); allow_pickle stays False."""
        with np.load(path, allow_pickle=False) as data:
            return cls.from_arrays(dict(data.items()))
