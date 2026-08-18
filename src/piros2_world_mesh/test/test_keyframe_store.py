# Copyright 2026 Benedict Thekkel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Unit tests for the keyframe store: pure data, no ROS, no camera.

Synthetic ORB descriptors are just seeded random bytes — two independent
draws differ by ~128 Hamming bits, so the 64-bit match gate separates
"same view" from "different view" exactly as it does for real ORB.
"""

import numpy as np
from piros2_world_mesh.keyframe_store import KeyframeStore


def descriptors(seed, n=60):
    return np.random.default_rng(seed).integers(
        0, 256, size=(n, 32), dtype=np.uint8)


def direction(yaw_deg, pitch_deg=0.0):
    yaw, pitch = np.radians(yaw_deg), np.radians(pitch_deg)
    return np.array([np.cos(pitch) * np.cos(yaw),
                     np.cos(pitch) * np.sin(yaw),
                     np.sin(pitch)])


def test_first_frame_is_always_stored():
    store = KeyframeStore()
    assert store.maybe_add(descriptors(0), direction(0)) == 0
    assert len(store) == 1


def test_near_duplicate_view_is_refused():
    store = KeyframeStore(novelty_deg=18.0)
    store.maybe_add(descriptors(0), direction(0))
    assert store.maybe_add(descriptors(1), direction(5)) is None
    assert len(store) == 1


def test_novel_view_is_stored():
    store = KeyframeStore(novelty_deg=18.0)
    store.maybe_add(descriptors(0), direction(0))
    assert store.maybe_add(descriptors(1), direction(25)) == 1
    assert len(store) == 2


def test_cap_replaces_the_nearest_view_not_the_oldest():
    store = KeyframeStore(novelty_deg=18.0, cap=3)
    for i, yaw in enumerate((0, 40, 80)):
        assert store.maybe_add(descriptors(i), direction(yaw)) == i
    # A novel view at 100° is nearest to the 80° slot (index 2): that
    # slot is refreshed; the store neither grows nor loses coverage.
    assert store.maybe_add(descriptors(3), direction(100)) == 2
    assert len(store) == 3
    assert np.allclose(store.keyframes[2].view_dir, direction(100))


def test_clear_empties_the_store():
    store = KeyframeStore()
    store.maybe_add(descriptors(0), direction(0))
    store.clear()
    assert len(store) == 0
    assert store.match(descriptors(0)) is None


def test_match_names_the_right_keyframe_with_aligned_pairs():
    store = KeyframeStore(novelty_deg=18.0)
    store.maybe_add(descriptors(0), direction(0))
    store.maybe_add(descriptors(1), direction(40))
    query = descriptors(1)
    best, kf_idx, query_idx = store.match(query, min_pairs=12)
    assert best == 1
    assert len(kf_idx) == len(query_idx) >= 12
    # Pair alignment is the contract recovery geometry depends on.
    kf_descriptors = store.keyframes[1].descriptors
    assert (kf_descriptors[kf_idx] == query[query_idx]).all()


def test_match_refuses_an_unknown_view():
    store = KeyframeStore()
    store.maybe_add(descriptors(0), direction(0))
    assert store.match(descriptors(99), min_pairs=12) is None


def test_match_margin_refuses_ambiguity():
    # Two identical-looking keyframes (the two-posters problem): a query
    # matching both equally must yield no answer, not a coin flip.
    store = KeyframeStore(novelty_deg=18.0)
    same = descriptors(7)
    store.maybe_add(same, direction(0))
    store.maybe_add(same, direction(40))
    assert store.match(same, min_pairs=12, margin=1.3) is None


def test_empty_inputs_are_refused():
    store = KeyframeStore()
    assert store.maybe_add(None, direction(0)) is None
    assert store.match(None) is None
    assert store.match(descriptors(0)) is None  # empty store
