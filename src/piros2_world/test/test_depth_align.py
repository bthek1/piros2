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

"""Unit tests for depth alignment: synthetic planes, wobble, and bias."""

import numpy as np
from piros2_world.depth_align import depth_ratio, ScaleAligner


def plane(depth, shape=(48, 64)):
    return np.full(shape, depth, dtype=np.float32)


# --- depth_ratio --------------------------------------------------------

def test_ratio_recovers_a_known_offset():
    ratio, overlap = depth_ratio(plane(2.5), plane(2.6))
    assert overlap == 1.0
    assert np.isclose(ratio, 2.5 / 2.6)


def test_median_ignores_new_geometry_and_outliers():
    expected = plane(2.5)
    expected[:10] = 0.0            # unmapped rows: no prediction there
    incoming = plane(2.5) * 1.04
    incoming[-5:] = 30.0           # a far-off depth failure
    ratio, _ = depth_ratio(expected, incoming)
    assert np.isclose(ratio, 1 / 1.04, atol=1e-3)


def test_low_overlap_returns_none():
    expected = plane(2.5)
    expected[5:] = np.nan          # almost nothing predicted
    ratio, overlap = depth_ratio(expected, plane(5.0), min_overlap=0.2)
    assert overlap < 0.2
    assert ratio is None


# --- ScaleAligner -------------------------------------------------------

def test_wobble_is_corrected_but_constant_bias_is_not_fed_back():
    """
    The high-pass contract, on the measured failure mode.

    The renderer reads a constant 1% far (bias) while the frames wobble
    ±4%. The applied scales must counter the wobble without absorbing
    the bias into a net push — the drift that walked the wall away.
    """
    rng = np.random.default_rng(7)
    aligner = ScaleAligner()
    truth = 2.5
    applied = []
    wobbles = []
    for _ in range(60):
        wobble = 1.0 + rng.normal(0.0, 0.04)
        wobbles.append(wobble)
        expected = plane(truth * 1.01)          # map + renderer bias
        incoming = plane(truth * wobble)
        scale, _ = aligner.scale_for(expected, incoming)
        applied.append(scale)
    applied = np.array(applied)
    corrected = np.array(wobbles) * applied
    # Wobble collapses: the corrected frames sit far tighter than raw.
    assert corrected.std() < 0.4 * np.array(wobbles).std()
    # No net push: cumulative correction stays near identity, so the
    # bias cannot compound into drift.
    assert abs(np.median(applied) - 1.0) < 0.01


def test_unalignable_frames_pass_through():
    aligner = ScaleAligner()
    expected = plane(2.5)
    expected[2:] = 0.0
    scale, overlap = aligner.scale_for(expected, plane(2.5))
    assert scale == 1.0
    assert overlap < 0.2
    assert aligner.ratios == []


def test_correction_is_clamped():
    aligner = ScaleAligner(max_correction=0.15)
    aligner.scale_for(plane(2.0), plane(2.0))   # baseline at ratio 1
    scale, _ = aligner.scale_for(plane(1.0), plane(2.0))
    assert scale == 0.85


def test_reset_clears_the_baseline():
    aligner = ScaleAligner()
    aligner.scale_for(plane(2.5), plane(2.5))
    aligner.reset()
    assert aligner.ratios == []
