# piros2_perception

Perception stack, built per [docs/plans/in-progress/perception-plan.md](../../docs/plans/in-progress/perception-plan.md).
P1 so far: `depth_estimator` — neural monocular depth on the dev box.

## The venv escape hatch (read this before running anything)

ONNX Runtime is PyPI-only; there is no rosdep key and no apt package. This
package is therefore the documented exception to the repo's apt-only rule
([docs/info/setup.md](../../docs/info/setup.md#on-sourcing-ros)):

```bash
/usr/bin/python3 -m venv --system-site-packages ~/.venvs/piros2-perception
~/.venvs/piros2-perception/bin/pip install onnxruntime
```

`--system-site-packages` is the load-bearing flag: `rclpy`, `cv2` and
`numpy` still resolve from the system/ROS installation, only `onnxruntime`
comes from pip. Note `/usr/bin/python3` explicitly — plain `python3` on this
machine is PlatformIO's venv
([docs/info/troubleshooting.md](../../docs/info/troubleshooting.md#rqt-tools-crash-with-no-module-named-yaml)).

Because colcon hardcodes `#!/usr/bin/python3` into entry-point scripts,
`ros2 run piros2_perception depth_estimator` would miss the venv. The node
is run as a module under the venv interpreter instead — `just depth` owns
the exact invocation:

```bash
~/.venvs/piros2-perception/bin/python -m piros2_perception.depth_estimator
```

## Model weights

Fetched once, checksum-pinned, git-ignored:

```bash
just fetch-model    # → models/depth_anything_v2_small.onnx (~99 MB)
```

Depth Anything V2 Small (ViT-S/14), fp32 ONNX export from
`onnx-community/depth-anything-v2-small`. Input `pixel_values`
[1,3,518,518], output `predicted_depth` [1,518,518] — relative *inverse*
depth, bigger = closer. Measured ~300 ms/frame on the dev-box CPU.

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/image_raw/compressed` | in | decoded in-node with `cv2.imdecode` |
| `/depth` | `sensor_msgs/Image` 32FC1 | metres-ish (`depth_scale`), input frame's header |
| `/depth/preview/compressed` | JPEG | colourised relative depth, for eyeballing |

Scale honesty: monocular depth is relative. `depth_scale` (see
`config/perception.yaml`) maps model output to metres-ish values and stays
arbitrary until P2's tape-measure check.
