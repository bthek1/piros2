# piros2_perception

Perception stack, built per [docs/plans/completed/perception-plan.md](../../docs/plans/completed/perception-plan.md).
P1 so far: `depth_estimator` — neural monocular depth on the dev box.

## The venv escape hatch (read this before running anything)

ONNX Runtime is PyPI-only; there is no rosdep key and no apt package. This
package is therefore the documented exception to the repo's apt-only rule
([docs/info/setup.md](../../docs/info/setup.md#on-sourcing-ros)):

```bash
/usr/bin/python3 -m venv --system-site-packages ~/.venvs/piros2-perception
~/.venvs/piros2-perception/bin/pip install "onnxruntime-gpu[cuda,cudnn]"
```

`--system-site-packages` is the load-bearing flag: `rclpy`, `cv2` and
`numpy` still resolve from the system/ROS installation, only the ONNX
runtime stack comes from pip. The `[cuda,cudnn]` extras pull the CUDA 13 /
cuDNN 9 libraries (~1.5 GB of `nvidia-*` wheels) so inference runs on the
dev box's GTX 1660 SUPER; on a machine without an NVIDIA GPU, plain
`pip install onnxruntime` gives the CPU-only build and the node falls back
automatically (it logs which provider won — see
[troubleshooting](../../docs/info/troubleshooting.md#onnxruntime-ignores-the-gpu-and-runs-on-cpu)
for the two ways the GPU path silently degrades to CPU). Note `/usr/bin/python3` explicitly — plain `python3` on this
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
depth, bigger = closer. Measured (2026-07-30, GTX 1660 SUPER via CUDA):
~57 ms inference, 72–79 ms/frame in-node (~13 fps). The earlier CPU
figure was 280–305 ms/frame (~3 fps).

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/image_raw/compressed` | in | decoded in-node with `cv2.imdecode` |
| `/depth` | `sensor_msgs/Image` 32FC1 | metres-ish (`depth_scale`), input frame's header |
| `/depth/preview/compressed` | JPEG | colourised relative depth, for eyeballing |

Scale honesty: monocular depth is relative. `depth_scale` (see
`config/perception.yaml`) maps model output to metres-ish values and stays
arbitrary until P2's tape-measure check.
