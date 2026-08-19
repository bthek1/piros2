# Perception and point clouds — the study file for section 5 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at conversation depth, the
sentence an interviewer is fishing for, and an honest **`piros2` line** — what the repo
actually does that touches the item, verified against the code before it was written. The
syllabus gives this section no priority note (it is "breadth"), but it is where `piros2` is
genuinely strong: the point cloud, the voxel map, the TSDF, the mesh completion, the ORB
frontend and the ONNX depth net were all built by hand and measured. Per the honest-claim
rule: reading ≠ holding, and the `piros2` lines say only what has been built and run — the
library-level things (PCL, OctoMap, TensorRT, 3D detectors) are *known about*, not used here.

## Mental model to carry through the whole file

```
sensor ─► points ─► (filter / downsample / normals) ─► REGISTER (pose) ─► FUSE ─► EXTRACT
                                                                          │         │
                                        TSDF · surfels · occupancy · voxel-avg   mesh · planes ·
                                                                                  objects · free space
```

Every representation here trades **memory** (dense grids die at fine resolution), **query
cost** (NN, lookup, ray traversal) and **what it can answer** (TSDF: where is the surface;
occupancy: is this cell free; mesh: render me; KD-tree: what is near p). Hovermap's
onboard/Aura split — 0.1 m voxels and a subset of rings onboard, full resolution in Aura
(§3 of [01](01_emesent-company-domain.md)) — is that trade made under a drone's compute budget.

## 1. Point cloud data structures: PCL, Open3D, PointCloud2

- **`sensor_msgs/msg/PointCloud2`** is not a list of points, it is a *byte blob with a
  schema*: `fields[]` (name, offset, datatype, count), `point_step` (bytes per point),
  `row_step`, `height × width` (height 1 = unorganised; height = rows for an organised
  depth-camera cloud), `is_bigendian`, `is_dense` (no NaN/inf), `data`. Consumers walk it
  with `sensor_msgs::PointCloud2Iterator<float>` in C++ or
  `sensor_msgs_py.point_cloud2.read_points` / `create_cloud` in rclpy. Because the layout is
  declared, a numpy structured array with the same dtype *is* the wire format — no
  per-point loop, `tobytes()` is the serialisation. RGB rides as `0x00RRGGBB` bit-cast into
  a `float32` field named `rgb` (a historic PCL convention RViz still expects).
- **PCL** (`pcl::PointCloud<PointT>`, C++ templates): `PointXYZ` is 16 bytes (x, y, z + a
  padding float for SSE alignment), `PointXYZRGB`/`PointXYZI` 32 bytes; `pcl_conversions`
  (`fromROSMsg`/`toROSMsg`) converts to/from `PointCloud2`; filters, KD-tree, normals,
  ICP, RANSAC segmentation, Euclidean clustering all live in it. Old, template-heavy,
  slow to compile, still the ROS default. **Open3D**: two APIs — *legacy*
  (`o3d.geometry.PointCloud`, Eigen vectors, CPU) and *tensor* (`o3d.t.geometry.*` on a
  `Device` that can be `CUDA:0`); the tensor side has the `VoxelBlockGrid` TSDF, ray
  casting, and GPU registration. Python-first, easy to script, 0.19 wheels ship CUDA.
- Rules of thumb: a 32-channel spinner produces ~640k–1.9M pts/s (Hovermap ST-X single/
  triple return) — 10–30 MB/s at 16 bytes before intensity/ring/time fields; a 720p depth
  image is 921k points before subsampling. Never publish a `PointCloud2` across Wi-Fi at
  frame rate.
- **Interviewer's target sentence:** "PointCloud2 is a self-describing byte buffer, so
  the cheap way to build one is to lay the points out in memory in the declared layout
  and hand over the bytes; PCL and Open3D are the two toolkits, PCL for C++/ROS
  plumbing, Open3D for scripting and GPU volumes."
- **`piros2` line:** `cloud_projector` builds `PointCloud2` by hand — `POINT_DTYPE =
  [x, y, z, rgb] <f4` (16 B/point), the four `PointField`s, `point_step = itemsize`,
  `data = cloud.tobytes()` — after unprojecting the depth image through K with numpy
  meshgrid; measured 33k–57k points in ~12 ms per cloud. Open3D's tensor API is the TSDF
  (`o3d.t.geometry.VoxelBlockGrid` on `CUDA:0`), the legacy API does decimation, Poisson
  and PLY I/O. PCL is not used anywhere in the repo.

## 2. Voxel grids, octrees, OctoMap, KD-trees, nearest-neighbour search

| Structure | Lookup | Memory | Good at | Weak at |
| --- | --- | --- | --- | --- |
| Dense voxel grid | O(1) index | volume/voxel³ — 10 m at 1 cm = 10⁹ cells | GPU fusion of a small volume (KinectFusion) | scale, sparsity |
| Hashed voxel blocks (Nießner 2013; Open3D `VoxelBlockGrid`, InfiniTAM, Voxblox, VDBFusion) | O(1) hash of block coord, then dense 8³ block | only allocated blocks | large scenes, GPU, TSDF | no hierarchy for coarse queries |
| Octree (OctoMap) | O(depth) descent, ~16 levels | prunes identical children; multi-resolution for free | occupancy at many resolutions, planning queries | slow random insert at scale, cache-unfriendly |
| KD-tree (FLANN, nanoflann, PCL, Open3D `KDTreeFlann`) | kNN / radius ≈ O(log n) in 3D | O(n), build O(n log n) | correspondence search in ICP, normals, clustering | static — rebuild on insert; FAST-LIO2's **ikd-tree** exists to fix that |

- **Voxel hashing** replaced dense grids because a room at 5 mm is billions of cells but
  the surface touches ~1 % of them; a `std::unordered_map<int3, block>` (or GPU hash) keeps
  only touched blocks. **OctoMap** (Hornung 2013) stores log-odds occupancy at each leaf
  and can answer at any depth; the standard 3D occupancy map in ROS. **KD-tree**: split
  space by alternating axes at medians; degrades above ~20 dimensions (irrelevant for
  xyz), but the build cost is what bites — scan-to-map ICP that rebuilds a tree per scan
  spends most of its time building, hence ikd-tree / voxel-hash local maps (KISS-ICP).
- **Interviewer's target sentence:** "For 3D I'd pick by the query: KD-tree for
  nearest-neighbour correspondence, a voxel hash for O(1) fusion at scale, an octree when I
  need multi-resolution occupancy; and I'd worry about rebuild cost as much as query cost."
- **`piros2` line:** two hand-written voxel structures — `piros2_world`'s `VoxelMap`
  (`floor(xyz / voxel_size)` keys → a Python dict from key to a row in preallocated numpy
  arrays, hard-capped, 5 cm) and the mesh's union-find over triangle edges in
  `mesh_fill.py` — plus Open3D's `VoxelBlockGrid` (`block_resolution=8`, 60k blocks,
  1.5 cm in `world_mesh.yaml`). Nearest-neighbour search is brute force on purpose: the
  `KeyframeStore` matches ~100 keyframes with `BFMatcher` and finds the nearest view by
  angle over a small array — the docstring says brute force is the honest choice at that
  scale. No KD-tree, no OctoMap.

## 3. Downsampling, filtering, outlier removal

- **Downsampling:** *voxel grid* (one centroid per voxel — changes point positions;
  the "approximate" variant keeps a real point), *uniform/random* (keep every Nth),
  *pass-through/crop-box* (drop outside a range or the vehicle's own body), *range
  clipping* (a monocular net's far tail, a LiDAR's near returns off the airframe).
  Choose the voxel to the *task*: registration wants a coarse cloud (KISS-ICP keeps ~one
  point per metre-scale voxel for matching), fusion wants finer; Hovermap's onboard mode
  runs 0.1 m voxels for speed and Aura re-processes at full resolution.
- **Outlier removal:** *statistical* (mean distance to k neighbours; drop points beyond
  mean + α·σ — PCL `StatisticalOutlierRemoval`, k≈50, α≈1), *radius* (fewer than n
  neighbours within r), *intensity/return-number* filters for dust and rain (mining dust
  is a real problem — multiple returns and intensity thresholds are how LiDAR sees
  through it), *temporal* confirmation (a cell must be seen N times), and mesh-side
  *component pruning* (drop tiny disconnected islands).
- Classic mistake: a voxel so coarse that the features scan matching needed are gone, or
  "outlier" removal that deletes thin real structure — wires, rebar, mesh, exactly what a
  Hovermap flies near.
- **Interviewer's target sentence:** "Downsample to what the next stage needs, not to a
  round number, and know whether your filter moves points; for outliers I'd reach for
  statistical/radius removal on the cloud and confirmation counts in the map."
- **`piros2` line:** `cloud_projector` subsamples every 4th pixel in both axes (`subsample:
  4`, ~1/16 the points) and clips `far_clip`; `cloud_mapper` drops points beyond
  `max_range` 6 m and holds back voxels seen fewer than `min_weight: 2` times (one-look
  noise never reaches the published map); the TSDF meshes only voxels with
  `weight_threshold: 3.0`; `mesh_fill.prune_small_components` removes islands under
  `min_component_triangles: 30` (373 debris components pruned in a live run); and
  `simplify_quadric_decimation` replaced the old every-Nth triangle cap that turned an
  intact mesh into a sieve (37k boundary edges at a 50 % cap, measured). The
  `ScaleAligner` uses a *median* ratio precisely because the overlap contains outliers.

## 4. Normal estimation

- Per point: take k nearest (or radius-r) neighbours, form the 3×3 covariance, the
  **eigenvector of the smallest eigenvalue** is the normal; the eigenvalue spectrum gives
  *curvature* `λ₀/(λ₀+λ₁+λ₂)` and *planarity* (Wildcat keeps a surfel only if it is planar
  enough by this spectrum). Normals have a **sign ambiguity** — orient toward the sensor
  viewpoint (you know where the scan came from) or propagate consistently over a tangent
  graph for a merged cloud without viewpoints. Organised clouds (depth images) allow
  integral-image normals in O(1) per pixel; a TSDF gives normals for free as its gradient.
- Neighbourhood scale is the knob: too small = noise, too large = blurred edges. Rule of
  thumb: radius ≈ 2–3× point spacing, or k in the 20–50 range.
- **Why it matters:** point-to-plane ICP, surfel matching, Poisson reconstruction and
  plane segmentation all need normals — and Poisson needs them *oriented*, otherwise the
  indicator function has no inside.
- **Interviewer's target sentence:** "PCA of the local neighbourhood; the smallest
  eigenvector is the normal, the spectrum tells you how planar the patch is, and the sign
  needs a viewpoint to resolve."
- **`piros2` line:** RANSAC plane normals in `tools/recon/room_layer.py` (`segment_plane`
  → `(a,b,c,d)`, normalised so `d` is in metres, then flipped so the scene lies on the
  positive side); `save_watertight` takes the TSDF's own gradient normals from
  `extract_point_cloud` and falls back to `estimate_normals()` before Poisson. No
  per-point PCA written by hand.

## 5. Registration and alignment

- **Two stages, always:** *coarse/global* (no initial guess: FPFH descriptors + RANSAC or
  Fast Global Registration, TEASER++, Scan Context for place-level, or a learned
  descriptor) then *fine/local* (ICP variants — point-to-point, point-to-plane, GICP, NDT;
  see [02_SLAM §7](02_SLAM.md)). Local methods have a small convergence basin, so the
  initial guess comes from the IMU/motion model or from the coarse stage.
- **Given correspondences**, the rigid fit is closed form: **Kabsch/Umeyama** — subtract
  centroids, SVD of the 3×3 cross-covariance, `R = V Uᵀ` with the determinant sign fixed,
  `t = c_dst − R c_src`; Umeyama adds a scale. ICP is that solve inside a loop that
  re-finds correspondences by nearest neighbour.
- **Evaluate** with fitness (inlier fraction under the correspondence distance) and inlier
  RMSE — a low RMSE on a tiny inlier set is a *worse* alignment, not a better one. Reject
  by residual (trimmed ICP, robust kernels) because descriptor matches carry a few percent
  of false pairs.
- **Interviewer's target sentence:** "Coarse to get into the basin, ICP-family to
  refine, Kabsch inside every iteration; report fitness and RMSE together, and never
  trust a fit that had to drop most of its pairs."
- **`piros2` line:** three registrations, all closed form plus reject-worst-and-refit
  rounds standing in for RANSAC: `kabsch()` on unit bearing rays for rotation-only
  odometry (the essential matrix is degenerate under pure rotation, so Kabsch on rays was
  the honest choice), `se3.rigid_transform_3d` (Umeyama without scale, returns `None` on a
  thin or inconsistent pair set) for 6-DoF relocalisation and loop-edge verification, and
  `pnp_pose` (`cv2.solvePnPRansac` EPnP + `solvePnPRefineLM`) for 3D-landmark ↔ 2D-pixel
  fits. The `ScaleAligner` is a one-parameter registration of a depth frame to the map by
  ray-cast median ratio. Iterative NN-correspondence ICP has not been written; RTAB-Map's
  `rgbd_odometry` was run and tuned, not built.

## 6. TSDF fusion, surfels, meshing (Poisson, marching cubes)

- **TSDF** (Curless & Levoy 1996; KinectFusion 2011): each voxel stores the *signed
  distance to the nearest surface along the viewing ray*, truncated to ±μ (a few voxels —
  Open3D's `trunc_voxel_multiplier`, 4 in the offline tools here), and a weight. Each new depth frame
  updates a **running weighted average** `D ← (W·D + w·d)/(W + w)`, `W ← min(W + w, W_max)`
  — that averaging is what kills per-frame depth noise; the cap keeps evidence
  displaceable. Poses are baked into every voxel at integration, so a loop closure cannot
  move an integrated surface — hence submaps, deformation, or re-fusion from kept frames.
- **Marching cubes** (Lorensen & Cline 1987) extracts the zero-crossing as triangles:
  each cube of 8 voxels is one of 256 sign patterns (15 unique cases), vertices
  interpolated along edges where the sign flips. It only ever produces surface *where the
  TSDF has evidence*, so open boundaries are honest frontiers.
- **Surfels** (ElasticFusion, Wildcat): oriented discs (position, normal, radius,
  confidence) instead of a grid — no memory for empty space, deform cheaply after loop
  closure, but no free-space knowledge and no direct mesh.
- **Poisson** (Kazhdan 2006; screened 2013): solve for an indicator function whose
  gradient matches the oriented normal field on an octree (depth ≈ resolution); the
  output is **watertight by construction**, which also means it *hallucinates* surface
  across every gap and beyond the scan — good for tools that demand a closed volume, bad
  as a record of what was seen. Ball-pivoting and Delaunay are the interpolating
  alternatives.
- **Interviewer's target sentence:** "A TSDF is a fusion accumulator — weighted-average
  signed distance per voxel — marching cubes reads the surface out of it, and Poisson is
  the closed-surface fit you use when downstream tools need watertight; know that Poisson
  invents geometry where marching cubes leaves a hole."
- **`piros2` line:** all of it, twice. Offline: `tools/recon/tsdf.py` fuses TUM fr1/desk
  (596 frames, 5.9 s, ~10 ms/frame on the GTX 1660 SUPER; marching cubes OOMs on the 6 GB
  card below ~8 mm voxels and falls back to CPU). Live: `tsdf_mesher` integrates synced
  depth + RGB into a `VoxelBlockGrid` (2 cm in `piros2_world`, 1.5 cm in the fork,
  ~52–78 ms/frame CUDA), `extract_triangle_mesh(weight_threshold=3)` on a 10–15 s timer,
  ships it as a TRIANGLE_LIST Marker, and `mesh_fill.py` then prunes debris, classifies
  every component's boundary loops — largest loop = frontier, left open; the rest =
  interior holes, fan-filled to the loop centroid under a 0.25 m radius guard (142 holes
  filled live) — and `save_watertight` writes a Poisson-closed (`depth=9`, then an
  unbounded `fill_holes`) `_closed.ply` beside the honest one. The measured warning:
  conform-to-map depth alignment was unstable because the ray-cast reads ~1.25 voxels far
  and the loop walked walls away — the `ScaleAligner` is a high-pass for that reason.
  Since the SLAM plan, the mesher keeps a thinned frame memory and *re-fuses* when the
  pose graph moves — the standard answer to "a TSDF can't be corrected".

## 7. Occupancy mapping: free vs unknown space

- Three states per cell — **occupied, free, unknown** — and the difference between free
  and unknown is the whole point: a cell is only free once a ray has *passed through it*
  to a hit beyond; never observed ≠ empty. Bayesian log-odds update per cell:
  `L ← L + log(p/(1−p))` with a hit model (~0.7) and a miss model (~0.4), clamped so a cell
  can change its mind (OctoMap's clamping thresholds). 2D: `nav2_costmap_2d` layers; 3D:
  OctoMap, the Nav2 voxel layer, Voxblox/nvblox (which derive an ESDF — Euclidean signed
  distance — from the TSDF for planning).
- **TSDF vs occupancy:** a TSDF answers "how far to the surface, which side"; occupancy
  answers "may I be here". Planners want the second (or an ESDF); meshing wants the first.
  A TSDF's truncation band carries *some* free-space evidence but not the whole ray.
- Frontier exploration (section 6) is defined on this map: frontiers are free cells
  adjacent to unknown ones. A drone's Explore mode cannot exist without an explicit
  unknown state.
- **Interviewer's target sentence:** "Free space is evidence, not the absence of hits —
  you carve it by ray casting through to each return, log-odds per cell, and the unknown
  cells are what exploration and safe planning are about."
- **`piros2` line:** not built. Nearest things: `mesh_fill.py`'s frontier concept —
  "unseen space is never invented", each component's largest boundary loop stays open —
  is the *unknown* state expressed on a mesh; the `VoxelMap` and the TSDF are occupied-only
  accumulators with confirmation counts (`min_weight`, `weight_threshold`) but no free
  cells. RTAB-Map can publish an occupancy grid, but `mapping.launch.py` was never used for
  it.

## 8. Ray casting

- **Two uses:** (a) *map update* — walk the ray from sensor origin to the return, marking
  traversed voxels free (Amanatides–Woo 3D DDA / Bresenham, cost ∝ length/voxel — a
  32-channel scan is ~10⁵ rays, so many systems subsample rays or cast at coarse
  resolution); (b) *map query/rendering* — march from a virtual camera through a TSDF until
  the sign flips, trilinearly interpolate the zero crossing (KinectFusion's raycast; also
  how you get an *expected* depth image for alignment or a virtual scan for localisation).
  Third use: line-of-sight and collision checks in planning.
- Costs and traps: ray casting is embarrassingly parallel (GPU), memory-bound on the
  voxel structure; long free rays into the sky/tunnel are the expensive ones; and the
  rendered surface sits systematically off the integrated one by a fraction of a voxel
  (interpolation and truncation) — a bias, not noise.
- **Interviewer's target sentence:** "Ray casting is how a map learns free space and how
  you read a TSDF back out as an image; it's cheap per ray and expensive per scan, so
  you subsample and parallelise."
- **`piros2` line:** `VoxelBlockGrid.ray_cast(...)` renders the *expected* depth from the
  frame's own pose every frame (`render_attributes=['depth']`, `weight_threshold=3`), and
  the `ScaleAligner` takes the median expected/incoming ratio over the overlap. The
  measured bias — the ray-cast surface sits ~1.25 voxels behind the integrated one,
  voxel-proportional, truncation-independent — is why the alignment is a high-pass. No
  free-space carving anywhere.

## 9. Segmentation: ground plane, clustering, semantic

- **Ground plane:** RANSAC a plane (3 samples, count inliers within d, iterate — 0.03 m
  and 1000 iterations are typical indoor numbers), or exploit LiDAR structure — LeGO-LOAM
  labels ground by the vertical angle between adjacent rings, range-image methods and
  Patchwork(++) handle slopes with piecewise/concentric patches. Ground removal is step
  one of most LiDAR object pipelines; iterating RANSAC (fit, remove inliers, refit) peels
  a scene one dominant plane at a time.
- **Clustering:** Euclidean cluster extraction (KD-tree radius growing, PCL), DBSCAN,
  depth-clustering on the range image (angle between neighbouring beams), connected
  components on a voxel/mesh graph. Cheap, label-free obstacle segmentation — enough for
  "something is 3 m ahead".
- **Semantic:** learned per-point/per-voxel labels — RangeNet++ (range image), Cylinder3D
  and SPVNAS (sparse conv), PointNet++/KPConv (point-based); needs labelled data
  (SemanticKITTI) and edge compute; underground scenes have almost no public labels,
  which is why geometric methods still dominate mining perception.
- **Interviewer's target sentence:** "Ground by RANSAC or ring geometry, obstacles by
  Euclidean/DBSCAN clustering, semantics by a network when you have the labels and the
  compute — and for a mine drone the geometric layer is what the safety case rests on."
- **`piros2` line:** `room_layer.py` peels up to 8 planes by iterated RANSAC
  (`segment_plane`, 3 cm, 1000 iterations, ≥2000 inliers), classifies each by `|n·up|`
  (>0.85 horizontal, <0.25 wall), picks the floor as the horizontal plane with >90 % of the
  scene *above* it (largest-plane picks the desk in fr1/desk — a real finding), snaps walls
  to a Manhattan frame and prints opposite-wall spans; `tools/verify/mesh_planes.py` uses
  the same RANSAC to measure wall *thickness* as a SLAM gate. Clustering: union-find
  connected components in `mesh_fill.py`. Semantic segmentation: none.

## 10. 3D object detection

- The LiDAR families: **PointPillars** (pillars → 2D BEV CNN, fast, the edge favourite),
  **VoxelNet/SECOND** (voxels + sparse 3D conv), **CenterPoint** (BEV heatmap of object
  centres, current strong baseline), **PV-RCNN** (point-voxel two-stage). Output: 7-DoF
  boxes (x, y, z, l, w, h, yaw) + class + score, then NMS. Trained on KITTI/nuScenes/
  Waymo — road scenes; nothing of the sort exists for stopes, vent ducting or rock bolts,
  so an underground product needs its own data or sticks to geometric obstacles.
  Deployment: TensorRT FP16; sparse-conv kernels are the hard part on Jetson.
- **Interviewer's target sentence:** "PointPillars-to-CenterPoint is the LiDAR detection
  lineage; on a drone in a mine I'd ask what a *class* buys me over a clustered obstacle
  before paying for the network."
- **`piros2` line:** not touched. Nearest thing is the geometric layer — plane
  segmentation, component clustering — and the ONNX runtime path that a detector would
  ride on (§12).

## 11. Computer vision: features (ORB, SIFT), optical flow, epipolar geometry, essential and fundamental matrices, triangulation

- **Features:** *SIFT* (Lowe 2004) — DoG scale-space keypoints, 128-float descriptor,
  L2 matching, scale/rotation invariant, slow-ish; *ORB* (Rublee 2011) — FAST corners on
  a pyramid + orientation + a learned-rotated BRIEF, **256-bit binary descriptor, Hamming
  distance** (a popcount), an order of magnitude cheaper — the SLAM default (ORB-SLAM).
  Match with brute force + cross-check or Lowe's ratio test; a chessboard's identical
  corners are the textbook failure of any descriptor.
- **Optical flow:** brightness constancy `I_x u + I_y v + I_t = 0`, one equation per pixel
  for two unknowns (aperture problem) → *Lucas–Kanade* solves it over a window (sparse KLT
  tracking, pyramidal for large motion), *Farnebäck* dense. Tracking replaces re-detection
  when frames are close; fails on blur, occlusion, and low texture.
- **Epipolar geometry:** for a calibrated pair, `x'ᵀ E x = 0` with `E = [t]× R` (5 DoF,
  Nistér 5-point + RANSAC → `cv2.findEssentialMat`/`recoverPose`, four solutions
  disambiguated by cheirality); uncalibrated, `x'ᵀ F x = 0`, `F = K'⁻ᵀ E K⁻¹`, 8-point
  algorithm. **Degenerate cases:** pure rotation (t → 0 so E → 0 — use a homography or,
  as here, solve rotation directly), planar scenes (F degenerate, H fine), and monocular
  translation is recovered only up to scale.
- **Triangulation:** with two poses and a match, intersect the rays (DLT / midpoint);
  depth uncertainty grows as `z²/(b·f)` — small baseline, far point = useless depth. That
  is why monocular VO waits for parallax before initialising and why RGB-D/stereo skip
  the problem.
- **Interviewer's target sentence:** "ORB is the cheap binary feature SLAM runs on; the
  essential matrix encodes relative pose from calibrated matches, it degenerates under
  pure rotation, and triangulation only works with baseline — those two facts decide a
  monocular pipeline's architecture."
- **`piros2` line:** the strongest section of the repo. `keypoint_detector`: `ORB_create`
  at 500 features (~14 ms/frame), `BFMatcher(NORM_HAMMING, crossCheck=True)`, a 10-frame
  match window against detection churn (~25 % of keypoints flicker), a Hamming-bit
  rejection threshold, and the design decision the essential matrix forced — pure
  rotation is degenerate, so `rays_from_pixels` unprojects through K to bearing rays and
  Kabsch gives R without any depth or triangulation. The chessboard test pins the
  descriptor failure; the blackout fix (`was_tracking`) came from a gate bag. Loop
  candidates are verified by `solvePnPRansac`. Not touched: SIFT, optical flow, F/E
  estimation, triangulation — depth comes from a network, not parallax. Earlier: the
  `piros2_vision` Canny node (`cv2.Canny(gray, 80, 160)`, ~16–20 fps on the Pi) was the
  first OpenCV-in-ROS exercise.

## 12. Deep learning inference on edge: ONNX, TensorRT, quantisation

- **ONNX** is the interchange graph (opset-versioned); **ONNX Runtime** executes it through
  *execution providers* — CPU, CUDA, TensorRT, OpenVINO, CoreML — with the important
  behaviour that a provider list is a *preference*, and unavailable providers are skipped
  **silently**. **TensorRT** compiles the graph for one GPU/precision into an engine
  (layer fusion, kernel autotuning, FP16/INT8), typically 2–5× over CUDA-EP; engines are
  not portable across GPUs or TensorRT versions, so you build on the target (Jetson) at
  install time and cache. **Quantisation:** FP16 is nearly free on tensor cores; INT8 needs
  calibration (post-training) or QAT and buys ~2× more with a small accuracy cost;
  transformers (ViTs) are more sensitive than CNNs. Jetson adds DLA cores for INT8 CNNs.
- Engineering rules: warm up (first inference builds/allocates), pin input memory, fix
  input shapes (dynamic shapes cost re-tuning), measure *in-node* latency against the
  process's own clock, and pace the pipeline to what downstream can absorb — an
  estimator that runs faster than its consumers only builds queues.
- **Interviewer's target sentence:** "ONNX for portability, TensorRT for the last 2–5× on
  the target, FP16 first and INT8 with calibration if the accuracy holds — and always log
  which provider actually ran, because the fallback is silent."
- **`piros2` line:** `depth_estimator` runs Depth Anything V2 Small (ViT-S/14, fp32 ONNX,
  518×518 input, ImageNet normalisation, relative *inverse* depth out — `z = depth_scale /
  output`, `depth_scale` tape-measured to 2.69) under `onnxruntime-gpu` with the pip
  `[cuda,cudnn]` extras: `onnxruntime.preload_dlls()` then
  `providers=['CUDAExecutionProvider', 'CPUExecutionProvider']`, and it *logs the winning
  provider* because the GPU path degrades to CPU silently — measured 72–79 ms/frame on the
  GTX 1660 SUPER vs 280–305 ms on CPU. `max_rate: 5` paces the whole session to what
  `rgbd_odometry` sustains. Weights are checksum-pinned and git-ignored (`just
  fetch-model`); the session is injectable so tests run with a fake. Not touched:
  TensorRT, quantisation, Jetson — the model is fp32 on a desktop GPU, and the honest
  next step for an embedded target would be an FP16 TensorRT engine and a smaller input.

## What to say if asked "what perception have you actually built?"

"A monocular RGB-D-style pipeline end to end, in ROS 2 Python: a neural depth node on
ONNX/CUDA, a hand-built PointCloud2 projector, a weighted-average voxel map, and a live
TSDF with marching-cubes extraction, hole classification and a Poisson-closed export —
all measured on replays. Registration is closed-form Kabsch/Umeyama/PnP with refit
rounds, not ICP; the map has no free-space or occupancy layer; and I've used Open3D and
OpenCV, not PCL or TensorRT. Where I'd be learning on the job is the LiDAR side — ring
structure, deskew, real-time C++ point pipelines." Then stop.
