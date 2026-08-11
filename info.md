
```
Don't pick one representation — you need three, at different stages. Trying to make a single structure serve capture, fusion, and output is the usual mistake.

## 1. Capture — the only data you can't regenerate

```python
Frame:
  t:     float          # timestamp, needed for IMU/depth sync
  rgb:   uint8[H,W,3]   # JPEG on disk
  depth: uint16[H,W]    # millimetres, 16-bit PNG. Store RAW — never the filtered version
  K_rgb, K_depth: float64[3,3]
  T_wc:  float64[4,4]   # world <- camera, filled in later by SLAM/BA
```

Keep every keyframe plus its pose. Everything downstream is derived, so when your fusion params or pose graph change you re-fuse instead of re-scanning. Follow the ScanNet/TUM RGB-D layout so you get existing tooling for free.

Poses live separately as a **pose graph**: nodes = keyframe SE(3), edges = relative transform + 6×6 information matrix. Optimise with GTSAM/g2o (or Open3D's built-in). Store poses as a separate file from frames — you'll rewrite them many times.

## 2. Fusion — sparse voxel-hashed TSDF

This is my recommendation over the alternatives:

```python
# hash map: int3 block coord -> block of 8^3 voxels
Block:
  tsdf:   float16[8,8,8]   # truncated signed distance, normalised to [-1,1]
  weight: uint16[8,8,8]    # running fusion weight
  color:  uint8[8,8,8,3]
```

Why: the running weighted average is what kills depth sensor noise. A raw accumulated point cloud just piles up 30 noisy copies of every surface and you can't mesh it cleanly.

Why *sparse*: a 5×5×3 m room at 5 mm voxels is 6×10⁸ voxels dense (~5 GB). Only ~2% is within the truncation band of a surface, so hashing gets you to ~100 MB. Voxel size 4–8 mm is the right range for a room; below 4 mm you're modelling sensor noise.

Use `open3d.t.geometry.VoxelBlockGrid` — CUDA-backed, does integration and raycasting, saves you a month.

The alternative is surfels (ElasticFusion-style: position, normal, radius, confidence). Better for non-rigid deformation when loop closures fire, worse for meshing. Not worth it unless you're doing large multi-room scans.

## 3. Output — mesh + structural scene graph

Marching cubes off the TSDF → triangle mesh, then texture from the keyframes (per-vertex colour is fine to start; a UV atlas with per-face best-view selection is a big quality jump later). glTF/GLB for delivery, PLY for intermediate.

But a mesh alone isn't a *room model*. Add a structural layer:

```json
{
  "up": [0,0,1], "units": "m", "handedness": "right",
  "planes":   [{"id":"wall_3","n":[1,0,0],"d":-2.41,"boundary":[[...]],"label":"wall"}],
  "openings": [{"type":"door","plane":"wall_3","polygon":[[...]]}],
  "objects":  [{"class":"chair","obb":{"c":[...],"R":[...],"e":[...]},"mesh":"obj_07.glb"}]
}
```

RANSAC plane fitting on the mesh gives you walls/floor/ceiling; snapping walls to a Manhattan frame makes the output look intentional rather than scanned. This layer is what makes the result measurable and editable.

## Conventions — decide now, write them down

Metres, right-handed, Z-up gravity-aligned (rotate out of the first camera frame using the IMU or the floor plane normal). Store `T_wc` not `T_cw` and put it in the field name. Half the bugs in reconstruction pipelines are inverted transforms.

## The alternative worth knowing

If the goal is photorealistic viewing rather than geometry, 3D Gaussian Splatting is a different representation entirely — N gaussians of `{μ[3], q[4], s[3], α, SH[48]}`. Far better renders, but you can't measure, collide, or edit it, and depth is only a weak prior. If someone will ever take a dimension off this model, stay with TSDF.
```


What you already have from piros2
Camera intrinsics K — you built c922_720p_approx.yaml and project through K in cloud_projector. The doc's K_rgb/K_depth is the same 3×3 matrix, just one per sensor.
Depth maps and back-projection to points — that's exactly your P1/P2 pipeline (/depth → PointCloud2).
Rigid transforms as 4×4 matrices — TF's base_link → camera_optical_frame chain is the same object as T_wc. You've also already been bitten by the direction convention (the optical-frame rotation), which is the doc's "half the bugs are inverted transforms" point.
Why raw accumulated point clouds are unsatisfying — your cloud_mapper voxel dict is a crude ancestor of the TSDF: latest-wins per voxel instead of a weighted average. The doc's section 2 is the principled version of what you hacked.
Keypoints, matching, rotation estimation — your ORB + Kabsch work is the front end of the SLAM system that fills in T_wc.
What to learn, per section
Section 1 (capture + pose graph) — the core gap is how poses are represented and optimised:

SE(3) and SO(3) — the group of rigid transforms. You use them daily via TF; the new part is treating them as things you can average, interpolate, and optimise. Learn: rotation representations (matrix vs quaternion vs axis-angle), composition, inversion, and why the notation T_wc means "maps camera coords into world coords". A little exposure to the Lie algebra view (so(3)/se(3), exp/log maps) pays off enormously — it's how every optimiser parameterises rotations. Barfoot's State Estimation for Robotics ch. 6–7, or the first lecture of any visual-SLAM course.
Pose graph optimisation — nodes are poses, edges are relative measurements, and you minimise total inconsistency. The new concepts: what an information matrix is (inverse covariance — "how much do I trust this edge, per axis"), least squares over a graph, and loop closure as the thing that makes the graph over-constrained and worth optimising. Grisetti's tutorial "A Tutorial on Graph-Based SLAM" is the canonical short read. You already saw the effect of this in RTAB-Map; this is what it does inside.
Bundle adjustment (BA) — mentioned in passing (filled in later by SLAM/BA). Just know what it is: jointly optimising poses and 3D points against reprojection error. You don't need to implement it.
Skim the TUM RGB-D dataset format — it's just a directory layout + a poses text file, ten minutes of reading, and it makes the "existing tooling for free" claim concrete.
Section 2 (TSDF fusion) — the biggest genuinely-new chunk:

Signed distance functions — a scalar field where each voxel stores distance to the nearest surface, negative inside. The surface is the zero crossing. Then truncated SDF: only store it within a band around the surface (that's the "~2% of voxels" claim).
TSDF integration — how one depth image updates the field: for each voxel, project into the camera, compare depth, and do a running weighted average. This averaging is the whole point of the section — understand why averaging distances denoises where averaging points doesn't. The KinectFusion paper (Newcombe 2011) is the origin; you only need §3.
Voxel hashing — spatial hash map from block coordinates to 8³ voxel blocks, allocating blocks only near surfaces. Nießner 2013 ("Voxel Hashing") is the reference, but the concept is just "sparse dict instead of dense array" — which your cloud_mapper already does at 5 cm.
Open3D's VoxelBlockGrid — practical: run their TSDF integration tutorial on a TUM sequence with ground-truth poses. This is the single best exercise on this list: real fusion, no SLAM needed, one evening.
Surfels — only need the one-paragraph version: oriented discs with confidence, updated per-frame, deformable when loop closures fire. Know why you're not using them.
Section 3 (output) — mostly classical geometry processing:

Marching cubes — the algorithm that extracts a triangle mesh from the TSDF's zero crossing. Understand the idea (per-voxel-cell lookup table of triangle configurations); never implement it, Open3D does it in one call.
Mesh basics — vertices/faces, per-vertex colour vs UV texture atlas, and the file formats (PLY = simple/lossless intermediate, glTF/GLB = delivery). Light reading.
RANSAC — you'll meet it here for plane fitting, and it recurs everywhere in geometry (you dodged it in the Kabsch work by using the matcher's cross-check). Learn the general loop: sample minimal set → fit → count inliers → repeat. Then plane-specific: a plane is (n, d), fit from 3 points.
Manhattan-world assumption — the idea that indoor scenes have three dominant orthogonal directions, so you snap wall normals to them. Concept only.
Oriented bounding boxes (OBB) — the {c, R, e} in the JSON: centre, rotation, extents. Trivial once SE(3) clicks.
The alternatives paragraph:

3D Gaussian Splatting — for now just the elevator version: scene = millions of anisotropic blobs optimised by differentiable rendering; photorealistic but not geometric. The {μ, q, s, α, SH} fields are position, orientation quaternion, scale, opacity, and spherical harmonics coefficients for view-dependent colour (SH is the one term there worth a five-minute lookup).
Suggested order
The dependency chain is roughly: SE(3) conventions → TSDF integration → run the Open3D tutorial → pose graphs → marching cubes/meshing → RANSAC planes, with BA, surfels, and splatting as "know what they are" sidebars. Items 1, 5–6, and 8 are the load-bearing ones — if you do only three things, do those. And item 8 connects straight back to your repo: a recorded bag from just record plus your depth node's output is almost a TUM-format capture already; the missing ingredient for feeding it into a real TSDF is exactly the T_wc poses, which is why the doc puts SLAM at the centre and why your rotation-only orientation work was the honest first step toward it.



topics:
1. Geometry & transforms (foundations)
Rotation representations — rotation matrix, quaternion, axis-angle; converting between them
SO(3) and SE(3) — rigid transforms as 4×4 matrices; composition and inversion
Transform conventions — T_wc vs T_cw notation, frame direction, right-handedness, Z-up gravity alignment
Lie algebra basics — so(3)/se(3), exp/log maps, why optimisers parameterise rotations this way
Camera model — pinhole intrinsics K, projection and back-projection, depth image → 3D points
2. Pose estimation & SLAM (fills in T_wc)
Visual odometry — feature matching frame-to-frame (your ORB/Kabsch work is this)
Keyframe selection — why you keep a subset of frames, not all
Pose graph optimisation
Graph structure: nodes = SE(3) poses, edges = relative transforms
Information matrix (inverse covariance) as edge weight
Nonlinear least squares over the graph
Loop closure — detection and why it makes optimisation worthwhile
Libraries: GTSAM, g2o, Open3D's built-in
Bundle adjustment — joint pose + landmark optimisation, reprojection error (concept only)
Dataset/capture formats — TUM RGB-D and ScanNet layouts
3. TSDF fusion (the core new material)
Signed distance functions — distance field, sign convention, surface as zero crossing
Truncation — storing only the band near surfaces; truncation distance
TSDF integration — per-voxel projective update, running weighted average, why averaging distances denoises where averaging points doesn't
Sparse voxel hashing — block structure (8³ voxels), hash map keyed on block coords, memory math
Voxel size selection — sensor noise vs resolution trade-off
Raycasting a TSDF — rendering/depth prediction from the field (used by KinectFusion-style tracking)
Tooling — open3d.t.geometry.VoxelBlockGrid, the Open3D reconstruction-system tutorial
Surfels (contrast topic) — position/normal/radius/confidence, ElasticFusion, deformation on loop closure
4. Mesh extraction & output
Marching cubes — zero-crossing extraction, the triangle lookup-table idea
Triangle mesh basics — vertices, faces, normals
Texturing — per-vertex colour vs UV atlas, best-view face selection
File formats — PLY (intermediate), glTF/GLB (delivery)
5. Structural scene understanding
RANSAC — the generic loop (minimal sample → fit → inlier count), then plane fitting specifically
Plane representation — normal n + offset d, boundary polygons
Manhattan-world assumption — snapping walls to three orthogonal axes
Scene graph layer — planes/openings/objects schema, why a mesh alone isn't a room model
Oriented bounding boxes — centre, rotation, extents
Object detection/segmentation on 3D data (for the "class": "chair" part — deferred topic)
6. Alternatives & context
3D Gaussian Splatting — gaussian parameters (μ, q, s, α, SH), differentiable rendering, spherical harmonics for view-dependent colour, why it's non-geometric
When to choose what — TSDF (measurable/editable) vs splatting (photorealistic) vs surfels (deformable)
Sections 1 and 3 are the load-bearing ones; 2 is where your existing piros2 work plugs in; 4–5 are downstream and mostly library calls once the earlier parts click.
