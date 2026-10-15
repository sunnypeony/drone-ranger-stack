
## Overall workflow overview
```
Phone photos
  │
  ▼
Phase 1: COLMAP sparse reconstruction (SfM)
  │ Output: camera poses + sparse point cloud
  ▼
Phase 2: COLMAP dense reconstruction (MVS)
  │ Output: dense point cloud fused.ply
  ▼
Phase 3: Open3D Poisson reconstruction → Mesh
  │ Output: scene_mesh.obj
  ▼
Phase 4: Blender cleanup and simplification
  │ Output: scene_clean.obj
  ▼
Phase 5: Import into Isaac Sim + collision properties
  │ Output: a flyable obstacle-aware simulation scene
  ▼
Phase 6: Verify that the depth camera can detect collisions
```

![alt text](<Architecture of COLMAP IsaacSim.svg>)

## Preparation: directory structure and tool installation
Create the workspace first; all subsequent commands will be based on this structure:
```bash
mkdir -p ~/backyard_reconstruction/{images,sparse,dense,mesh}
export PROJECT=~/backyard_reconstruction
```
Install the required tools:
```bash
# Open3D (point cloud processing and Poisson reconstruction)
pip3 install open3d
# MeshLab (optional, GUI for viewing Mesh)
sudo apt install meshlab -y
# Verify COLMAP is installed
colmap --version
# Expected output: COLMAP 4.x.x
```

## Phase 1: Photo collection (most important)
The quality of the COLMAP reconstruction depends on the photo quality by around 90%, so this step cannot be ignored.

### Shooting strategy (using a backyard as an example)
```
Recommended shooting path:
  Outer loop: stay about 3~5 m from the scene boundary and walk a full circle, taking one shot every 0.5~1 m
  Middle loop: stay 1~2 m from the scene and focus on details such as buildings, fences, and the ground
  Additional shots: take extra photos from different angles for areas with few features, such as trees and corners
Target total number of images: about 150~250 for an outdoor backyard
```

**Key camera settings (iPhone example):**
```
- Lock exposure (AE/AF lock): press and hold the screen until AE/AF LOCK appears
- Avoid panoramas or ultra-wide lenses
- Do not enable HDR (it changes image appearance and affects feature matching)
- Move slowly to avoid motion blur
- Uniform overcast light works best; avoid strong shadows
```

**Simple criterion for adequacy:** if the same area in adjacent images covers more than 70% of the scene (large overlap), it is usually enough.

Copy the images to the Linux machine:
```bash
# Method 1: copy via USB connection
cp /media/your_phone/DCIM/*.jpg $PROJECT/images/
# Method 2: transfer from Mac with rsync
rsync -avz ~/Pictures/backyard/ drone@your_vm_ip:~/backyard_reconstruction/images/
```

## Phase 2: COLMAP sparse reconstruction (SfM)
The COLMAP SfM pipeline reconstructs a sparse 3D structure from a collection of overlapping images taken from different viewpoints. The input is a set of overlapping images of the same scene, and the output is the 3D reconstruction plus the intrinsic/extrinsic parameters of all images.

### Step 2.1: Feature extraction
```bash
colmap feature_extractor \
    --database_path $PROJECT/database.db \
    --image_path $PROJECT/images \
    --ImageReader.single_camera 1 \
    --SiftExtraction.use_gpu 1 \
    --SiftExtraction.max_num_features 8192
```
Parameter explanation:
- `--ImageReader.single_camera 1`: tells COLMAP that all photos come from the same camera (your phone), preventing it from estimating different intrinsics for different images and significantly improving reconstruction stability.
- `--SiftExtraction.max_num_features 8192`: extract at most 8192 feature points per image; this is enough for outdoor scenes.

Typical runtime: about 2~5 minutes, depending on the number of images.

**Core change on Mac:**
Use `--use_gpu 0` instead of `--use_gpu 1` for all commands.

### Debug tip:
```bash
colmap feature_extractor --help 2>&1 | grep -i gpu
```

**Case A: the parameter name changed to `gpu_index`**
```bash
colmap feature_extractor \
    --database_path $PROJECT/database.db \
    --image_path $PROJECT/images \
    --ImageReader.single_camera 1 \
    --SiftExtraction.gpu_index -1 \
    --SiftExtraction.max_num_features 8192
```
`-1` means do not use GPU and force CPU mode.

**Case B: the new version does not have a GPU option at all (Mac homebrew COLMAP normally uses CPU by default)**
```bash
colmap feature_extractor \
    --database_path $PROJECT/database.db \
    --image_path $PROJECT/images \
    --ImageReader.single_camera 1 \
    --SiftExtraction.max_num_features 8192
```
Simply remove the GPU-related parameters. The Homebrew build of COLMAP on Mac is usually compiled without CUDA support, so without any GPU parameter it defaults to CPU mode.

Completion signal:
```
I1234  feature_extractor.cc:xxx] Processed file [150/150]
```

### Step 2.2: Feature matching
For an outdoor backyard with fewer than 300 images, use `exhaustive_matcher`, which matches every pair of images:
```bash
colmap exhaustive_matcher \
    --database_path $PROJECT/database.db \
    --SiftMatching.use_gpu 1
```
> ⚠️ If you have more than 500 images, switch to `vocab_tree_matcher` for better speed.

Typical runtime: around 5~15 minutes.

### Step 2.3: SfM reconstruction (Mapper)
```bash
colmap mapper \
    --database_path $PROJECT/database.db \
    --image_path $PROJECT/images \
    --output_path $PROJECT/sparse
```
Typical runtime: about 5~20 minutes.

**Validate the reconstruction quality:**
```bash
colmap model_analyzer \
    --path $PROJECT/sparse/0
```
Check the following metrics:
```
Cameras:           1
Images:            150      ← total images
Registered images: 145      ← successful registrations, should be >90%
Points:            85000    ← sparse point count, >50000 is good
Mean reprojection error: 0.8px  ← <1.5px is good
```
If the number of registered images is low (<80%), the usual cause is insufficient overlap between photos; you need to add more coverage.

**Visualize the sparse point cloud:**
```bash
colmap gui
# File → Import → select the $PROJECT/sparse/0 folder
# You can see the camera positions (small triangles) and sparse point cloud
```

## Phase 3: COLMAP dense reconstruction (MVS)
MVS builds depth maps and normal maps for all registered images based on the sparse reconstruction and camera poses. It then fuses the depth maps and normals into a dense point cloud with normal information, and finally estimates a dense surface using Poisson reconstruction.

### Step 3.1: Undistort the images
```bash
colmap image_undistorter \
    --image_path $PROJECT/images \
    --input_path $PROJECT/sparse/0 \
    --output_path $PROJECT/dense \
    --output_type COLMAP
```

### Step 3.2: Estimate depth maps via MVS
```bash
colmap patch_match_stereo \
    --workspace_path $PROJECT/dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.geom_consistency true
```
> ⚠️ This step requires a CUDA GPU. CPU mode is extremely slow (several hours). If run on a machine with a GPU, it usually takes about 10~30 minutes.

**Important note:** `patch_match_stereo` for dense reconstruction must be run on RunPod or another machine with CUDA support. On a Mac, COLMAP MVS is only supported with CUDA, and Apple Silicon GPUs are not supported.

### Step 3.3: Fuse depth maps into a dense point cloud
```bash
colmap stereo_fusion \
    --workspace_path $PROJECT/dense \
    --workspace_format COLMAP \
    --input_type geometric \
    --output_path $PROJECT/dense/fused.ply
```
After it finishes, check:
```bash
ls -lh $PROJECT/dense/fused.ply
# Typical size: 50MB ~ 500MB (depends on scene complexity)
```
Preview it in MeshLab:
```bash
meshlab $PROJECT/dense/fused.ply
```
You should see a colored dense point cloud, where you can recognize the grass, trees, and building outlines of the backyard.

## Phase 4: Open3D Poisson reconstruction → Mesh
A dense point cloud is not yet a mesh; it still needs surface reconstruction. Use Open3D’s Poisson reconstruction:
```python
# poisson_mesh.py
import open3d as o3d
import numpy as np

print("Loading point cloud...")
pcd = o3d.io.read_point_cloud(
    "/root/backyard_reconstruction/dense/fused.ply")
print(f"  Points: {len(pcd.points):,}")

# 1. Statistical filtering to remove isolated noise
print("Denoising...")
pcd, ind = pcd.remove_statistical_outlier(
    nb_neighbors=20,
    std_ratio=2.0)
print(f"  After denoising: {len(pcd.points):,} points")

# 2. Estimate normals (required for Poisson reconstruction)
print("Estimating normals...")
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))

# 3. Orient normals consistently toward the camera
pcd.orient_normals_towards_camera_location(
    camera_location=np.array([0, 0, 5]))

# 4. Poisson surface reconstruction
print("Running Poisson reconstruction (depth=10)...")
mesh, densities = o3d.geometry.TriangleMesh\
    .create_from_point_cloud_poisson(pcd, depth=10)

# 5. Remove low-density vertices (eliminate poor edge reconstruction)
print("Removing low-density vertices...")
density_threshold = np.quantile(np.asarray(densities), 0.05)
vertices_to_remove = np.asarray(densities) < density_threshold
mesh.remove_vertices_by_mask(vertices_to_remove)
print(f"  Triangles: {len(mesh.triangles):,}")

# 6. Save mesh
output_path = "/root/backyard_reconstruction/mesh/scene_mesh.obj"
o3d.io.write_triangle_mesh(output_path, mesh)
print(f"Saved: {output_path}")
```
```bash
python3 poisson_mesh.py
```
Typical runtime: about 2~5 minutes.

## Phase 5: Blender cleanup and simplification
The dense-reconstruction mesh usually contains millions to tens of millions of faces. If imported directly into Isaac Sim, it may become very slow. We simplify it to below **500,000 faces**.

Install Blender if it is not already installed:
```bash
sudo snap install blender --classic
```
Use the Blender Python API to process the mesh in batch without opening the GUI:
```python
# simplify_mesh.py  (run with blender --background)
import bpy, sys

input_path  = "/root/backyard_reconstruction/mesh/scene_mesh.obj"
output_path = "/root/backyard_reconstruction/mesh/scene_clean.obj"
target_face_ratio = 0.05  # Keep 5% of faces (usually enough)

# Clear the default scene
bpy.ops.wm.read_factory_settings(use_empty=True)

# Import OBJ
bpy.ops.wm.obj_import(filepath=input_path)
obj = bpy.context.selected_objects[0]
print(f"Original faces: {len(obj.data.polygons):,}")

# Add a Decimate modifier (mesh simplification)
mod = obj.modifiers.new(name="Decimate", type="DECIMATE")
mod.ratio = target_face_ratio

# Apply the modifier
bpy.ops.object.modifier_apply(modifier="Decimate")
print(f"After decimation: {len(obj.data.polygons):,}")

# Export the simplified OBJ
bpy.ops.wm.obj_export(
    filepath=output_path,
    export_selected_objects=True)
print(f"Saved: {output_path}")
```
```bash
blender --background --python simplify_mesh.py
```

**Check whether the face count is reasonable:**
```bash
# Count the number of faces in the OBJ file (lines starting with f)
grep -c "^f " ~/backyard_reconstruction/mesh/scene_clean.obj
# Target: < 500000
```

## Phase 6: Import into Isaac Sim and add collision properties
This step is done in the Isaac Sim environment (RunPod GPU).

### Step 6.1: Transfer the mesh file to RunPod
```bash
# Run on the local machine
scp -P your_port ~/backyard_reconstruction/mesh/scene_clean.obj \
    root@your_runpod_ip:/root/backyard_reconstruction/mesh/scene_clean.obj
```

### Step 6.2: Import the mesh, align it and add collision
Open [`simulation/isaac_sim/usr_scripts/import_scene_mesh.py`](../../simulation/isaac_sim/usr_scripts/import_scene_mesh.py) in the Isaac Sim Script Editor (`Window -> Script Editor`).
Set `MESH_PATH` at the top and run it. The script:

1. converts `scene_clean.obj` to `scene_clean.usd` with `omni.kit.asset_converter` (USD cannot reference an OBJ directly);
2. references it at `/World/BackyardScene/mesh` and applies translate / rotate / scale to `/World/BackyardScene`;
3. applies `UsdPhysics.CollisionAPI` + `MeshCollisionAPI` to **every `Mesh` prim** under it.

Things that are easy to get wrong:

- Collision APIs only work on the `Mesh` prims. Applying them to the referenced root `Xform` has no effect.
- A static environment needs **no** `RigidBodyAPI`: a collider without a rigid body is static.
- Approximation options for the mesh collider:
  - `"none"` (exact triangle mesh): the default in the script and the standard choice for static scenes.
  - `"convexDecomposition"`: cheaper, but fills in concave regions such as doorways and gaps between trees.
  - `"sdf"`: meant for dynamic bodies with complex shapes; not needed here.

When it looks right, save the stage (`File -> Save As`, e.g. `backyard.usd`).
That file can be loaded straight into a Pegasus simulation with `pg.load_environment(".../backyard.usd")`; see [pegasus.md](../simulation/isaac-sim/pegasus.md#9-use-your-own-scene-reconstructed-backyard).

### Step 6.3: Position the scene at the correct location and scale
COLMAP's world frame is arbitrary: it is not Y-up or Z-up by design, and it has **no metric scale**.
Adjust `ROTATE_XYZ_DEG`, `SCALE` and `TRANSLATE` at the top of the script and re-run it:

- **Rotation**: rotate until the ground is horizontal in Isaac Sim's Z-up world. `-90°` about X is a common starting point, not a rule.
  `colmap model_aligner` (e.g. `--alignment_type plane`) can level the sparse model before meshing.
- **Scale**: measure a known distance in the scene, such as a gate width of ≈ 1 m, and set `SCALE = real / measured`.
  Without this, depth values in simulation do not match reality.
- **Translation**: bring the area you want to fly in near the origin and put the ground at `z = 0`, so the drone spawns above it.

## Phase 7: Verify that the depth camera can detect collisions
After spawning the Pegasus drone into the scene, run the camera script from Phase 3 and validate it.

**Terminal — subscribe to the depth camera and inspect the data:**
```bash
# Verify that the depth topic is publishing data
ros2 topic hz /drone/depth
# Expected: ~60Hz

# Inspect the depth image value range
python3 - << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.sub = self.create_subscription(
            Image, '/drone/depth',
            self.callback, 10)

    def callback(self, msg):
        # Format is 32FC1: each value is a float32
        data = np.frombuffer(msg.data, dtype=np.float32)
        valid = data[np.isfinite(data) & (data > 0)]
        if len(valid) > 0:
            print(f"Depth range: {valid.min():.2f}m ~ "
                  f"{valid.max():.2f}m | "
                  f"Valid pixels: {len(valid)}/{len(data)}")

rclpy.init()
node = DepthChecker()
rclpy.spin(node)
EOF
```

**After takeoff, fly close to the scene boundary:**
```bash
# In pxh>
commander arm
commander takeoff
# Manually fly toward a wall or tree
```
When the drone approaches the imported mesh, the minimum depth value reported by the depth camera should decrease, indicating that nearby obstacles are being detected. This confirms that the collision mesh is working correctly.

## Complete data flow summary
```
Phone photos (150~250)
    │
    ▼ COLMAP feature_extractor + exhaustive_matcher
Sparse point cloud + camera poses (sparse/0/)
    │
    ▼ COLMAP patch_match_stereo + stereo_fusion
Dense point cloud (dense/fused.ply, 50~500MB)
    │
    ▼ Open3D denoising + normal estimation + Poisson reconstruction
Original mesh (mesh/scene_mesh.obj, millions of faces)
    │
    ▼ Blender Decimate (keep 5%)
Simplified mesh (mesh/scene_clean.obj, <500k faces)
    │
    ▼ Isaac Sim import + UsdPhysics.CollisionAPI (static triangle mesh)
Collision-enabled simulation scene
    │
    ▼ Pegasus drone + Action Graph depth camera
/drone/depth topic → depth data reflects real obstacle geometry
```

## Quick troubleshooting

**Q: COLMAP registers very few images (<60%)**
```
Cause: insufficient photo overlap or too much lighting variation (for example, shoot morning and afternoon separately)
Solution: add more overlap; complete the shoot in a single time window
```

**Q: `patch_match_stereo` reports a CUDA error**
```
Cause: COLMAP was not built with CUDA support
Solution: rebuild with:
    cmake .. -DFETCH_CUDA=ON -DCUDA_ENABLED=ON
    make -j$(nproc)
```

**Q: The Poisson reconstruction mesh has many holes**
```
Cause: the point cloud has insufficient coverage in some regions (for example, ground or sky)
Solution: increase the Poisson depth parameter (from 10 to 11), or reshoot and rerun COLMAP for the missing areas
```

**Q: The scene orientation or scale is wrong after import into Isaac Sim**
```
Solution: adjust ROTATE_XYZ_DEG / SCALE / TRANSLATE in import_scene_mesh.py and re-run it
          inspect the coordinate range of the sparse cloud with COLMAP model_analyzer
          estimate the scale factor using real-world distances (e.g. gate width ≈ 1 m)
```
