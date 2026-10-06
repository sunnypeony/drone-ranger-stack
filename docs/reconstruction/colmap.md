

# Installation

	Sparse（SIFT / SfM） → CPU可以 ✔️
	Dense（PatchMatch） → 必须CUDA ❌****


#### Step 1 - Check version


```bash
# check OS version
uname - #x86_64
lsb_release -a #ubuntu22.04

# check nvidia version
nvidia-smi
nvcc --version
colmap -h | grep CUDA
```
#### Step 2 - Install Dependencies
```
apt install -y \
  git cmake ninja-build
  libopenimageio-dev \
  openimageio-tools \
  libopenexr-dev \
  libsuitesparse-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libglew-dev \
  libgtest-dev \
  libboost-all-dev \
  libeigen3-dev \
  libflann-dev \
  libfreeimage-dev \
  libmetis-dev \
  libsqlite3-dev \
  qtbase5-dev \
  libqt5svg5-dev \
  libqt5opengl5-dev \
  libcgal-dev \
  libceres-dev
  
```

##### for Ubuntu24

```bash
apt update
add-apt-repository universe -y
apt update
apt install -y software-properties-common
apt install -y apt-transport-https ca-certificates gnupg
```
if openimageio incomplete,
install openimageio-tools, and veriry `which iconvert`

#### Step 3 - build

```bash
git clone https://github.com/colmap/colmap.git  
cd colmap  
mkdir build && cd build
```


```bash
cmake .. -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCUDA_ENABLED=ON \
  -DCMAKE_CUDA_ARCHITECTURES=89 # 89 = RTX 4090(Ada architecture), if without it, takes more time to build, or abnormal performance, or not support GPU
```


P.S.:
- How to debug:
==add-apt-repository== universe -y
apt install -y software-properties-common


If error occurs during build, decrease gcc:
```bash
apt install gcc-11 g++-11
export CC=/usr/bin/gcc-11
export CXX=/usr/bin/g++-11
```



#### Step 4 - Build & Install

`ninja -j$(nproc)`

```bash
ninja
ninja install
```


#### C Make issue


sudo apt install -y \
  cmake=3.30.2-0kitware1ubuntu22.04.1 \
  cmake-data=3.30.2-0kitware1ubuntu22.04.1

## 如果你想从源码编译（可选，适合最新版本）

git clone https://ceres-solver.googlesource.com/ceres-solver  
cd ceres-solver  
mkdir build && cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF  
make -j$(nproc)  
make install

然后在 COLMAP cmake 时指定：

cmake .. -DCeres_DIR=/usr/local/lib/cmake/Ceres

> 但对于 Ubuntu 容器，apt 包已经足够，而且更简单



## What is inside sparse/0?
```
sparse/0/
├── cameras.bin     ← camera intrinsics (focal length, distortion, etc.)
├── images.bin      ← camera extrinsics for each image (position and orientation)
└── points3D.bin    ← sparse point cloud (3D coordinates + color)
```
These three `.bin` files are COLMAP’s proprietary binary format and cannot be dragged directly into other software. However, COLMAP provides ==conversion commands== to export them into a more portable format.

## Export to a usable format

### Method 1: Export to PLY (most general format; supported by MeshLab and CloudCompare)
```bash
# Export the sparse point cloud as PLY
colmap model_converter \
    --input_path $PROJECT/sparse/0 \
    --output_path $PROJECT/sparse/0 \
    --output_type PLY
```
After running, a file called `$PROJECT/sparse/0/points3D.ply` will be generated. You can drag it directly into MeshLab or CloudCompare to inspect it.

***Note***: ==PLY contains only the point cloud and does not include camera poses.==

### Method 2: Export to TXT (human-readable)
```bash
colmap model_converter \
    --input_path $PROJECT/sparse/0 \
    --output_path $PROJECT/sparse/0_txt \
    --output_type TXT
```
This generates three text files:
```
cameras.txt   ← camera intrinsics, viewable in a text editor
images.txt    ← camera poses for each image (quaternion + translation)
points3D.txt  ← each 3D point with XYZ + RGB + reprojection error
```
This is the most useful format for debugging. You can inspect the camera poses directly with:
```bash
cat images.txt | head -20
```

### Method 3: Read it in Python with pycolmap (most flexible)
```bash
pip3 install pycolmap
```
```python
import pycolmap

# Load the sparse reconstruction
recon = pycolmap.Reconstruction("/Users/your_username/backyard/sparse/0")

# Print basic statistics
print(f"Number of cameras: {len(recon.cameras)}")
print(f"Number of images: {len(recon.images)}")
print(f"Number of 3D points: {len(recon.points3D)}")

# Print the pose of the first few images
for img_id, img in list(recon.images.items())[:3]:
    print(f"{img.name}: position = {img.cam_from_world.translation}")
```

## What each software can visualize

| Software | View point cloud | View camera poses | Usage |
|---|---|---|---|
| COLMAP GUI | ✅ | ✅ (small triangles) | `colmap gui` → File → Import |
| MeshLab | ✅ (import PLY) | ❌ | Drag in the PLY file |
| CloudCompare | ✅ (import PLY) | ❌ | Drag in the PLY file |
| pycolmap | ✅ | ✅ | Read in Python |

## Recommendation

The fastest validation step at this stage is to export the PLY and open it in ***MeshLab*** to inspect the point cloud quality. If the point cloud clearly reveals the rough outline of the backyard, the sparse reconstruction is likely successful and you can proceed to the dense reconstruction stage. If the cloud is too sparse or the shape looks wrong, *it is time to check whether the image overlap is sufficient.*

This is normal and not a bug. `patch_match_stereo` runs two passes for each image; the "restart from scratch" you see is actually the second pass.

## Why does it run twice?

- *First pass* (**photometric** pass):
  For each image, estimate depth from neighboring images based on **color consistency**
  → obtain an *initial* depth map

- *Second pass* (**geometric** consistency pass):
  Triggered by `--PatchMatchStereo.geom_consistency true`
  Cross-check the depth maps from the first pass and filter out geometrically inconsistent points
  → obtain a *cleaner* depth map

So for 200 images, COLMAP is effectively processing 400 depth estimates, which makes it look like it is starting over.

## How to monitor progress and tell if it is normal

Check the generated depth maps:
```bash
# Count how many images have already been processed
ls $PROJECT/dense/stereo/depth_maps/ | wc -l

# See which files were most recently created
ls -lt $PROJECT/dense/stereo/depth_maps/ | head -5

# See how many total images need processing (should be image count × 2)
ls $PROJECT/dense/images/ | wc -l
```

Monitor progress in real time in another terminal:
```bash
watch -n 10 "ls $PROJECT/dense/stereo/depth_maps/ | wc -l"
# Refresh every 10 seconds; the number increasing means the process is healthy
```

Expected file counts:
```
First pass (photometric pass)  → generates .photometric.bin files
Second pass (geometric pass)  → generates .geometric.bin files
N = total number of input images
```

## If you do not want to run the second pass (to save time)

Remove the `geom_consistency` option and run only one pass:
```bash
colmap patch_match_stereo \
    --workspace_path $PROJECT/dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.geom_consistency false
```
The quality will be slightly worse, but for later collision-mesh generation the difference is usually acceptable. This will also be roughly twice as fast.

## Rough time estimates

| Number of images | Single pass time (RTX 4090) | Total time for two passes |
|---|---|---|
| 100 | ~5 minutes | ~10 minutes |
| 200 | ~10 minutes | ~20 minutes |
| 300 | ~15 minutes | ~30 minutes |

A successful run is indicated when the command returns to the shell prompt and the number of files under `depth_maps/` is equal to roughly twice the number of input images.

You can stop the process with `Ctrl+C` and then run `stereo_fusion` using the first-pass results.
The only change is that the `--input_type` argument should be switched from `geometric` to `photometric`:
```bash
# Ctrl+C to stop patch_match_stereo
# Fuse using only the first-pass depth maps
colmap stereo_fusion \
    --workspace_path $PROJECT/dense \
    --workspace_format COLMAP \
    --input_type photometric \
    --output_path $PROJECT/dense/fused.ply
```
First confirm that the first-pass depth map files exist:
```bash
ls $PROJECT/dense/stereo/depth_maps/*.photometric.bin | wc -l
# The number should be close to the total image count; the closer the better
```
If this number is very small (for example, only 20 files), it means the first pass has not finished enough. The resulting fused point cloud will be sparse, but it is still sufficient to get the pipeline running and continue.


