# Drone Ranger

UAV perception, 3D reconstruction, and simulation stack for RGB-D and LiDAR-based environmental understanding.

Drone Ranger is a research and engineering project focused on bridging real-world drone sensing with simulation-based validation. It combines ROS 2 data acquisition, camera and LiDAR calibration, offline reconstruction, and Isaac Sim/PEGASUS-based testing for obstacle perception and navigation.

## Project overview

The project follows a practical pipeline:

1. Collect synchronized sensor data from real hardware.
2. Build a calibrated spatial representation from images and point clouds.
3. Reconstruct dense geometry for downstream simulation use.
4. Import the reconstructed environment into Isaac Sim.
5. Validate that the drone can perceive and avoid obstacles in simulation.

This repository covers the full flow from sensor capture to reconstructed scene generation and simulation integration.

## Key capabilities

- RGB-D acquisition with OAK-D Lite and ROS 2 publishing
- LiDAR integration for 2D range sensing
- Calibration and TF setup for sensor frames
- Offline structure-from-motion (SfM) and multi-view stereo (MVS) with COLMAP
- Dense point cloud generation and mesh reconstruction with Open3D
- Blender-based mesh cleanup and simplification
- Isaac Sim scene import and collision setup
- Experiment tracking and reusable configuration files

## System architecture

The system combines both real and simulated environments under a shared ROS 2 interface:

<p align="center">
  <img src="docs/architecture/system_architecture.svg"
       alt="Drone Ranger System Architecture"
       width="100%">
</p>



The Drone Ranger platform provides a unified ROS 2 interface across real-world and simulated UAV environments.
The real platform integrates an OAK-D Lite RGB-D camera, LiDAR, Pixhawk/PX4, and NVIDIA Jetson companion computer.

- Real platform: PX4, Jetson, OAK-D Lite, LiDAR, and ROS 2 topics
- Simulation platform: Isaac Sim, Pegasus, and PX4 SITL
- Offline processing: COLMAP, Open3D, Blender, mesh simplification
- Target result: obstacle-aware environment representation for UAV perception and navigation

## Hardware and software stack

### Hardware

- Holybro X500 V2 frame
- Pixhawk 6C flight controller
- NVIDIA Jetson companion computer
- OAK-D Lite RGB-D camera
- YDLIDAR T-mini Plus LiDAR

### Software

- Ubuntu 22.04 LTS
- ROS 2 Humble
- PX4
- uXRCE-DDS
- DepthAI / OAK-D drivers
- COLMAP (SfM + MVS)
- Open3D (point cloud processing, Poisson reconstruction, TSDF-related workflows)
- Isaac Sim / Omniverse
- Isaac Lab
- Foxglove for topic visualization
- Blender for mesh cleanup and simplification

## Repository structure

```text
drone-ranger-stack/
├── README.md
├── config/
│   ├── calibration/
│   ├── sensors/
│   ├── tf/
│   └── topics.yaml
├── docs/
│   ├── architecture/
│   ├── auto_planning/
│   ├── calibration/
│   ├── hardware/
│   ├── interfaces/
│   ├── reconstruction/
│   ├── simulation/
│   └── troubleshooting/
├── experiments/
│   └── 2026-10-rgbd-lidar-sync/
│   └── 2026-10-tsdf-backyard/
├── onboard/
│   ├── acquisition/
│   ├── launch/
│   ├── sensors/
│   └── synchronization/
├── reconstruction/
│   └── preprocessing/
├── simulation/
│   └── isaac_sim/
└── .gitignore
```

### Directory highlights

- `config/`: sensor, calibration, TF, and topic configuration files
- `onboard/`: ROS 2 launch, acquisition scripts, and sensor bridge code
- `docs/`: architecture docs, calibration guides, reconstruction notes, and hardware tutorials
- `experiments/`: experiment configs and notes for active recordings and reconstruction runs
- `reconstruction/`: offline reconstruction workflow and preprocessing assets
- `simulation/`: Isaac Sim setup and related assets

## Typical workflow

The repository is organized around a practical end-to-end perception workflow:

```text
Phone / drone capture
    ↓
ROS 2 sensor streaming
    ↓
Synchronization and calibration
    ↓
COLMAP sparse reconstruction (SfM)
    ↓
COLMAP dense reconstruction (MVS)
    ↓
Open3D mesh generation
    ↓
Blender cleanup / simplification
    ↓
Isaac Sim import + collision setup
    ↓
Depth-camera collision validation in simulation
```

## Quick start

### 1. Prepare the workspace

```bash
mkdir -p ~/backyard_reconstruction/{images,sparse,dense,mesh}
export PROJECT=~/backyard_reconstruction
```

### 2. Install dependencies

```bash
pip3 install open3d
colmap --version
```

Additional tooling may be installed depending on the stage you are working on:

```bash
sudo apt install meshlab -y
```

### 3. Capture data

The project currently includes acquisition scripts and setup documents for collecting RGB-D and LiDAR data from the UAV platform. See the folders under `onboard/` and `experiments/`.

### 4. Build a reconstruction

Use the COLMAP pipeline for sparse and dense reconstruction, then convert the result into a mesh for downstream use. See the reconstruction documentation under `docs/reconstruction/`.

### 5. Import the mesh into Isaac Sim

After mesh cleanup and simplification, the scene can be imported into Isaac Sim and assigned collision properties for obstacle-aware simulation.

## Data acquisition and calibration

The project includes support for:

- RGB-D camera publishing via the OAK-D bridge
- lidar and sensor topic configuration
- calibration and TF definition
- synchronization and time-aligned data collection

The active bridge implementation is located in:

- `onboard/sensors/oak_d/oak_rgbd_ros_v2.py`

This script publishes aligned RGB and depth streams and emits the corresponding camera_info metadata needed by downstream reconstruction and perception workflows.

## Reconstruction pipeline

The reconstruction workflow is centered around COLMAP and Open3D:

- `colmap feature_extractor`
- `colmap exhaustive_matcher`
- `colmap mapper`
- `colmap patch_match_stereo`
- `colmap stereo_fusion`
- Open3D Poisson reconstruction
- Blender simplification

The relevant notes live in:

- `docs/reconstruction/colmap.md`
- `docs/reconstruction/colmap_pipeline.md`
- `docs/reconstruction/colmap_pipeline.ipynb`

## Simulation and robotics stack

The simulation workflow targets Isaac Sim, Pegasus, and PX4-based validation:

- scene import from reconstructed mesh
- collision setup and physics configuration
- camera validation in simulation
- UAV depth-camera-perception tests near obstacles

This is intended to bridge the gap between offline world reconstruction and real-time drone autonomy evaluation.

## Documentation

Useful project documentation is organized into the following areas:

- `docs/architecture/`: architecture and system-level descriptions
- `docs/calibration/`: calibration and sensor extrinsics documentation
- `docs/hardware/`: platform setup and hardware notes
- `docs/interfaces/`: ROS topic, frame, and interface definitions
- `docs/reconstruction/`: reconstruction and 3D modeling guidance
- `docs/simulation/`: Isaac Sim and simulation-specific setup guidance
- `docs/troubleshooting/`: debugging notes for hardware and ROS issues

## Experiments

The repository contains experiment folders under `experiments/` for project-specific datasets and run configurations. These can be used as references for reconstruction and synchronization experiments.

## Development status

This repository is a research-oriented UAV perception and reconstruction stack. It is intended to be extended and adapted as the platform evolves. Some modules are still in active development and are organized as a set of practical engineering scripts and documentation rather than a single package.

## License

This project currently does not declare a specific license in the repository root. If you intend to reuse or redistribute it, confirm the project’s licensing requirements before publication or deployment.

## Contributing

This repo is best used as a working research environment for:

- sensor integration
- 3D reconstruction experiments
- calibration validation
- Isaac Sim obstacle scene generation
- perception validation for drone navigation

If you are extending the project, keep the following in mind:

- prefer clear documentation under `docs/`
- keep sensor configurations under `config/`
- keep acquisition scripts under `onboard/`
- keep reconstruction-related tasks under `reconstruction/`
- store experiment-specific notes under `experiments/`
