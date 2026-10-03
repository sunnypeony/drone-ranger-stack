# drone-ranger-stack
UAV perception pipeline for RGB-D/LiDAR sensing, ROS 2 data acquisition, 3D reconstruction, PX4 integration, and Isaac Sim simulation.

## System Architecture

                         DRONE RANGER
                              │
             ┌────────────────┼────────────────┐
             │                │                │
          Hardware        Simulation       Datasets
             │                │                │
             ▼                ▼                ▼
     ┌──────────────┐   ┌────────────┐   ┌───────────┐
     │ OAK-D / LiDAR│   │ Isaac Sim  │   │ ROS bags  │
     │ PX4 / Jetson │   │ Sensors    │   │ Recorded  │
     └──────┬───────┘   └─────┬──────┘   └─────┬─────┘
            │                 │                 │
            ▼                 ▼                 │
      Sensor Drivers     Synthetic Data         │
            │                 │                 │
            └──────────┬──────┘                 │
                       ▼                        │
                  ROS 2 Topics ◀────────────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
       RGB-D          LiDAR       IMU/PX4
          │            │            │
          └────────────┼────────────┘
                       ▼
              TF + Calibration
                       │
                       ▼
               Data Acquisition
                       │
                       ▼
                   ROS Bag
                       │
                       ▼
                Preprocessing
                       │
                       ▼
            Odometry / Registration
                       │
                       ▼
               3D Reconstruction
                       │
                       ▼
          Point Cloud / TSDF / Mesh
                       │
                       ▼
              World Representation
                       │
                       ▼
            Isaac Sim / Navigation


# Drone Ranger

Drone Ranger is a UAV perception and simulation platform for RGB-D/LiDAR sensing, ROS 2 data acquisition, 3D reconstruction,
PX4 integration, and Isaac Sim experimentation.

## Overview

[architecture diagram]

Real UAV
   ↓
OAK-D + LiDAR + IMU
   ↓
ROS 2
   ↓
Sensor synchronization
   ↓
Data acquisition
   ↓
3D reconstruction
   ↓
World representation
   ↓
Isaac Sim / autonomous navigation

## Hardware

- Holybro X500 V2
- Pixhawk
- NVIDIA Jetson
- OAK-D Lite
- YDLIDAR T-mini Plus

## Software Stack

- Ubuntu
- ROS 2 Humble
- PX4
- uXRCE-DDS
- DepthAI
- Open3D
- Isaac Sim

## Repository Structure

...

## Quick Start

...

## Data Collection

...

## Reconstruction

...

## Simulation

...

## Roadmap

...

## License
