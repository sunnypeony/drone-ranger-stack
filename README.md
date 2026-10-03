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
