
## System Overview

                         REAL UAV
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  OAK-D Lite                                              │
│  ┌──────────────────────┐                                │
│  │ Myriad X / DepthAI   │                                │
│  │ stereo depth         │                                │
│  └──────────┬───────────┘                                │
│             │ USB 3                                      │
│             ▼                                            │
│  ┌─────────────────────────────────────┐                 │
│  │          Jetson Orin Nano           │                 │
│  │                                     │                 │
│  │  OAK ROS node / DepthAI             │                 │
│  │       │                             │                 │
│  │       ├── RGB topic                 │                 │
│  │       └── Depth topic               │                 │
│  │                                     │                 │
│  │  Micro XRCE-DDS Agent               │                 │
│  │       │                             │                 │
│  │       ▼                             │                 │
│  │  ROS 2 / DDS domain                 │                 │
│  │       │                             │                 │
│  │       ▼                             │                 │
│  │  perception / mapping / rosbag      │                 │
│  └──────────────▲──────────────────────┘                 │
│                 │ UART                                   │
│                 │ uXRCE-DDS transport                    │
│                 │                                        │
│          ┌──────┴───────────┐                            │
│          │    Pixhawk 6C    │                            │
│          │                  │                            │
│          │ PX4              │                            │
│          │ uXRCE-DDS Client │                            │
│          └──────────────────┘                            │
│                                                          │
└──────────────────────────────────────────────────────────┘

                  Pixhawk
                     │
                  MAVLink
                     │
                     ▼
              QGroundControl