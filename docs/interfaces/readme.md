


### Interface Contract

```
                 DRONE RANGER CONTRACT

Topics
────────────────────────
/drone/rgb
/drone/depth
/scan
/fmu/out/...
/tf
/tf_static


Frames
────────────────────────
base_link
oak_rgb_camera_frame
oak_rgb_camera_optical_frame
laser_frame


Data conventions
────────────────────────
timestamps
camera intrinsics
units
coordinate conventions
```

### Real Physical World

```
OAK + LiDAR + PX4
        ↓
    CONTRACT
```

### Simulation World
```
Isaac + SITL
        ↓
    CONTRACT
```

### Reconstruction
```
    CONTRACT
        ↓
reconstruction
```

interfaces/
├── topics.md
├── frames.md
└── data_conventions.md

