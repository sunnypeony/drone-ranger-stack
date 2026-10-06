# Health Check flow

```
./collect.sh backyard_001
          │
          ▼
Check OAK-D USB
          │
          ✓
Check LiDAR serial
          │
          ✓
Check ROS topics
          │
          ✓
Check topic frequency
          │
          ✓
Check TF tree
          │
          ✓
Check disk space
          │
          ✓
Create metadata
          │
          ✓
START RECORDING
```

*Example*:
    Drone Ranger Data Collection
    ─────────────────────────────

    [✓] OAK-D detected
    [✓] LiDAR detected
    [✓] /oak/rgb/image_raw     10.0 Hz
    [✓] /oak/depth/image_raw   10.0 Hz
    [✓] /scan                  10.02 Hz
    [✓] TF base_link → camera  OK
    [✓] TF base_link → lidar   OK
    [✓] Disk available         184 GB

    Dataset:
    backyard_20261003_001

    Recording...