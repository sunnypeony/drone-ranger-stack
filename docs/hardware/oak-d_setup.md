

After connecting camera to Jetson, ros2 topic list shows more published topics:
- Not connected:
```
    p1234@p1234-desktop:~$ ros2 topic list
    /oak/depth/camera_info
    /oak/depth/image_rect_raw
    /oak/test_depth_stats
```
```
p1234@p1234-desktop:~$ ros2 topic echo /oak/depth/camera_info --once
header:
  stamp:
	sec: 1785391194
	nanosec: 45404812
  frame_id: oak_depth_optical_frame
height: 480
width: 640
distortion_model: plumb_bob
d:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
k:
- 465.0
- 0.0
- 320.0
- 0.0
- 465.0
- 240.0
- 0.0
- 0.0
- 1.0
r:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
p:
- 465.0
- 0.0
- 320.0
- 0.0
- 0.0
- 465.0
- 240.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: false
```


p1234@p1234-desktop:~$ ros2 topic echo /oak/depth/image_rect_raw --once
header:
  stamp:
	sec: 1785391179
	nanosec: 251490001
  frame_id: oak_depth_optical_frame
height: 480
width: 640
encoding: 16UC1
is_bigendian: 0
step: 1280
data:
- 0
- 0
- 0
- 0
- 0
…
- 0
- 0
- '...'
---



