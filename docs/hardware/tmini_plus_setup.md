# Installation

`sudo apt update`
`sudo apt install build-essential cmake` (for solving cmake error)
`sudo apt install swig` (for python SDK installation)


```
cd YDLidar-SDK
Mkdir build && cd build
Cmake ..
Make
Sudo make install
```

Just copy `/src`, generate `/build`, `/install`, `/log`, each machine itself.

### Compile and Build Lidar SDK

```bash
cd ~/lidar_T-mini-plus/YDLidar-SDK
rm -rf build
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```


## Compile and Build ROS2 driver

```bash
cd ~/lidar_T-mini-plus/yahboomcar_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Run

```bash
source ~/lidar_T-mini-plus/yahboomcar_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

The launch will start two process:
- ydlidar_ros2_driver_node:
from `~/lidar_T-mini-plus/yahboomcar_was/install/..`
- static_transform_publisher:
from Jeston `/opt/ros/humble/…`

```
                        Jetson ARM64
                            |
                ------------------------
                |                       |
    /opt/ros/humble/...         yahboomcar_ws/install/...
        ARM64 binary                 x86-64 binary
            |                            |
    static_transform publisher      ydlidar_ros2_driver_node
```

[*Reference*](https://www.yahboom.net/study/T-mini_Plus)


