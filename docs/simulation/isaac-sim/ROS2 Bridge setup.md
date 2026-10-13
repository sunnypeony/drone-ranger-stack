# ROS 2 setup for Isaac Sim 5.1

*Isaac Sim 5.x ships a built-in ROS 2 bridge extension (`isaacsim.ros2.bridge`). It supports ROS 2 **Humble** (Ubuntu 22.04) and **Jazzy** (Ubuntu 24.04); Foxy is no longer supported.*

Two separate Python environments are involved:

| Where | Python | ROS 2 libraries |
|---|---|---|
| Your terminal (`ros2 topic ...`, ROS nodes, MAVROS, MicroXRCEAgent) | system Python 3.10 | `/opt/ros/humble` |
| Inside Isaac Sim (bridge, Script Editor, standalone scripts) | Isaac Sim's bundled **Python 3.11** | Isaac Sim's **internal** Humble libraries in `exts/isaacsim.ros2.bridge/humble/lib` |

System ROS 2 is still needed: you use it to see and process the topics Isaac Sim publishes.
But **Isaac Sim itself must not load the system Humble libraries**, because they are built for Python 3.10.

## Install ROS 2 Humble for Ubuntu 22.04

[ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

1. Set up locale
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

2. Add ROS2 apt source
```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

3. Install ROS2 Humble
```bash
sudo apt update
sudo apt upgrade -y
# desktop-full includes rviz2, rqt, demo nodes
sudo apt install -y ros-humble-desktop-full
# dev tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep ros-dev-tools
```

4. Initialize ***rosdep***
```bash
sudo rosdep init
rosdep update
```

5. Configure environment
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

6. Verify
```bash
# terminal 1
ros2 run demo_nodes_cpp talker
# terminal 2
ros2 run demo_nodes_py listener
```

## Enable the ROS 2 Bridge inside Isaac Sim

1. **Launch Isaac Sim with a clean environment.**
Use the `isaac_run` function from [pegasus.md](pegasus.md#21-configure-environment-variables-and-isaac_run).
It removes the system ROS variables, which `~/.bashrc` sets because of step 5 above,
and points Isaac Sim at its internal Humble libraries:
```bash
isaac_run                    # GUI
isaac_run my_script.py       # standalone Python script
```
Doing it by hand amounts to:
```bash
unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"
${ISAACSIM_PATH}/isaac-sim.sh
```
> ⚠️ Do **not** `source /opt/ros/humble/setup.bash` in the terminal you launch Isaac Sim from.
> Python 3.10 `rclpy` from the system install will then shadow the bridge's Python 3.11 libraries and the bridge fails to load (`rclpy` / `_rclpy_pybind11` import errors).

2. **Check the extension is enabled.**
`Window -> Extensions`, search `ROS 2 Bridge`, make sure `isaacsim.ros2.bridge` is enabled
(the old name `omni.isaac.ros2_bridge` is from Isaac Sim 4.x).
The startup log should contain a line like `ROS2 Bridge startup` without errors.

3. **Test inside Isaac Sim** (`Window -> Script Editor`):
```python
import rclpy
print("rclpy from:", rclpy.__file__)   # should point inside the Isaac Sim install, not /opt/ros/humble
print("ROS2 Bridge OK")
```

4. **Test the system ROS 2 side** in a normal terminal, where it is fine to source `/opt/ros/humble/setup.bash`:
```bash
python3 - << 'EOF'
import rclpy
from rclpy.node import Node
rclpy.init()
print("System ROS 2 OK")
rclpy.shutdown()
EOF
```

5. **Cross-check.** Press Play in a scene that publishes something (for example the camera graph in [cameras.md](cameras.md)), then in a normal terminal:
```bash
ros2 topic list
```
If the topics do not appear, make sure both sides use the same `ROS_DOMAIN_ID` (unset = 0) and the same `RMW_IMPLEMENTATION` (`rmw_fastrtps_cpp`).

## Install MAVROS (optional)

Only needed if you want to talk to PX4 over MAVLink from ROS 2.
The main path in this project uses uXRCE-DDS (`/fmu/*` topics, see [px4_SITL.md](px4_SITL.md)), not MAVROS.

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
# Install the GeographicLib datasets (required)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Real hardware over USB
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
# Or SITL
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

## Check running processes

```bash
ps aux | grep -E "isaac|ros2|python3" | grep -v grep
```
