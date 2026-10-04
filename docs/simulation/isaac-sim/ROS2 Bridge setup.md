
*Isaac Sim 5.x has built-in ROS2 Bridge(by extension), support ROS2 Humble/Foxy, the corresponding ROS should be installed firstly.*

## Install ROS2 Humble for Ubuntu 22.04

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
    sudo add-apt-repository universe
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
    # desktop-full includes rviz2、rqt、demo nodes
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

7. Enable ROS2 Bridge inside Isaac Sim

Run `source /opt/ros/humble/setup.bash` before launching Isaac Sim, rather than opening it by double-clicking the desktop icon, to make sure isaac sim inherit the ROS2 environment variables.

- Setup inside Isaac Sim
    - start Isaac Sim, Window -> Extension
    - search ROS Bridge
    - Omni.isaac.ros2_bridge, check 'Enable'
    - set 'LD_LIBRARY_PATH', let the built-in python env inside isaac to find ROS2.

- Test python script inside isaac-sim
```python
    import rclpy
    from rcly.node import Node
    rclpy.init()
    print("ROS2 Bridge OK")
```



## Install MAVROS2(for PX4)

```
    sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
    # 安装 GeographicLib 数据集（必须）
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

    ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
    # 或 SITL 模式
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

> Isaac Sim 5.x comes bundled with its own Python 3.10. If you encounter an "rclpy not found" error, it is usually because Isaac Sim has not inherited the ROS 2 environment variables from your shell. The solution is as mentioned above: run `source /opt/ros/humble/setup.bash` before launching Isaac Sim, rather than opening it by double-clicking the desktop icon.



Check all process
`ps aux | grep -E "isaac|ros2|python3" | grep -v grep | grep -v rangerlab`


