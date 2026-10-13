# Isaac Sim 5.1 + Pegasus Simulator + PX4 v1.14.3: end-to-end setup

This is the main guide for the Drone Ranger simulation stack.
It takes a clean Ubuntu 22.04 machine to a PX4-controlled drone flying in Isaac Sim, with PX4 state and camera images (RGB and depth) available as ROS 2 topics.

It follows the official [Pegasus Simulator installation guide](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)
and adds the ROS 2 / uXRCE-DDS parts that Drone Ranger needs.

- [Pegasus GNC](https://pegasusresearch.github.io/pegasus/) (the wider Pegasus project)
- [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/): an Isaac Sim extension for multirotors, with a PX4/ArduPilot MAVLink backend, a ROS 2 backend, and support for multiple drones

---

## 0. Version matrix

Use exactly these versions. They are the combination Pegasus is tested with.

| Component | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 LTS, x86_64 | 24.04 works with ROS 2 Jazzy, but is not covered here |
| NVIDIA driver | ≥ 550 (Pegasus tested with 550.163.01) | check with `nvidia-smi` |
| GPU | RTX-class, ≥ 8 GB VRAM (16 GB+ recommended) | see [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) |
| Isaac Sim | **5.1.0**, workstation (zip) install | bundled Python **3.11** |
| Pegasus Simulator | **v5.1.0** (git tag) | matches Isaac Sim 5.1.0 |
| PX4-Autopilot | **v1.14.3** | build target `px4_sitl_default none` |
| PX4 airframe for Pegasus | `gazebo-classic_iris` | Pegasus default for PX4 ≥ v1.14 |
| ROS 2 | Humble (system install) | for your own nodes and `ros2` CLI |
| px4_msgs | branch **`release/1.14`** | must match the PX4 version |
| Micro-XRCE-DDS-Agent | v2.4.2 | PX4 ↔ ROS 2 bridge |
| QGroundControl | latest AppImage | optional, for manual flying |
| Isaac Lab (optional) | v2.3.0 | Python 3.11, shares the Isaac Sim install; see [§12](#12-using-pegasus-together-with-isaac-lab) |

## 1. How the pieces connect

```
┌─────────────────────────── Isaac Sim 5.1 (Python 3.11) ───────────────────────────┐
│  Scene (USD)  +  Pegasus Multirotor at /World/quadrotor                            │
│     │  physics, IMU / GPS / baro / mag                                              │
│     ├── PX4MavlinkBackend ── MAVLink TCP :4560 ──►  PX4 SITL v1.14.3 (auto-launched)│
│     │                      ◄── motor commands ───                                   │
│     └── ROS2Backend (internal Humble libs) ──► /drone1/state/*, /drone1/camera/*    │
└────────────────────────────────────────────────────────────────────────────────────┘
                                   PX4 SITL
                                   ├── MAVLink UDP :14550 ──► QGroundControl
                                   └── uxrce_dds_client UDP :8888 ──► MicroXRCEAgent ──► /fmu/in/*, /fmu/out/*

                       System ROS 2 Humble (Python 3.10): ros2 CLI, rviz2, your nodes
```

Key points:

- **Pegasus starts PX4 for you** (`px4_autolaunch`). You do not run `make px4_sitl ...` yourself while Isaac Sim is running.
- PX4 v1.14 SITL starts `uxrce_dds_client` on UDP 8888 by default. Starting `MicroXRCEAgent udp4 -p 8888` is enough to get `/fmu/*` topics.
- Isaac Sim uses its **own** internal ROS 2 Humble libraries. Never `source /opt/ros/humble/setup.bash` in the terminal that starts Isaac Sim. The `isaac_run` function in step 2 takes care of this.

---

## 2. Install Isaac Sim 5.1.0 (workstation)

```bash
cd ~
mkdir -p isaacsim
cd isaacsim

# Download the Isaac Sim 5.1.0 standalone package (~ several GB)
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip

# Post-install scripts
./post_install.sh
./isaac-sim.selector.sh

rm isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

> The Docker install in [isaac-sim.md](isaac-sim.md) is useful for headless or remote GPU machines.
> Pegasus is only tested with the workstation install, and the rest of this guide assumes it.

### 2.1 Configure environment variables and `isaac_run`

Add this to `~/.bashrc` (taken from the Pegasus guide).
`isaac_run` strips the system ROS 2 environment and points Isaac Sim at its internal ROS 2 Humble bridge libraries.
That way your `~/.bashrc` can keep `source /opt/ros/humble/setup.bash` for normal terminals.

```bash
# ---------------------------
# ISAAC SIM SETUP
# ---------------------------
export ISAACSIM_PATH="${HOME}/isaacsim"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
export ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

# Launch Isaac Sim or run scripts with Isaac Sim's python,
# avoiding conflicts between system ROS 2 (Python 3.10) and Isaac Sim (Python 3.11)
isaac_run() {
    if [ ! -x "$ISAACSIM_PYTHON" ]; then
        echo "❌ IsaacSim python.sh not found at: $ISAACSIM_PYTHON"
        return 1
    fi
    if [ ! -x "$ISAACSIM" ]; then
        echo "❌ IsaacSim launcher not found at: $ISAACSIM"
        return 1
    fi

    # Clean ROS 2 environment
    unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH
    local ros_paths=("/opt/ros/humble" "/opt/ros/jazzy" "/opt/ros/iron")
    for ros_path in "${ros_paths[@]}"; do
        export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "^${ros_path}" | paste -sd':' -)
    done

    # Use Isaac Sim's internal ROS 2 libraries
    if [ -f /etc/os-release ]; then
        UBUNTU_VERSION=$(grep "^VERSION_ID=" /etc/os-release | cut -d'"' -f2)
    fi
    if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
        export ROS_DISTRO=jazzy
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/jazzy/lib"
        echo "🧩 Detected Ubuntu 24.04 -> Using ROS_DISTRO=jazzy"
    else
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"
        echo "🧩 Detected Ubuntu ${UBUNTU_VERSION:-unknown} -> Using ROS_DISTRO=humble"
    fi

    if [ $# -eq 0 ]; then
        echo "🧠 Launching Isaac Sim GUI..."
        "${ISAACSIM}"
    elif [[ "$1" == --* ]]; then
        echo "⚙️  Launching Isaac Sim with options: $*"
        "${ISAACSIM}" "$@"
    elif [ -f "$1" ]; then
        local SCRIPT_PATH="$1"
        shift
        echo "🚀 Running Python script with Isaac Sim: $SCRIPT_PATH"
        "${ISAACSIM_PYTHON}" "$SCRIPT_PATH" "$@"
    else
        echo "❌ Unknown argument or file not found: '$1'"
        echo "Usage:"
        echo "  isaac_run                 → launch GUI"
        echo "  isaac_run my_script.py    → run script with IsaacSim Python"
        echo "  isaac_run --headless ...  → launch IsaacSim with CLI flags"
        return 1
    fi
}
```

Run `source ~/.bashrc`, or open a new terminal.

> `isaac_run` changes the environment of the terminal it runs in: ROS 2 is unset there afterwards. Use a dedicated terminal for Isaac Sim, and other terminals for `ros2` commands.

### 2.2 Verify Isaac Sim

Do not continue until all of these work:

```bash
isaac_run --help        # prints options
isaac_run               # GUI window opens (first start can take several minutes: shader cache)

"$ISAACSIM_PYTHON" -c "print('Hello World.')"
"$ISAACSIM_PYTHON" "${ISAACSIM_PATH}/standalone_examples/api/isaacsim.core.api/add_cubes.py"
```

> The NVIDIA scene assets (grid rooms, warehouse, etc.) are streamed from NVIDIA's cloud asset server on first use, so the machine needs internet access.

---

## 3. Build PX4-Autopilot v1.14.3

Follow [px4_SITL.md](px4_SITL.md) **steps 0–2** (clone v1.14.3, `ubuntu.sh`, Python deps, `make px4_sitl_default none`).
Short version:

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.3
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh          # log out and back in afterwards
make px4_sitl_default none            # stop it with `shutdown` once you see pxh>
```

Keep it at `~/PX4-Autopilot`, which is Pegasus's default path. If you put it elsewhere, set the path in step 4.3.

---

## 4. Install Pegasus Simulator v5.1.0

### 4.1 Clone at the matching tag

```bash
cd ~
git clone https://github.com/PegasusSimulator/PegasusSimulator.git
cd PegasusSimulator
git checkout v5.1.0
```

### 4.2 Enable the extension (GUI mode)

1. `isaac_run`
2. `Window -> Extensions`
3. Click the settings (☰ / gear) icon and add the extension search path `~/PegasusSimulator/extensions`.
4. In the **Third-party** tab, enable **Pegasus Simulator**. Optionally tick *Autoload*.
5. The first time, it installs its Python requirements into Isaac Sim's Python. After a few seconds the **Pegasus Simulator** panel appears, docked bottom-right.

### 4.3 Install the extension as a Python library (standalone scripts)

Standalone scripts need this so that `import pegasus.simulator` works:

```bash
cd ~/PegasusSimulator/extensions
"$ISAACSIM_PYTHON" -m pip install --editable pegasus.simulator
```

### 4.4 Check PX4 path and airframe

In the Pegasus panel, the **PX4 Path** field should be `~/PX4-Autopilot`. If you change it, press **Make Default**.
The same settings live in `~/PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml`:

```yaml
px4_dir: ~/PX4-Autopilot
px4_default_airframe: gazebo-classic_iris   # use 'iris' only for PX4 < v1.14
```

---

## 5. First flight: extension mode + QGroundControl

### 5.1 Install QGroundControl (once)

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y        # ModemManager grabs serial ports, see ../../troubleshooting/modem.md
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libqt5gui5 libfuse2
# download QGroundControl-x86_64.AppImage from https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
chmod +x ./QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

Log out and back in after `usermod`.

### 5.2 Fly

1. `isaac_run`, and make sure the Pegasus extension is enabled.
2. Pegasus panel → **Load Scene** (for example *Default Environment*).
3. Pegasus panel → **Load Vehicle** (Iris, PX4 backend).
4. Press **Play**. Pegasus launches PX4 in the background, and the terminal shows PX4's log.
5. In QGroundControl the vehicle appears. Use **Takeoff**, then click the map → **Go to location**.

✅ **Success:** the drone takes off in the Isaac Sim viewport and follows the waypoint.

---

## 6. Standalone mode (Python scripts)

Everything the GUI does can be scripted. This is what Drone Ranger uses for repeatable experiments.

```bash
cd ~/PegasusSimulator/examples
isaac_run 1_px4_single_vehicle.py
```

This loads a scene and spawns a PX4-controlled Iris at `/World/quadrotor`.
Fly it from QGroundControl or from `pxh>` as in step 7.

The essential part of the script:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})   # must come before any other omni/isaac import

from omni.isaac.core.world import World
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation

pg = PegasusInterface()
pg._world = World(**pg._world_settings)
pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])   # or a path to your own .usd

config = MultirotorConfig()
config.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
    "vehicle_id": 0,
    "px4_autolaunch": True,
    "px4_dir": pg.px4_path,
    "px4_vehicle_model": pg.px4_default_airframe,   # gazebo-classic_iris for PX4 v1.14
}))]

Multirotor("/World/quadrotor", ROBOTS["Iris"], 0, [0.0, 0.0, 0.07],
           Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(), config=config)
pg.world.reset()
```

---

## 7. Connect PX4 to ROS 2 (uXRCE-DDS → `/fmu/*`)

You need Micro-XRCE-DDS-Agent and `px4_msgs` (**branch `release/1.14`**) from [px4_SITL.md](px4_SITL.md) steps 3–5.

| Terminal | Command | Environment |
|---|---|---|
| 1 | `MicroXRCEAgent udp4 -p 8888` | any |
| 2 | `isaac_run ~/PegasusSimulator/examples/1_px4_single_vehicle.py` | **no** ROS sourced (handled by `isaac_run`) |
| 3 | `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`<br>`ros2 topic list \| grep fmu`<br>`ros2 topic echo /fmu/out/vehicle_local_position` | system ROS 2 |

Once the simulation is playing, terminal 1 prints `create_client` / `establish_session` lines, and terminal 3 lists `/fmu/in/*` and `/fmu/out/*`.

To take off without QGroundControl, type in the PX4 shell. The PX4 console is in terminal 2's log; for an interactive shell, use `python3 ~/PX4-Autopilot/Tools/mavlink_shell.py 0.0.0.0:14550` (close QGroundControl first, both use UDP 14550):

```bash
commander arm
commander takeoff
```

✅ **Success:** `z` in `/fmu/out/vehicle_local_position` goes negative (NED, metres up), and the drone climbs in the viewport.

---

## 8. Cameras and depth on ROS 2

Pegasus can attach cameras to the vehicle and publish them through its `ROS2Backend`.
Example `8_camera_vehicle.py` does this. Copy it and set `px4_dir` (the example hard-codes `/home/marcelo/...`):

```python
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

config.backends = [
    PX4MavlinkBackend(PX4MavlinkBackendConfig({"vehicle_id": 0, "px4_autolaunch": True,
                                                "px4_dir": pg.px4_path,
                                                "px4_vehicle_model": pg.px4_default_airframe})),
    ROS2Backend(vehicle_id=1, config={
        "namespace": "drone",          # topics become /drone<vehicle_id>/...
        "pub_sensors": False,
        "pub_graphical_sensors": True,
        "pub_state": True,
        "sub_control": False,          # PX4 controls the motors, not ROS
    }),
]
config.graphical_sensors = [MonocularCamera("camera", config={
    "depth": True,
    "resolution": (640, 480),          # keep it small; the default 1920x1200 is heavy
    "frequency": 30,
})]
```

Resulting topics, with `namespace="drone"` and `vehicle_id=1`:

| Topic | Type |
|---|---|
| `/drone1/camera/color/image_raw` | `sensor_msgs/Image` (RGB) |
| `/drone1/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/drone1/camera/depth` | `sensor_msgs/Image`, `32FC1`, metres |
| `/drone1/state/pose`, `/drone1/state/twist`, ... | vehicle state (ENU) |

Check from a system ROS 2 terminal:

```bash
ros2 topic hz /drone1/camera/depth
ros2 run rqt_image_view rqt_image_view /drone1/camera/color/image_raw
```

The depth checkers in [`simulation/isaac_sim/usr_scripts/`](../../../simulation/isaac_sim/usr_scripts/) subscribe to `/depth`. Remap them to the Pegasus topic:

```bash
python3 simulation/isaac_sim/usr_scripts/depth_checker_purenp.py --ros-args -r /depth:=/drone1/camera/depth
```

Alternatives: the manual OmniGraph camera in [cameras.md](cameras.md), or `setup_drone_cameras.py`, which publishes `/drone/rgb` and `/drone/depth`.
Both need a camera prim under the vehicle body.
Pegasus spawns the body at `/World/quadrotor/body`; check the exact path in the *Stage* panel before running those scripts.

---

## 9. Use your own scene (reconstructed backyard)

`pg.load_environment()` accepts any USD path:

1. Build the mesh as in [colmap_pipeline.md](../../reconstruction/colmap_pipeline.md) (`scene_clean.obj`).
2. Convert or import it into Isaac Sim, add collision, and save the stage as a `.usd` (Phase 6 of that guide / [`import_scene_mesh.py`](../../../simulation/isaac_sim/usr_scripts/import_scene_mesh.py)).
3. In your standalone script: `pg.load_environment("/path/to/backyard.usd")`.
4. Spawn the drone above the ground of your scene (adjust the `[x, y, z]` spawn position).

The COLMAP reconstruction has no metric scale. Fix scale and orientation before flying, otherwise depth values will not match reality.

---

## 10. Terminal layout summary

```
Terminal 1  MicroXRCEAgent udp4 -p 8888                         (PX4 → ROS 2 /fmu/*)
Terminal 2  isaac_run my_drone_ranger_sim.py                    (Isaac Sim + Pegasus, auto-launches PX4)
Terminal 3  source ROS 2 + ~/ros2_ws; ros2 topic list / echo    (inspect /fmu/* and /drone1/*)
Optional    ./QGroundControl-x86_64.AppImage                    (manual flight, UDP 14550)
```

## 11. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `rclpy` / `_rclpy_pybind11` import errors, or ROS 2 bridge fails to start | Isaac Sim was launched from a terminal with `/opt/ros/humble` sourced. Launch with `isaac_run` instead. |
| `ModuleNotFoundError: pegasus` in a standalone script | Step 4.3 not done, or the script was run with system `python3` instead of `isaac_run`. |
| Play pressed but the drone never arms; no PX4 log | Wrong **PX4 Path** (step 4.4), or PX4 was never built with `make px4_sitl_default none`. |
| PX4 starts but reports an unknown airframe | Set `px4_default_airframe: gazebo-classic_iris` for PX4 v1.14.x. |
| `/fmu/*` topics listed but `ros2 topic echo` shows nothing or errors | `px4_msgs` branch does not match PX4. Use `release/1.14`, rebuild `~/ros2_ws`. |
| No `/fmu/*` topics at all | MicroXRCEAgent not running on UDP 8888, or a different `ROS_DOMAIN_ID` on the two sides. |
| Isaac Sim topics (`/drone1/*`) not visible from the terminal | Same `ROS_DOMAIN_ID` (default 0) and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on both sides. |
| QGroundControl does not see the vehicle | Simulation not playing; or another program bound UDP 14550. |
| `[Fatal] [gpu.foundation.plugin] Out of resource descriptors` | Raise the open-file limit: `DefaultLimitNOFILE=65535` in `/etc/systemd/user.conf` and `* soft/hard nofile 65535` in `/etc/security/limits.conf`, then reboot. |
| Depth image is all `inf` | Nothing within the camera clipping range (default 0.05–100 m), or the camera faces the sky. |

---

## 12. Using Pegasus together with Isaac Lab

**Python versions do not conflict on this stack.**
Isaac Sim 5.1, Pegasus v5.1.0 and Isaac Lab v2.3.0 all run on Isaac Sim's Python **3.11**.
The 3.10 vs 3.11 conflict people remember comes from Isaac Sim 4.x (Python 3.10), or from ROS 2 Humble's Python 3.10.
What does need care:

| Issue | What happens | What to do |
|---|---|---|
| One shared interpreter | With the `_isaac_sim` symlink ([isaac_lab.md](../../auto_planning/isaac_lab.md)), `./isaaclab.sh --install` installs torch, RL libraries and version pins into the **same** `~/isaacsim/python.sh` that Pegasus uses | Install in a fixed order and re-check Pegasus afterwards (below) |
| Active conda env | `isaaclab.sh` uses `$CONDA_PREFIX/bin/python` whenever a conda env is active, **including `base`**. Install and run then happen in a different Python, often not 3.11 | `conda deactivate` before every `isaaclab.sh` call (the `isaaclab_run` wrapper checks this) |
| System ROS 2 sourced | Same Python 3.10 `rclpy` problem as in §2.1; `isaaclab.sh` does not clean the environment | Use the `isaaclab_run` wrapper in [isaac_lab.md](../../auto_planning/isaac_lab.md) |
| Different Kit app | Isaac Lab scripts start `isaaclab.python*.kit`, which does not load Pegasus or its dependencies (`omni.isaac.core`, `omni.isaac.dynamic_control`, `isaacsim.replicator.agent.core`) | Only if you really need Pegasus inside an Isaac Lab script: `--kit_args "--ext-folder ${HOME}/PegasusSimulator/extensions --enable pegasus.simulator"` |
| One simulation context | Pegasus's `World` and Isaac Lab's `SimulationContext` both derive from the same `isaacsim.core.api` singleton. Isaac Lab also runs many cloned envs on the GPU, while each PX4 vehicle needs its own lockstep SITL process | Do **not** mix them in one process; use them for separate jobs (below) |

### Recommended split

```
Isaac Lab (isaaclab_run / isaaclab.sh -p)            Pegasus + PX4 (isaac_run)
──────────────────────────────────────────          ─────────────────────────────────────
policy / perception training                         SITL validation with the real autopilot
thousands of parallel envs, no PX4                   1..N vehicles, PX4 v1.14.3 in lockstep
e.g. Isaac-Quadcopter-Direct-v0 as a starting point  /fmu/* via uXRCE-DDS, cameras via ROS 2
              │                                                     ▲
              └──── trained policy (.pt / .onnx) ───────────────────┘
                    run as a ROS 2 node → /fmu/in/trajectory_setpoint (offboard)
        both load the same scene USD (e.g. backyard.usd from colmap_pipeline.md)
```

Same Isaac Sim install, two entry points, one shared scene asset.
Training does not need PX4. Validation does not need Isaac Lab.

### Install order on a shared Isaac Sim

1. Isaac Sim 5.1.0 + `isaac_run` (§2), verified.
2. Pegasus (§4), then run `isaac_run ~/PegasusSimulator/examples/1_px4_single_vehicle.py` once.
3. Isaac Lab ([isaac_lab.md](../../auto_planning/isaac_lab.md)), with conda deactivated.
4. Check both still work and record the environment:
   ```bash
   "$ISAACSIM_PYTHON" -m pip check
   isaac_run ~/PegasusSimulator/examples/1_px4_single_vehicle.py                 # Pegasus still OK
   isaaclab_run ~/IsaacLab/scripts/tutorials/00_sim/create_empty.py               # Isaac Lab OK
   "$ISAACSIM_PYTHON" -m pip freeze > ~/isaacsim_python_freeze.txt                # keep this with your experiment notes
   ```

### Fully isolated alternative

If the Isaac Lab dependencies keep breaking Pegasus, install Isaac Lab against a **pip-installed** Isaac Sim instead of the symlink.
That means a Python 3.11 `uv`/conda env with `pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com`,
following the Isaac Lab pip installation guide.
Pegasus keeps the untouched `~/isaacsim` binary install.
This costs a second copy of Isaac Sim on disk (tens of GB), but the two environments can no longer affect each other.

