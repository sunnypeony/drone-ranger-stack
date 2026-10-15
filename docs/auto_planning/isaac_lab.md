# Isaac Lab Installation

Isaac Lab v2.3.0 on top of the Isaac Sim 5.1.0 workstation install, sharing the same Isaac Sim (Python **3.11**) as Pegasus.
`ISAACSIM_PATH` is the Isaac Sim root folder, see [Pegasus setup §2.1](../simulation/isaac-sim/pegasus.md#21-configure-environment-variables-and-isaac_run).
For how Isaac Lab and Pegasus coexist and what each is used for, read [Pegasus setup §12](../simulation/isaac-sim/pegasus.md#12-using-pegasus-together-with-isaac-lab) first.

## Before you start

- **No conda env active**, not even `base`: run `conda deactivate`.
  If `CONDA_PREFIX` is set, `isaaclab.sh` installs into and runs with the conda Python instead of `_isaac_sim/python.sh`.
- **No system ROS 2 in the environment.** Use a fresh terminal without `/opt/ros/humble` sourced, or the `isaaclab_run` wrapper below.
- Pegasus already installed and verified, so you can check afterwards that it still works.

## Install

```bash
conda deactivate 2>/dev/null || true

cd ~
git clone --branch v2.3.0 https://github.com/isaac-sim/IsaacLab.git
cd ~/IsaacLab

# Link the existing Isaac Sim install into Isaac Lab
ln -s "${ISAACSIM_PATH}" _isaac_sim

# CMAKE_POLICY_VERSION_MINIMUM works around CMake >= 4 rejecting old sub-projects
CMAKE_POLICY_VERSION_MINIMUM=3.5 ./isaaclab.sh --install
```

Pinned packages that were needed to get a clean `pip check`:

```bash
./isaaclab.sh -p -m pip install -e ./source/isaaclab
./isaaclab.sh -p -m pip install "setuptools==80.9.0" wheel
./isaaclab.sh -p -m pip install --no-build-isolation "flatdict==4.0.1"
./isaaclab.sh -p -m pip install "stable-baselines3==2.7.0" "torch==2.7.0+cu128"
./isaaclab.sh -p -m pip install "psutil>=5.9,<6" "packaging==23.2" "wheel==0.45.1" "lxml>=4.9.2,<5" docstring-parser "PyJWT>=1,<3"
./isaaclab.sh -p -m pip check
```

These all go into `~/isaacsim/python.sh`, the interpreter Pegasus uses too.

## `isaaclab_run` wrapper

Add this to `~/.bashrc`, next to `isaac_run`.
It refuses to run with a conda env active and strips system ROS 2 in a subshell, so your terminal keeps its ROS environment:

```bash
export ISAACLAB_PATH="${HOME}/IsaacLab"

isaaclab_run() {
    if [ -n "${CONDA_PREFIX}" ]; then
        echo "❌ conda env '${CONDA_DEFAULT_ENV}' is active: run 'conda deactivate' first"
        return 1
    fi
    (
        unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH
        for ros_path in /opt/ros/humble /opt/ros/jazzy /opt/ros/iron; do
            export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "^${ros_path}" | paste -sd':' -)
        done
        "${ISAACLAB_PATH}/isaaclab.sh" -p "$@"
    )
}
```

## Verify

```bash
isaaclab_run ~/IsaacLab/scripts/tutorials/00_sim/create_empty.py

# Pegasus must still work on the shared interpreter
"$ISAACSIM_PYTHON" -m pip check
isaac_run ~/PegasusSimulator/examples/1_px4_single_vehicle.py

# Record the shared environment for reproducibility
"$ISAACSIM_PYTHON" -m pip freeze > ~/isaacsim_python_freeze.txt
```

A quadrotor starting point for training (no PX4 involved):

```bash
isaaclab_run ~/IsaacLab/scripts/reinforcement_learning/skrl/train.py --task Isaac-Quadcopter-Direct-v0 --headless
```
