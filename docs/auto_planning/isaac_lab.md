# Isaac Lab Installation

Isaac Lab v2.3.0 on top of the Isaac Sim 5.1.0 workstation install.
`ISAACSIM_PATH` is the Isaac Sim root folder (see [Pegasus setup](../simulation/isaac-sim/pegasus.md#21-configure-environment-variables-and-isaac_run)).

```bash
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

Verify:

```bash
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```
