# Isaac Lab Installation

git clone --branch v2.3.0 https://github.com/isaac-sim/IsaacLab.git

ln -s /home/rangerlab/isaac-sim/ _isaac_sim
./isaaclab.sh –install

cd /home/rangerlab/IsaacLab
CMAKE_POLICY_VERSION_MINIMUM=3.5 ./isaaclab.sh --install

./isaaclab.sh -p -m pip install -e ./source/isaaclab
./isaaclab.sh -p -m pip install "setuptools==80.9.0" wheel
./isaaclab.sh -p -m pip install --no-build-isolation "flatdict==4.0.1"
./isaaclab.sh -p -m pip install "stable-baselines3==2.7.0" "torch==2.7.0+cu128"
./isaaclab.sh -p -m pip check

./isaaclab.sh -p -m pip install   "psutil>=5.9,<6"   "packaging==23.2"   "wheel==0.45.1"   "lxml>=4.9.2,<5"   docstring-parser   "PyJWT>=1,<3"


Verify:
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
