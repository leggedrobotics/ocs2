# OCS2 on ROS 2 Jazzy (branch `ros2`)

This branch ports OCS2 from ROS 1/catkin to ROS 2 Jazzy/colcon.

## Prerequisites

- Ubuntu 24.04 + ROS 2 Jazzy installed.
- C++17 compiler.
- `colcon` build tooling.

## System Dependencies (Ubuntu apt)

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep \
  python3-dev pybind11-dev \
  libeigen3-dev libboost-all-dev libglpk-dev \
  libgmp-dev libmpfr-dev libcgal-dev libopencv-dev libpcl-dev \
  ros-jazzy-eigen3-cmake-module \
  ros-jazzy-pinocchio ros-jazzy-hpp-fcl \
  ros-jazzy-grid-map \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2
```

Notes:

- You must `source /opt/ros/jazzy/setup.bash` in every shell where you use `ros2`/`colcon`.
- `blasfeo_catkin` / `hpipm_catkin` keep their names but are ROS 2 `ament_cmake` packages in this branch.
- `ocs2_mpcnet` and `ocs2_raisim` are currently ignored via `COLCON_IGNORE` (not ported to Jazzy yet).
- `rqt_multiplot` is not released for Jazzy on Ubuntu 24.04. Multiplot launch files are optional; build `rqt_multiplot` from source if you need them.

## Build (colcon)

```bash
# Create a workspace
mkdir -p ~/ocs2_ws/src
cd ~/ocs2_ws/src

# Clone OCS2 (this branch)
git clone --branch ros2 https://github.com/leggedrobotics/ocs2.git

# Optional but recommended: assets used by examples/tests
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git

cd ~/ocs2_ws
source /opt/ros/jazzy/setup.bash

# rosdep setup (only once per machine)
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

If `sudo rosdep init` fails with `default sources list file already exists`, you already initialized rosdep on this machine; run only `rosdep update`.

## Run an example

```bash
source /opt/ros/jazzy/setup.bash
source ~/ocs2_ws/install/setup.bash

ros2 launch ocs2_ballbot_ros ballbot_mpc_mrt.launch.py
```

## Build documentation (optional)

Docs are off by default. To build them:

```bash
sudo apt install -y doxygen python3-sphinx
cd ~/ocs2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ocs2_doc --cmake-args -DBUILD_DOCS=ON
```

The HTML output is under `build/ocs2_doc/output/sphinx/index.html`.
