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
  liburdfdom-dev \
  ros-jazzy-eigen3-cmake-module \
  ros-jazzy-hpp-fcl \
  ros-jazzy-grid-map \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2
```

### Pinocchio (robotpkg)

On Jazzy, `rosdep` currently resolves `pinocchio` to `ros-jazzy-pinocchio` which is not released. We install Pinocchio from OpenRobots
robotpkg instead:

```bash
sudo apt install -y curl ca-certificates gnupg lsb-release
sudo install -d -m 0755 /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc >/dev/null
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(. /etc/os-release && echo $VERSION_CODENAME) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list >/dev/null
sudo apt update
sudo apt install -y robotpkg-pinocchio robotpkg-coal
```

Make sure CMake can find robotpkg installs:

```bash
export CMAKE_PREFIX_PATH=/opt/openrobots:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=/opt/openrobots/lib:${LD_LIBRARY_PATH}
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

# Clone OCS2 (ROS 2 Jazzy port)
git clone --branch ros2 https://github.com/leggedrobotics/ocs2.git

# Robotic assets (required by several examples/tests)
git clone --branch ros2 https://github.com/leggedrobotics/ocs2_robotic_assets.git

# Plane segmentation packages used by perceptive examples (convex_plane_decomposition*, grid_map_filters_rsl).
# These packages live in elevation_mapping_cupy but can be fetched without the rest of the repo using sparse-checkout.
git clone --filter=blob:none --sparse --branch ros2 https://github.com/leggedrobotics/elevation_mapping_cupy.git
cd elevation_mapping_cupy
git sparse-checkout set plane_segmentation
cd ..

cd ~/ocs2_ws
source /opt/ros/jazzy/setup.bash

# rosdep setup (only once per machine)
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys pinocchio

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
