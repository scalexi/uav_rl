#!/usr/bin/env bash
set -euo pipefail

# --- Fix for COLCON_TRACE unbound variable ---
: "${COLCON_TRACE:=}"   # ensures COLCON_TRACE is defined (empty if not set)
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${COLCON_CURRENT_PREFIX:=}"
: "${COLCON_PREFIX_PATH:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
: "${PKG_CONFIG_PATH:=}"

### ─────────────────────────────────────────────────────────────────────────────
# PX4 + ROS 2 + MAVROS + SLAM Setup & Model Copy
# This script will:
# 1) Clone & build PX4 v1.15.2 (SITL)
# 2) Prepare a ROS 2 workspace and install MAVLink + MAVROS from source
# 3) Install slam_toolbox (2D) and RTAB-Map (3D)
# 4) Copy custom models into PX4 Gazebo models directory
# 5) Copy a custom airframe init file (4020_gz_x500_lidar_rgbd) into PX4 ROMFS
#
# NOTE: Don’t forget to update your CMakeLists.txt (as needed for your packages).
### ─────────────────────────────────────────────────────────────────────────────

# ===== user-configurable paths =====
ROS_WS="${HOME}/ros2_ws"
PX4_DIR="${HOME}/PX4-Autopilot"
UAV_MODELS_SRC="${ROS_WS}/src/uav_rl/drone_2d_3d_slam/models"
UAV_WORLD_SRC="${ROS_WS}/src/uav_rl/drone_2d_3d_slam/worlds"
PX4_GZ_MODELS_DST="${PX4_DIR}/Tools/simulation/gz/models"
PX4_GZ_WORLD_DST="${PX4_DIR}/Tools/simulation/gz/worlds"

# If you keep your custom airframe init script in your repo, set it here:
AIRFRAME_FILE_SRC="${ROS_WS}/src/uav_rl/drone_2d_3d_slam/models/4020_gz_x500_lidar_rgbd"
CMAKE_FILE_SRC="${ROS_WS}/src/uav_rl/drone_2d_3d_slam/models/CMakeLists.txt"
#---------------# Destination path inside PX4 repo:---
AIRFRAME_FILE_DST="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKE_FILE_DST="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"



# ===== helpers =====
require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "❌ Missing command: $1"; exit 1; }
}

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -e "$src" ]]; then
    mkdir -p "$(dirname "$dst")"
    cp -r "$src" "$dst"
    echo "✓ Copied: $src → $dst"
  else
    echo "⚠ Skipped (not found): $src"
  fi
}

# ===== pre-flight checks =====
require_cmd git
require_cmd tee
require_cmd rosinstall_generator
require_cmd vcs
require_cmd colcon
require_cmd rosdep
require_cmd make
require_cmd bash

echo "=== 1) PX4 v1.15.2 clone & build (SITL) ==="
if [[ ! -d "$PX4_DIR" ]]; then
  cd "${HOME}"
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
fi
cd "$PX4_DIR"
git fetch --all --tags
git checkout v1.15.2
git submodule update --init --recursive
# Build SITL (first build may take a while)
make px4_sitl

echo "=== 2) System packages for tooling ==="
sudo apt update
sudo apt install -y \
  python3-vcstool \
  python3-rosinstall-generator \
  python3-osrf-pycommon

echo "=== 3) Create/enter ROS 2 workspace ==="
mkdir -p "${ROS_WS}/src"
cd "$ROS_WS"

echo "=== 4) Generate repos (MAVLink + MAVROS) ==="
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
rosinstall_generator --format repos --upstream mavros | tee /tmp/mavros.repos
# For latest development branch instead, use:
# rosinstall_generator --format repos --upstream-development mavros | tee /tmp/mavros.repos

echo "=== 5) Import repos into src/ ==="
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos

echo "=== 6) Resolve dependencies via rosdep ==="
rosdep update
rosdep install --from-paths src --ignore-src -y

echo "=== 7) Install GeographicLib datasets (for MAVROS) ==="
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

echo "=== 8) Build ROS 2 workspace ==="
colcon build

echo "=== 9) Source the workspace ==="
# shellcheck disable=SC1091
source "${ROS_WS}/install/setup.bash"

echo "=== 10) Install SLAM tools ==="
# 2D SLAM (slam_toolbox)
sudo apt install -y ros-jazzy-slam-toolbox
# 3D SLAM (RTAB-Map)
sudo apt install -y ros-jazzy-rtabmap-ros

echo "=== 11) Copy custom models into PX4 Gazebo models dir ==="
mkdir -p "$PX4_GZ_MODELS_DST"
if [[ -d "$UAV_MODELS_SRC" ]]; then
  cp -r "${UAV_MODELS_SRC}/." "$PX4_GZ_MODELS_DST/"
  echo "✓ Models copied: ${UAV_MODELS_SRC} → ${PX4_GZ_MODELS_DST}"
else
  echo "⚠ Models source directory not found: ${UAV_MODELS_SRC}"
fi

echo "=== 12) Copy custom airframe init script (4020_gz_x500_lidar_rgbd) ==="
copy_if_exists "$AIRFRAME_FILE_SRC" "$AIRFRAME_FILE_DST"
copy_if_exists "$CMAKE_FILE_SRC" "$CMAKE_FILE_DST"

echo "=== 13) Rebuild PX4 SITL with updated ROMFS ==="
cd "$PX4_DIR"
make px4_sitl

echo "=== 14) Copy custom worlds into PX4 Gazebo worlds dir ==="
mkdir -p "$PX4_GZ_WORLD_DST"
if [[ -d "$UAV_WORLD_SRC" ]]; then
  cp -r "${UAV_WORLD_SRC}/." "$PX4_GZ_WORLD_DST/"
  echo "✓ Worlds copied: ${UAV_WORLD_SRC} → ${PX4_GZ_WORLD_DST}"
else
  echo "⚠ Worlds source directory not found: ${UAV_WORLD_SRC}"
fi

echo
echo "✅ ALL DONE."
echo "• PX4 SITL (v1.15.2) built at: ${PX4_DIR}"
echo "• ROS 2 workspace built at:    ${ROS_WS}"
echo "• Models installed at:         ${PX4_GZ_MODELS_DST}"
echo "• Airframe file (if provided): ${AIRFRAME_FILE_DST}"
echo "📝 Reminder: Don’t forget if you changed PX4 Version you need to change the CMakeLists.txt."
