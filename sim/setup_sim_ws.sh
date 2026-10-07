#!/usr/bin/env bash
# One-shot setup of a standalone f1tenth_gym simulation workspace on a laptop.
# Usage:  ./setup_sim_ws.sh [~/sim_ws] [path/to/maps_latest.tar.gz]
# Requires: Ubuntu 22.04 + ROS 2 Humble already installed (source /opt/ros/humble/setup.bash).
set -euo pipefail
WS=${1:-$HOME/sim_ws}
MAP_TAR=${2:-}
REPO=https://github.com/churrosboy/SKKUDERIA_SKKAI.git
BRANCH=sim-baseline

[ -n "${ROS_DISTRO:-}" ] || { echo "source /opt/ros/humble/setup.bash first"; exit 1; }
[ "$ROS_DISTRO" = humble ] || echo "WARNING: stack was developed on humble, you are on $ROS_DISTRO"

mkdir -p "$WS/src"
if [ ! -d "$WS/src/.git" ]; then
  git clone -b "$BRANCH" "$REPO" "$WS/src"
fi

# Packages that need car hardware / aarch64 prebuilt binaries -> not built in sim.
for d in sensors/soslab_gl5_driver_prebuilt sensors/vesc/vesc_driver sensors/vesc/vesc_ackermann \
         base_system/f1tenth_system/f1tenth_stack base_system/f1tenth_system/teleop_tools \
         system_identification; do
  [ -d "$WS/src/$d" ] && touch "$WS/src/$d/COLCON_IGNORE" || true
done

# apt deps (cartographer_ros only for package.xml resolution; not launched in sim)
sudo apt-get install -y python3-venv python3-colcon-common-extensions ros-humble-rviz2 \
  ros-humble-ackermann-msgs ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-cartographer-ros ros-humble-tf-transformations ros-humble-robot-localization \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager || true
cd "$WS" && rosdep install --from-paths src --ignore-src -r -y || true

# python venv (system-site-packages so rclpy stays visible)
if [ ! -d "$WS/.venv" ]; then
  python3 -m venv --system-site-packages "$WS/.venv"
fi
# shellcheck disable=SC1091
source "$WS/.venv/bin/activate"
pip install -U pip
pip install "gymnasium==0.29.1" "pyglet==1.5.20" numba pyopengl transforms3d
pip install -e "$WS/src/base_system/f110_simulator/f1tenth_gym"
python -c "import f110_gym, gymnasium; print('f110_gym OK')"

# maps are git-ignored -> restore from tarball
if [ -n "$MAP_TAR" ]; then
  tar -xzf "$MAP_TAR" -C "$WS/src/stack_master/maps/"
fi
[ -f "$WS/src/stack_master/maps/latest/latest.yaml" ] || echo "WARNING: maps/latest missing - pass maps_latest.tar.gz as 2nd arg"

cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo
echo "Done. Add to ~/.bashrc:"
echo "  sim() { source $WS/.venv/bin/activate; source $WS/install/setup.bash; export ROS_DOMAIN_ID=77 RACECAR_VERSION=SIM; }"
