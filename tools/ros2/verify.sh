#!/usr/bin/env bash
set -eo pipefail
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS2 Humble is not installed in this environment (/opt/ros/humble/setup.bash missing)." >&2
  echo "Use Ubuntu 22.04 with Humble installed, or run this script inside the Humble container." >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/../../ros2_ws"
# CMake caches and symlink installs contain absolute paths. Never share them
# between a /work container mount and the host checkout.
build_dir=build
install_dir=install
log_dir=log
if [ -f /.dockerenv ]; then
  build_dir=build-docker
  install_dir=install-docker
  log_dir=log-docker
fi
colcon --log-base "$log_dir" build --build-base "$build_dir" --install-base "$install_dir" --symlink-install --cmake-args -DBUILD_TESTING=ON
source "$install_dir/setup.bash"
set -u
colcon --log-base "$log_dir" test --build-base "$build_dir" --install-base "$install_dir" --executor sequential --ctest-args -j1 --output-on-failure
colcon test-result --test-result-base "$build_dir" --verbose
ros2 run mrider_autonomy offline_demo
