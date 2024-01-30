#!/bin/bash

if [ "$EUID" -eq 0 ]; then
  echo "Error: Please don't run the script with sudo."
  exit 1
fi

echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Assuming the script is located one level above 'src'
ROS2_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$ROS2_WS_DIR/src"

echo "Using current ROS2 workspace in: $ROS2_WS_DIR"

# No need to create a new workspace, already in the desired location
cd "$SRC_DIR"

if [ -d "ethercat_driver_ros2" ]; then
  echo "ethercat_driver_ros2 repository already exists. Skipping cloning."
else
  echo "Cloning ethercat_driver_ros2 repository..."
  git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git
fi

cd "$ROS2_WS_DIR"

echo "Installing ROS2 dependencies..."
rosdep install --ignore-src --from-paths . -y -r

echo "Building ROS2 workspace..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

echo "Sourcing the ROS2 workspace..."
source "$ROS2_WS_DIR/install/setup.bash"

echo "ROS2 environment setup and workspace creation completed."
exit 0
