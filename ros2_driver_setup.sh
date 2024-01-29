#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Error: Please run the script with sudo."
  exit 1
fi

echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

echo "Creating a new ROS2 workspace..."
mkdir -p ~/eth_ros2_ws/src
cd ~/eth_ros2_ws/src

if [ -d "ethercat_driver_ros2" ]; then
  echo "ethercat_driver_ros2 repository already exists. Skipping cloning."
else
  echo "Cloning ethercat_driver_ros2 repository..."
  git clone https://github.com/ICube-Robotics/ethercat_driver_ros2.git
fi


echo "Installing ROS2 dependencies..."
cd ~/eth_ros2_ws
rosdep install --ignore-src --from-paths . -y -r

echo "Building ROS2 workspace..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

echo "Sourcing the ROS2 workspace..."
source install/setup.bash

echo "ROS2 environment setup and workspace creation completed."
exit 0

