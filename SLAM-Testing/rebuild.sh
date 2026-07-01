#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "🧹 Cleaning up old build, install, and log files..."
rm -rf build install log

echo "📦 Running a clean Colcon build..."
colcon build

echo "🔄 Sourcing the environment..."
# This allows the script to source the workspace for your current terminal session
source install/setup.bash

echo "🚀 Launching Miloje SLAM Stack..."
ros2 launch lidar_pkg miloje_slam.launch.py
