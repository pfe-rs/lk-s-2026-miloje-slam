#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# 1. Navigate to the repository
cd /home/pfe/repo/SLAM-Testing

# 2. Build only the specific package
echo "lidar_pkg pravljenje"
colcon build --packages-select lidar_pkg

# 3. Source the workspace setup file
echo "source komanda"
source install/setup.bash

# --- Node Execution Setup ---
# Array to keep track of background process IDs (PIDs)
PIDs=()

# Function to cleanly kill all nodes when you press Ctrl+C
cleanup() {
    echo -e "\nGasi sve procese"
    for pid in "${PIDs[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid"
        fi
    done
    exit 0
}

# Trap Ctrl+C (SIGINT) and call the cleanup function
trap cleanup SIGINT

# 4. Launch the nodes in the background (&)
echo "Pokrece node-ove"

ros2 run lidar_pkg lidar_scan_node &
PIDs+=($!)

ros2 run lidar_pkg logger_node &
PIDs+=($!)

ros2 run lidar_pkg slam_mapping_node &
PIDs+=($!)

ros2 run lidar_pkg komunikacija &
PIDs+=($!)

echo "Sve nodeovi rade! Crtl+C za gasenje"

# Keep the script alive so it can catch the Ctrl+C signal
wait