#!/bin/bash
# Standalone simulation without PX4
# Uses stabilized flight controller + mock attitude publisher
# Usage: ./scripts/run_standalone.sh [--fly]

set -e

AUTO_FLY="false"
if [ "$1" == "--fly" ]; then
    AUTO_FLY="true"
fi

echo "=============================================="
echo "  Fiber Navigation - Standalone Mode"
echo "  (No PX4 - using mock attitude)"
echo "=============================================="
echo ""

# Source workspace
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash

# Set Gazebo paths
export GZ_SIM_RESOURCE_PATH=/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/models:/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

echo "Starting standalone simulation..."
echo "  Auto-fly: $AUTO_FLY"
echo ""
echo "Topics to monitor:"
echo "  ros2 topic echo /model/quadtailsitter/odometry"
echo "  ros2 topic echo /sensors/fiber_spool/velocity"
echo "  ros2 topic echo /sensors/vision_direction"
echo "  ros2 topic echo /fmu/in/vehicle_visual_odometry"
echo ""
if [ "$AUTO_FLY" == "true" ]; then
    echo "Stabilized flight controller will auto-enable after valid odometry."
    echo "Foxglove: ws://localhost:8765"
    echo ""
else
    echo "To enable auto-fly, restart with: ./scripts/run_standalone.sh --fly"
    echo ""
fi

ros2 launch fiber_nav_bringup simulation.launch.py use_px4:=false headless:=false auto_fly:=$AUTO_FLY
