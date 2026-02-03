#!/bin/bash
# Standalone simulation without PX4
# Uses mock attitude publisher for testing sensor fusion
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
echo "  ros2 topic echo /model/plane/odometry"
echo "  ros2 topic echo /sensors/fiber_spool/velocity"
echo "  ros2 topic echo /sensors/vision_direction"
echo "  ros2 topic echo /fmu/in/vehicle_visual_odometry"
echo ""
if [ "$AUTO_FLY" == "false" ]; then
    echo "To make the plane fly, run:"
    echo "  ros2 service call /plane_controller/enable std_srvs/srv/SetBool '{data: true}'"
    echo "Or restart with: ./scripts/run_standalone.sh --fly"
    echo ""
fi

ros2 launch fiber_nav_bringup simulation.launch.py use_px4:=false headless:=false auto_fly:=$AUTO_FLY
