#!/bin/bash
# Full simulation demo with Gazebo Harmonic
# Plane automatically flies to demonstrate sensor fusion
# Usage: ./scripts/run_demo.sh [--headless]

set -e

HEADLESS="false"
if [ "$1" == "--headless" ]; then
    HEADLESS="true"
fi

echo "=============================================="
echo "  Fiber Navigation Simulation Demo"
echo "  Gazebo Harmonic + Sensor Fusion"
echo "=============================================="
echo ""

# Source workspace
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash

# Set Gazebo paths
export GZ_SIM_RESOURCE_PATH=/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/models:/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

echo "Starting simulation with auto-fly enabled..."
echo "  World: canyon_harmonic"
echo "  Model: plane"
echo "  Headless: $HEADLESS"
echo "  Auto-fly: true (thrust=20N, lift=20N)"
echo ""

ros2 launch fiber_nav_bringup simulation.launch.py \
    headless:=$HEADLESS \
    use_px4:=false \
    auto_fly:=true \
    thrust:=20.0 \
    lift:=20.0
