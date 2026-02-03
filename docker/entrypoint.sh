#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source ${WORKSPACE}/install/setup.bash
fi

# Set Gazebo Harmonic paths (GZ_SIM_* for Gazebo Harmonic)
export GZ_SIM_RESOURCE_PATH=${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/models:${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

# Legacy Gazebo paths (for compatibility)
export GAZEBO_MODEL_PATH=${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds:${GAZEBO_RESOURCE_PATH}

# PX4 paths (if available)
if [ -d "/root/PX4-Autopilot" ]; then
    export PX4_HOME=/root/PX4-Autopilot
    export PATH=$PATH:$PX4_HOME/build/px4_sitl_default/bin
fi

# Execute command
exec "$@"
