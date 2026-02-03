#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source ${WORKSPACE}/install/setup.bash
fi

# Set Gazebo paths
export GAZEBO_MODEL_PATH=${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds:${GAZEBO_RESOURCE_PATH}

# Execute command
exec "$@"
