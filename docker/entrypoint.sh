#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source ${WORKSPACE}/install/setup.bash
fi

# Source PX4
if [ -f "${PX4_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash" ]; then
    source ${PX4_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${PX4_HOME} ${PX4_HOME}/build/px4_sitl_default
fi

# Set Gazebo paths
export GAZEBO_MODEL_PATH=${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=${WORKSPACE}/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds:${GAZEBO_RESOURCE_PATH}

# Execute command
exec "$@"
