#!/bin/bash
# Run full PX4 SITL simulation with Foxglove visualization
# Uses Xvfb virtual display for controlled framerate rendering

set -e

echo "=========================================="
echo "PX4 SITL + Foxglove Demo"
echo "=========================================="

# Install required packages
echo "Installing required packages..."
apt-get update >/dev/null 2>&1
apt-get install -y xvfb ros-jazzy-foxglove-bridge >/dev/null 2>&1

# Configuration
export DISPLAY=:99
export PX4_HOME=/root/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/models:/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds

# Start Xvfb virtual display
echo "Starting Xvfb virtual display..."
Xvfb :99 -screen 0 1280x720x24 &
XVFB_PID=$!
sleep 2

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash

# Start Foxglove bridge
echo "Starting Foxglove bridge on port 8765..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
FOXGLOVE_PID=$!
sleep 2

# Start Gazebo with canyon world and quadtailsitter
echo "Starting Gazebo Harmonic with quadtailsitter..."
gz sim -r /root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/worlds/canyon_harmonic.sdf &
GZ_PID=$!
sleep 5

# Spawn the quadtailsitter model
echo "Spawning quadtailsitter at altitude 10m..."
gz service -s /world/canyon_world/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    -r "sdf_filename: '/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/models/quadtailsitter/model.sdf' pose: { position: { z: 10.0 } } name: 'quadtailsitter'" &
sleep 3

# Start MicroXRCE-DDS Agent
echo "Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &
DDS_PID=$!
sleep 2

# Start PX4 SITL
echo "Starting PX4 SITL..."
cd ${PX4_HOME}/build/px4_sitl_default/rootfs
rm -f dataman parameters*.bson
PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4 &
PX4_PID=$!
sleep 10

# Start ros_gz_bridge for sensor topics
echo "Starting ros_gz_bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /model/quadtailsitter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /world/canyon_world/model/quadtailsitter/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /world/canyon_world/model/quadtailsitter/link/base_link/sensor/camera_down/image@sensor_msgs/msg/Image[gz.msgs.Image &
BRIDGE_PID=$!
sleep 2

# Start sensor simulators
echo "Starting sensor simulators..."
ros2 run fiber_nav_sensors spool_sim_driver \
    --ros-args -p odom_topic:=/model/quadtailsitter/odometry &
SPOOL_PID=$!

ros2 run fiber_nav_sensors vision_direction_sim \
    --ros-args -p odom_topic:=/model/quadtailsitter/odometry &
VISION_PID=$!

# Start fusion node
echo "Starting fiber vision fusion..."
ros2 run fiber_nav_fusion fiber_vision_fusion &
FUSION_PID=$!

echo ""
echo "=========================================="
echo "Demo running!"
echo "=========================================="
echo ""
echo "Connect Foxglove Studio to: ws://localhost:8765"
echo "Load layout: foxglove/fiber_nav_layout.json"
echo ""
echo "QGroundControl: Connect to UDP localhost:14550"
echo ""
echo "Press Ctrl+C to stop..."
echo ""

# Cleanup function
cleanup() {
    echo "Stopping all processes..."
    kill $FUSION_PID $VISION_PID $SPOOL_PID $BRIDGE_PID $PX4_PID $DDS_PID $GZ_PID $FOXGLOVE_PID $XVFB_PID 2>/dev/null || true
    echo "Done!"
}
trap cleanup EXIT

# Wait
wait
