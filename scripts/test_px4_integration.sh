#!/bin/bash
# Test PX4-Gazebo native integration
# Verifies that EKF accepts visual odometry from the fusion node

set -e

echo "=========================================="
echo "PX4-Gazebo Integration Test"
echo "=========================================="

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash

# Start MicroXRCE-DDS agent in background
echo "Starting MicroXRCE-DDS agent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/dds_agent.log 2>&1 &
DDS_PID=$!
sleep 2

# Start PX4 SITL with vision airframe in background
echo "Starting PX4 SITL (airframe 4251 - gz_quadtailsitter_vision)..."
cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
rm -f dataman parameters*.bson
PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4 > /tmp/px4.log 2>&1 &
PX4_PID=$!
sleep 10

# Start fusion node in background
echo "Starting fusion node..."
ros2 run fiber_nav_fusion fiber_vision_fusion > /tmp/fusion.log 2>&1 &
FUSION_PID=$!
sleep 3

# Check EKF status
echo ""
echo "=========================================="
echo "Checking EKF Status..."
echo "=========================================="

timeout 5 ros2 topic echo /fmu/out/estimator_status_flags --qos-reliability best_effort --once 2>/dev/null | grep -E "(cs_ev_vel|cs_gnss_vel|cs_baro_hgt)" || echo "Could not get EKF status"

# Check visual odometry input
echo ""
echo "=========================================="
echo "Checking Visual Odometry Input..."
echo "=========================================="

timeout 5 ros2 topic echo /fmu/in/vehicle_visual_odometry --qos-reliability best_effort --once 2>/dev/null | head -20 || echo "Could not get visual odometry"

# Check vehicle velocity
echo ""
echo "=========================================="
echo "Checking Vehicle Velocity (Ground Truth)..."
echo "=========================================="

timeout 5 ros2 topic echo /model/quadtailsitter/odometry --once 2>/dev/null | grep -A3 "linear:" || echo "Could not get vehicle velocity"

# Cleanup
echo ""
echo "=========================================="
echo "Cleanup..."
echo "=========================================="

kill $FUSION_PID 2>/dev/null || true
kill $PX4_PID 2>/dev/null || true
kill $DDS_PID 2>/dev/null || true

echo ""
echo "Test complete. Check cs_ev_vel above - should be 'true' for successful integration."
echo "Log files: /tmp/px4.log, /tmp/fusion.log, /tmp/dds_agent.log"
