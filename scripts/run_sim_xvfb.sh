#!/bin/bash
# Run Gazebo simulation with virtual display for stable physics
# This enforces real-time simulation rate even without a physical display

set -e

DURATION=${1:-30}
THRUST=${2:-10.0}
LIFT=${3:-0.0}

echo "Starting simulation with Xvfb (virtual display)..."
echo "  Duration: ${DURATION}s"
echo "  Thrust: ${THRUST}N"
echo "  Lift: ${LIFT}N"

# Start Xvfb virtual display
Xvfb :99 -screen 0 1024x768x24 &
XVFB_PID=$!
export DISPLAY=:99
sleep 2

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash

# Start simulation (NOT headless - uses virtual display)
ros2 launch fiber_nav_bringup simulation.launch.py \
    headless:=false \
    auto_fly:=true \
    thrust:=${THRUST} \
    lift:=${LIFT} \
    spawn_z:=30.0 &
SIM_PID=$!

# Wait for simulation to start
sleep 10

echo "Recording flight data for ${DURATION}s..."

# Record flight data
python3 /root/ws/src/fiber-nav-sim/scripts/record_test_flight.py ${DURATION}

echo "Running 3-way comparison..."

# Run analysis
python3 /root/ws/src/fiber-nav-sim/scripts/compare_three_way.py /tmp/flight_data.csv

# Cleanup
kill $SIM_PID 2>/dev/null || true
kill $XVFB_PID 2>/dev/null || true

echo "Done!"
