#!/bin/bash
# Run all unit tests
# Usage: ./scripts/run_tests.sh

set -e

echo "=============================================="
echo "  Fiber Navigation - Running Tests"
echo "=============================================="
echo ""

# Source workspace
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash

cd /root/ws

# Build with tests
echo "Building with tests enabled..."
colcon build --symlink-install --packages-skip px4_msgs --cmake-args -DBUILD_TESTING=ON

echo ""
echo "Running tests..."
colcon test --packages-skip px4_msgs

echo ""
echo "Test results:"
colcon test-result --verbose
