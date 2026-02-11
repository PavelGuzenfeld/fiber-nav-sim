#!/bin/bash
# Phase 0 GO/NO-GO Test: O3DE Headless GPU Rendering in Docker
#
# Tests:
# 1. Vulkan detects NVIDIA GPU (not llvmpipe)
# 2. O3DE GameLauncher starts in console mode
# 3. ROS 2 camera topic receives non-empty images
#
# Pass criteria: All 3 tests pass
# Fail action: Evaluate Xvfb+GPU fallback, then abort if that also fails

set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0
PROJECT_PATH="${PROJECT_PATH:-/opt/project}"
PROJECT_NAME="${PROJECT_NAME:-HeadlessTest}"

pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS + 1)); }
fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL + 1)); }
info() { echo -e "${YELLOW}[INFO]${NC} $1"; }

cleanup() {
    info "Cleaning up..."
    # Kill GameLauncher if running
    pkill -f "${PROJECT_NAME}.GameLauncher" 2>/dev/null || true
    # Kill Xvfb if we started it
    pkill -f "Xvfb :99" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT

echo "========================================="
echo " O3DE Phase 0: Headless Docker GPU Test"
echo "========================================="
echo ""

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# ─── Test 1: Vulkan GPU Detection ───
info "Test 1: Checking Vulkan GPU..."

if ! command -v vulkaninfo &> /dev/null; then
    fail "vulkaninfo not found"
else
    VULKAN_OUTPUT=$(vulkaninfo --summary 2>&1 || true)
    GPU_NAME=$(echo "$VULKAN_OUTPUT" | grep "deviceName" | head -1 | sed 's/.*= //')

    if [ -z "$GPU_NAME" ]; then
        fail "No Vulkan device detected"
    elif echo "$GPU_NAME" | grep -qi "llvmpipe\|software\|mesa"; then
        fail "Vulkan using software renderer: $GPU_NAME"
    else
        pass "Vulkan GPU detected: $GPU_NAME"

        # Show VRAM info
        VRAM=$(echo "$VULKAN_OUTPUT" | grep "deviceMemory" | head -1 | sed 's/.*= //' || true)
        if [ -n "$VRAM" ]; then
            info "  VRAM: $VRAM"
        fi
    fi
fi

# ─── Test 2: O3DE GameLauncher Starts ───
info "Test 2: Starting O3DE GameLauncher in console mode..."

LAUNCHER="${PROJECT_PATH}/build/linux/bin/profile/${PROJECT_NAME}.GameLauncher"

if [ ! -f "$LAUNCHER" ]; then
    fail "GameLauncher not found at $LAUNCHER"
else
    # Start GameLauncher in background with console mode
    # Give it 60 seconds to initialize (asset loading can be slow)
    $LAUNCHER -console-mode \
        --regset="/O3DE/Atom/Bootstrap/CreateNativeWindow=false" \
        > /tmp/o3de_launcher.log 2>&1 &
    LAUNCHER_PID=$!

    info "  GameLauncher PID: $LAUNCHER_PID"
    info "  Waiting for initialization (up to 60s)..."

    # Wait for the launcher to either crash or start publishing
    STARTED=false
    for i in $(seq 1 60); do
        if ! kill -0 $LAUNCHER_PID 2>/dev/null; then
            # Process exited
            EXIT_CODE=$(wait $LAUNCHER_PID 2>/dev/null || echo $?)
            fail "GameLauncher exited with code $EXIT_CODE after ${i}s"
            info "  Last 20 lines of log:"
            tail -20 /tmp/o3de_launcher.log 2>/dev/null || true
            break
        fi

        # Check if ROS 2 topics are being published (sign of successful start)
        TOPICS=$(ros2 topic list 2>/dev/null || true)
        if echo "$TOPICS" | grep -q "camera\|image\|sensor"; then
            STARTED=true
            pass "GameLauncher started successfully (${i}s, ROS 2 topics visible)"
            break
        fi

        sleep 1
    done

    if [ "$STARTED" = false ] && kill -0 $LAUNCHER_PID 2>/dev/null; then
        # Still running but no camera topics after 60s
        # Check if at least ROS 2 node is alive
        NODES=$(ros2 node list 2>/dev/null || true)
        if [ -n "$NODES" ]; then
            pass "GameLauncher running (60s, ROS 2 nodes: $NODES) — no camera topic yet"
        else
            fail "GameLauncher running but no ROS 2 nodes after 60s"
            info "  Last 20 lines of log:"
            tail -20 /tmp/o3de_launcher.log 2>/dev/null || true
        fi
    fi
fi

# ─── Test 3: Camera Image Capture ───
info "Test 3: Checking camera image output..."

if ! kill -0 $LAUNCHER_PID 2>/dev/null; then
    fail "GameLauncher not running — cannot test camera"
else
    # List all image-related topics
    ALL_TOPICS=$(ros2 topic list 2>/dev/null || true)
    IMAGE_TOPICS=$(echo "$ALL_TOPICS" | grep -i "image\|camera" || true)

    if [ -z "$IMAGE_TOPICS" ]; then
        fail "No camera/image topics found. Available topics:"
        echo "$ALL_TOPICS" | head -20
    else
        info "  Found image topics:"
        echo "$IMAGE_TOPICS" | while read -r t; do echo "    $t"; done

        # Try to capture one image from the first topic
        TOPIC=$(echo "$IMAGE_TOPICS" | head -1)
        info "  Capturing one frame from $TOPIC (timeout 30s)..."

        # Use Python to capture and validate the image
        python3 -c "
import sys
import time

# Minimal ROS 2 image check
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node('headless_test')

    received = {'img': None}

    def callback(msg):
        received['img'] = msg

    sub = node.create_subscription(Image, '$TOPIC', callback, 10)

    start = time.time()
    while received['img'] is None and (time.time() - start) < 30:
        rclpy.spin_once(node, timeout_sec=0.5)

    if received['img'] is None:
        print('FAIL: No image received in 30s')
        sys.exit(1)

    img = received['img']
    print(f'Image: {img.width}x{img.height}, encoding={img.encoding}, step={img.step}')

    # Check non-black (at least some pixel variation)
    data = bytes(img.data)
    if len(data) == 0:
        print('FAIL: Image data is empty')
        sys.exit(1)

    # Sample pixels across the image
    total = sum(data[::100])  # sample every 100th byte
    if total == 0:
        print('FAIL: Image appears to be all black')
        sys.exit(1)

    unique_values = len(set(data[::100]))
    print(f'Pixel variation: {unique_values} unique values in sample')
    print(f'Total brightness (sampled): {total}')
    print('PASS: Camera produced valid non-black image')
    sys.exit(0)

except Exception as e:
    print(f'FAIL: {e}')
    sys.exit(1)
finally:
    try:
        node.destroy_node()
        rclpy.shutdown()
    except:
        pass
" && pass "Camera produces valid image" || fail "Camera image validation failed"
    fi
fi

# ─── Summary ───
echo ""
echo "========================================="
echo " Results: ${PASS} passed, ${FAIL} failed"
echo "========================================="

if [ "$FAIL" -eq 0 ]; then
    echo -e "${GREEN}GO: Phase 0 passed — O3DE headless GPU rendering works${NC}"
    exit 0
else
    echo -e "${RED}NO-GO: Phase 0 failed — see failures above${NC}"
    echo ""
    echo "Fallback options:"
    echo "  1. Try Xvfb with different GPU config"
    echo "  2. Try VirtualGL + Xvfb"
    echo "  3. Abort O3DE migration"
    exit 1
fi
