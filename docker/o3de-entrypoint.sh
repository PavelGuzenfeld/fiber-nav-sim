#!/bin/bash
set -e

# O3DE entrypoint for native Ubuntu + NVIDIA Vulkan
# Starts Xvfb, verifies Vulkan, optionally launches PX4 SITL in HIL mode,
# then starts O3DE GameLauncher before exec'ing the user command.

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace if available
if [ -f /root/ws/install/setup.bash ]; then
    source /root/ws/install/setup.bash
fi

# Start Xvfb (O3DE links against XCB — needs X11 even in console mode)
if [ -z "$DISPLAY" ] || [ "$DISPLAY" = "" ]; then
    export DISPLAY=:99
fi
if ! pgrep -f "Xvfb ${DISPLAY}" > /dev/null 2>&1; then
    Xvfb ${DISPLAY} -screen 0 1920x1080x24 +extension GLX &
    sleep 1
    echo "[o3de-entrypoint] Xvfb started on $DISPLAY"
fi

# Verify Vulkan sees the GPU
if command -v vulkaninfo &> /dev/null; then
    GPU_NAME=$(vulkaninfo --summary 2>/dev/null | grep "deviceName" | head -1 || true)
    if [ -n "$GPU_NAME" ]; then
        echo "[o3de-entrypoint] Vulkan GPU: $GPU_NAME"
    else
        echo "[o3de-entrypoint] WARNING: No Vulkan GPU detected — rendering will use CPU (llvmpipe)"
    fi
fi

# ── PX4 SITL in HIL mode (if USE_PX4=true) ──
if [ "${USE_PX4}" = "true" ] || [ "${USE_PX4}" = "1" ]; then
    echo "[o3de-entrypoint] Starting PX4 SITL in HIL mode..."

    # Start MicroXRCE-DDS Agent
    MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
    echo "[o3de-entrypoint] MicroXRCE-DDS Agent started (UDP 8888)"

    # Start PX4 SITL with O3DE HIL airframe
    PX4_HOME=${PX4_HOME:-/root/PX4-Autopilot}
    cd ${PX4_HOME}/build/px4_sitl_default/rootfs
    rm -f dataman parameters*.bson
    PX4_SYS_AUTOSTART=${AIRFRAME:-4252} ../bin/px4 > /dev/null 2>&1 &
    PX4_PID=$!
    echo "[o3de-entrypoint] PX4 SITL started (PID: $PX4_PID, airframe: ${AIRFRAME:-4252})"

    sleep 3
fi

# ── Start O3DE GameLauncher (if PROJECT_PATH is set) ──
PROJECT_PATH=${PROJECT_PATH:-/opt/HeadlessTest}
PROJECT_NAME=${PROJECT_NAME:-HeadlessTest}
LAUNCHER="${PROJECT_PATH}/build/linux/bin/profile/${PROJECT_NAME}.GameLauncher"

if [ -f "$LAUNCHER" ] && [ "${START_O3DE}" != "false" ]; then
    echo "[o3de-entrypoint] Starting O3DE GameLauncher..."
    $LAUNCHER -console-mode \
        --regset="/O3DE/Atom/Bootstrap/CreateNativeWindow=false" \
        > /tmp/o3de_launcher.log 2>&1 &
    O3DE_PID=$!
    echo "[o3de-entrypoint] GameLauncher started (PID: $O3DE_PID)"

    # Wait for ROS 2 topics to appear (up to 30s)
    for i in $(seq 1 30); do
        if ! kill -0 $O3DE_PID 2>/dev/null; then
            echo "[o3de-entrypoint] WARNING: GameLauncher exited early"
            tail -10 /tmp/o3de_launcher.log 2>/dev/null || true
            break
        fi
        TOPICS=$(ros2 topic list 2>/dev/null || true)
        if echo "$TOPICS" | grep -q "camera\|image\|sensor"; then
            echo "[o3de-entrypoint] GameLauncher ready (${i}s, ROS 2 topics visible)"
            break
        fi
        sleep 1
    done
fi

exec "$@"
