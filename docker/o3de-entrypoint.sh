#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Start Xvfb (O3DE links against XCB — needs X11 even in console mode)
if [ -z "$DISPLAY" ] || [ "$DISPLAY" = "" ]; then
    export DISPLAY=:99
    Xvfb :99 -screen 0 1920x1080x24 +extension GLX &
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

exec "$@"
