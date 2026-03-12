#!/bin/bash
set -e

echo "=== O3DE Headless GPU Prerequisite Test ==="
echo "    Native Ubuntu + NVIDIA Vulkan ICD"
echo ""

# Test 1: NVIDIA GPU accessible via CUDA
echo "[1/3] NVIDIA GPU detection..."
if nvidia-smi > /dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=gpu_name --format=csv,noheader 2>/dev/null | head -1)
    echo "  PASS: GPU detected — $GPU_NAME"
else
    echo "  FAIL: nvidia-smi not available"
    echo "  Ensure --gpus all is passed to docker run"
    exit 1
fi

# Test 2: Xvfb starts
echo "[2/3] Xvfb virtual display..."
Xvfb :99 -screen 0 1920x1080x24 &
XVFB_PID=$!
sleep 1
export DISPLAY=:99

if kill -0 $XVFB_PID 2>/dev/null; then
    echo "  PASS: Xvfb running on :99"
else
    echo "  FAIL: Xvfb failed to start"
    exit 1
fi

# Test 3: Vulkan ICD detection (expect NVIDIA native driver)
echo "[3/3] Vulkan device detection..."

# Show available ICDs
echo "  Available Vulkan ICDs:"
for icd in /usr/share/vulkan/icd.d/*.json /usr/local/share/vulkan/icd.d/*.json /etc/vulkan/icd.d/*.json; do
    [ -f "$icd" ] && echo "    $(basename $icd)"
done

echo ""
echo "  Testing Vulkan drivers..."
ALL_OUTPUT=$(vulkaninfo --summary 2>&1 || true)

if echo "$ALL_OUTPUT" | grep -qi "NVIDIA"; then
    echo "  PASS: NVIDIA Vulkan ICD detected"
    echo "$ALL_OUTPUT" | grep -i "deviceName\|driverName\|apiVersion\|driverVersion" | head -8 | sed 's/^/    /'
    VULKAN_OK=true
    VULKAN_MODE="NVIDIA native"
elif echo "$ALL_OUTPUT" | grep -qi "llvmpipe\|lavapipe\|SwiftShader"; then
    echo "  WARN: Only software Vulkan available (llvmpipe)"
    echo "$ALL_OUTPUT" | grep -i "deviceName\|apiVersion" | head -5 | sed 's/^/    /'
    VULKAN_OK=true
    VULKAN_MODE="llvmpipe (software)"
else
    echo "  FAIL: No Vulkan devices detected"
    echo "$ALL_OUTPUT" | head -15 | sed 's/^/    /'
    VULKAN_OK=false
fi

# Cleanup
kill $XVFB_PID 2>/dev/null || true

echo ""
echo "================================"
if [ "${VULKAN_OK:-false}" = "true" ]; then
    echo "RESULT: PASS — Vulkan available via $VULKAN_MODE"
    echo ""
    if [ "$VULKAN_MODE" = "llvmpipe (software)" ]; then
        echo "NOTE: Software rendering will work but may have performance"
        echo "limitations for camera sensor simulation. For production,"
        echo "native NVIDIA Vulkan is recommended."
    fi
    echo ""
    echo "Proceed to full O3DE build with Dockerfile.o3de"
else
    echo "RESULT: FAIL — No Vulkan device available"
    echo "O3DE requires at least software Vulkan (llvmpipe) to render."
    exit 1
fi
