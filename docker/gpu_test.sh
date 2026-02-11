#!/bin/bash
set -e

echo "=== O3DE Headless GPU Prerequisite Test ==="
echo ""

# Test 1: NVIDIA GPU accessible via CUDA
echo "[1/4] NVIDIA GPU detection..."
if nvidia-smi > /dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=gpu_name --format=csv,noheader 2>/dev/null | head -1)
    echo "  PASS: GPU detected — $GPU_NAME"
else
    echo "  FAIL: nvidia-smi not available"
    echo "  Ensure --gpus all is passed to docker run"
    exit 1
fi

# Test 2: Xvfb starts
echo "[2/4] Xvfb virtual display..."
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

# Test 3: D3D12/WSL2 libraries present
echo "[3/4] WSL2 D3D12 libraries..."
WSL_LIBS_OK=true
for lib in libd3d12.so libd3d12core.so libdxcore.so; do
    if [ -f "/usr/lib/wsl/lib/$lib" ]; then
        echo "  OK:   /usr/lib/wsl/lib/$lib"
    else
        echo "  MISS: /usr/lib/wsl/lib/$lib"
        WSL_LIBS_OK=false
    fi
done

if $WSL_LIBS_OK; then
    echo "  PASS: All WSL2 D3D12 libraries present"
else
    echo "  WARN: Some WSL2 D3D12 libs missing (GPU Vulkan may not work)"
fi

# Test 4: Vulkan ICD detection
echo "[4/4] Vulkan device detection..."

# Show available ICDs
echo "  Available Vulkan ICDs:"
for icd in /usr/share/vulkan/icd.d/*.json /usr/local/share/vulkan/icd.d/*.json; do
    [ -f "$icd" ] && echo "    $(basename $icd)"
done

# Try DZN first (GPU via D3D12)
DZN_ICD="/usr/local/share/vulkan/icd.d/dzn_icd.x86_64.json"
if [ -f "$DZN_ICD" ]; then
    echo ""
    echo "  Testing DZN (D3D12→GPU) Vulkan driver..."
    DZN_OUTPUT=$(DISPLAY="" VK_ICD_FILENAMES="$DZN_ICD" vulkaninfo --summary 2>&1 || true)

    if echo "$DZN_OUTPUT" | grep -qi "Direct3D12\|D3D12\|NVIDIA\|Microsoft"; then
        echo "  PASS: DZN Vulkan sees GPU via D3D12"
        echo "$DZN_OUTPUT" | grep -i "deviceName\|apiVersion\|driverVersion" | head -5 | sed 's/^/    /'
        VULKAN_OK=true
        VULKAN_MODE="DZN (GPU via D3D12)"
    else
        echo "  WARN: DZN ICD present but did not detect GPU"
        echo "$DZN_OUTPUT" | head -10 | sed 's/^/    /'
    fi
fi

# Try all ICDs
if [ "${VULKAN_OK:-false}" != "true" ]; then
    echo ""
    echo "  Testing all available Vulkan drivers..."
    ALL_OUTPUT=$(vulkaninfo --summary 2>&1 || true)

    if echo "$ALL_OUTPUT" | grep -qi "Direct3D12.*NVIDIA\|NVIDIA.*Direct3D12"; then
        echo "  PASS: Vulkan sees NVIDIA GPU via DZN (D3D12)"
        echo "$ALL_OUTPUT" | grep -i "deviceName\|apiVersion" | head -5 | sed 's/^/    /'
        VULKAN_OK=true
        VULKAN_MODE="DZN (GPU via D3D12)"
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
        echo "GPU Vulkan (DZN) is recommended."
    fi
    echo ""
    echo "Proceed to full O3DE build with Dockerfile.o3de"
else
    echo "RESULT: FAIL — No Vulkan device available"
    echo "O3DE requires at least software Vulkan (llvmpipe) to render."
    exit 1
fi
