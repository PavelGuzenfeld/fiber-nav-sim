#!/bin/bash
# Colosseum (Cosys-AirSim) entrypoint
# Starts Xvfb + Blocks binary (as ue_user) + waits for API port + launches ROS 2
set -e

echo "[colosseum-entrypoint] Starting..."

# ── Phase 1: Xvfb (virtual display for UE rendering) ──
export DISPLAY=:99
Xvfb :99 -screen 0 1920x1080x24 +extension GLX &
XVFB_PID=$!
sleep 2

if ! kill -0 $XVFB_PID 2>/dev/null; then
    echo "[colosseum-entrypoint] ERROR: Xvfb failed to start"
    exit 1
fi
# Allow all users to access X display
xhost +local: 2>/dev/null || true
echo "[colosseum-entrypoint] Xvfb started on :99"

# ── Phase 2: Verify GPU (OpenGL + Vulkan) ──
GL_INFO=$(glxinfo 2>/dev/null | grep "OpenGL renderer" || echo "unknown")
echo "[colosseum-entrypoint] OpenGL: $GL_INFO"

VK_INFO=$(vulkaninfo --summary 2>&1 | grep -E "GPU|deviceName|driverName" | head -3 || echo "no vulkan")
echo "[colosseum-entrypoint] Vulkan: $VK_INFO"

# ── Phase 3: Start Blocks binary as non-root user ──
# UE5 binary is VulkanRHI only — requires Vulkan ICD (Mesa DZN on WSL2)
# UE5 refuses to run as root — use ue_user created in Dockerfile
BLOCKS_BIN="${BLOCKS_HOME}/Blocks.sh"

if [ ! -f "$BLOCKS_BIN" ]; then
    echo "[colosseum-entrypoint] ERROR: Blocks not found at $BLOCKS_BIN"
    exit 1
fi

# Ensure ue_user can write to /tmp for logs
chmod 1777 /tmp

echo "[colosseum-entrypoint] Starting Blocks (Cosys-AirSim) as ue_user..."
runuser -u ue_user -- env \
    DISPLAY=:99 \
    GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}" \
    MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}" \
    MESA_VK_DEVICE_SELECT="${MESA_VK_DEVICE_SELECT:-10de:2520}" \
    LD_LIBRARY_PATH="${LD_LIBRARY_PATH}" \
    HOME=/home/ue_user \
    SDL_VIDEODRIVER=x11 \
    VK_ICD_FILENAMES=/usr/local/share/vulkan/icd.d/dzn_icd.x86_64.json \
    ENABLE_DZN_COMPAT_LAYER=1 \
    "$BLOCKS_BIN" \
    -nosound \
    -nosplash \
    -novsync \
    -windowed \
    -resx=640 -resy=480 \
    -ExecCmds="sg.ShadowQuality 0,sg.TextureQuality 1,sg.EffectsQuality 0,sg.FoliageQuality 0,sg.PostProcessQuality 0,t.MaxFPS 60" \
    > /tmp/blocks.log 2>&1 &
BLOCKS_PID=$!

echo "[colosseum-entrypoint] Blocks PID: $BLOCKS_PID"

# ── Phase 4: Wait for AirSim API (port 41451) ──
echo "[colosseum-entrypoint] Waiting for AirSim API on port 41451..."
MAX_WAIT=180
for i in $(seq 1 $MAX_WAIT); do
    if timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/41451" 2>/dev/null; then
        echo "[colosseum-entrypoint] AirSim API ready after ${i}s"
        break
    fi

    if ! kill -0 $BLOCKS_PID 2>/dev/null; then
        echo "[colosseum-entrypoint] ERROR: Blocks crashed before API ready"
        echo "[colosseum-entrypoint] Last 50 lines of log:"
        tail -50 /tmp/blocks.log 2>/dev/null
        exit 1
    fi

    if [ "$i" -eq "$MAX_WAIT" ]; then
        echo "[colosseum-entrypoint] ERROR: AirSim API not ready after ${MAX_WAIT}s"
        tail -50 /tmp/blocks.log 2>/dev/null
        exit 1
    fi

    sleep 1
done

# ── Phase 5: Source ROS 2 ──
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null || true

echo "[colosseum-entrypoint] Environment ready"
echo "[colosseum-entrypoint]   Blocks PID: $BLOCKS_PID"
echo "[colosseum-entrypoint]   AirSim API: localhost:41451"
echo "[colosseum-entrypoint]   Display: $DISPLAY"

# ── Phase 6: Start PX4 SITL in HIL mode (if enabled) ──
if [ "${PX4_HIL:-0}" = "1" ]; then
    echo "[colosseum-entrypoint] Starting PX4 SITL in HIL mode..."

    # Copy custom airframe if present
    AIRFRAME_SRC="${WORKSPACE}/src/fiber-nav-sim/docker/airframes/4251_gz_quadtailsitter_vision"
    AIRFRAME_DST="/root/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4251_gz_quadtailsitter_vision"
    if [ -f "$AIRFRAME_SRC" ]; then
        cp "$AIRFRAME_SRC" "$AIRFRAME_DST"
    fi

    # Start PX4 SITL — it listens on TCP:4560 for HIL simulator connections
    cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
    rm -f dataman parameters*.bson
    PX4_SYS_AUTOSTART=4251 \
    HIL_ENABLED=1 \
    PX4_SIMULATOR=1 \
    ../bin/px4 > /dev/null 2>&1 &
    PX4_PID=$!
    echo "[colosseum-entrypoint] PX4 SITL PID: $PX4_PID (HIL mode, TCP:4560)"
    sleep 3

    # Start MicroXRCE-DDS agent for ROS 2 communication
    MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
    echo "[colosseum-entrypoint] MicroXRCE-DDS agent started on UDP:8888"
fi

# ── Phase 7: Execute command or launch ROS 2 ──
if [ "$#" -gt 0 ]; then
    exec "$@"
else
    echo "[colosseum-entrypoint] No command specified, starting interactive shell"
    exec bash
fi
