#!/bin/bash
# Copyright 2026 Pavel Guzenfeld — All rights reserved.
# PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
# Version: 0.0.1
#
# PX4 SITL Orchestrator
# Starts all simulation services in order with health checks.
# Used as docker-compose command for the px4-sitl service.
#
# Phases:
#   1. MicroXRCE-DDS Agent          (PX4 <-> ROS 2 bridge)
#   2. Gazebo + sensors + Foxglove  (via simulation.launch.py, includes TERCOM + position EKF)
#   3. sim_distance_sensor.py       (terrain height for PX4)
#   4. terrain_gis_node.py          (terrain GIS height queries)
#   5. PX4 SITL                     (autopilot)
#   6. map_bridge_node.py           (NavSatFix + terrain GeoJSON for Foxglove Map)
#   7. cable_dynamics_node          (virtual fiber force model)
#   8. Mission auto-launch (optional, if MISSION is set)
#   9. Ready
#
# Environment variables (set via docker-compose):
#   HEADLESS       - Run Gazebo without GUI (default: true)
#   FOXGLOVE       - Enable Foxglove bridge (default: true)
#   WORLD          - World SDF file name (default: canyon_harmonic)
#   WORLD_NAME     - Gazebo internal world name (default: canyon_world)
#   MISSION        - Auto-launch mission: vtol_terrain, vtol_canyon, or empty (manual)
#   MISSION_CONFIG - Override config file path (optional)

set -euo pipefail

# --- Configuration ---
HEADLESS="${HEADLESS:-true}"
FOXGLOVE="${FOXGLOVE:-true}"
WORLD="${WORLD:-canyon_harmonic}"
WORLD_NAME="${WORLD_NAME:-canyon_world}"
MISSION="vtol_gps_denied"                # FORCED: GPS-denied testing. Revert to "${MISSION:-}" for env-based selection.
MISSION_CONFIG="${MISSION_CONFIG:-}"      # Override config file path (optional)
AIRFRAME="${AIRFRAME:-4251}"             # PX4 airframe: 4251 (GPS), 4252 (GPS-denied)
SRC="/root/ws/src/fiber-nav-sim"
mkdir -p "${SRC}/logs"
PX4_DIR="/root/PX4-Autopilot/build/px4_sitl_default"
WS="/root/ws"

# --- Colors ---
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# --- Logging ---
log()  { echo -e "${GREEN}[ORCH]${NC} $1"; }
warn() { echo -e "${YELLOW}[ORCH]${NC} $1"; }
fail() { echo -e "${RED}[ORCH FAIL]${NC} $1"; exit 1; }
phase() { echo -e "${CYAN}[$1]${NC} $2"; }

# --- Resolve mission config early (needed for phase 2 launch) ---
RESOLVED_MISSION_CONFIG=""
if [ -n "$MISSION" ]; then
    case "$MISSION" in
        vtol_gps_denied)
            RESOLVED_MISSION_CONFIG="${MISSION_CONFIG:-${SRC}/src/fiber_nav_mode/config/gps_denied_mission.yaml}"
            ;;
        vtol_canyon)
            RESOLVED_MISSION_CONFIG="${MISSION_CONFIG:-${SRC}/src/fiber_nav_mode/config/canyon_mission.yaml}"
            ;;
        *)
            fail "Unknown mission: $MISSION (expected: vtol_gps_denied, vtol_canyon)"
            ;;
    esac
    log "Mission config resolved: $RESOLVED_MISSION_CONFIG"
fi

# --- Process tracking ---
PIDS=()

cleanup() {
    echo ""
    log "Shutting down all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    # Give processes time to exit gracefully
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    log "All processes stopped."
}
trap cleanup EXIT SIGINT SIGTERM

# --- Health check: wait for ROS 2 topic to publish ---
# Usage: wait_for_topic TOPIC TIMEOUT_SECONDS
# Returns 0 when the topic is publishing (any rate > 0)
wait_for_topic() {
    local topic="$1"
    local timeout_sec="$2"
    local deadline=$((SECONDS + timeout_sec))

    while [ $SECONDS -lt $deadline ]; do
        # Try to get one message with a short timeout
        if timeout 5 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
            return 0
        fi
        sleep 2
    done
    return 1
}

# --- Health check: verify process is alive ---
check_alive() {
    local pid="$1"
    local name="$2"
    kill -0 "$pid" 2>/dev/null || fail "$name (PID $pid) exited unexpectedly"
}

# --- Source ROS + workspace (disable -u for setup scripts that use unbound vars) ---
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi
set -u

# ============================================================
# Phase 0: Rebuild workspace (source is volume-mounted, may be newer than image)
# ============================================================
phase "0/9" "Rebuilding workspace (symlink-install)..."
cd "$WS"

# Patch px4-ros2-interface-lib: disable watchdog shutdown entirely.
# The HealthAndArmingChecks watchdog kills our node when PX4 doesn't send
# health check requests quickly enough. During VTOL transitions under
# Gazebo CPU load, PX4 can go >30s without sending check requests.
# Instead of increasing the timer, disable _shutdown_on_timeout completely.
HEALTH_HPP="$WS/src/px4-ros2-interface-lib/px4_ros2_cpp/include/px4_ros2/components/health_and_arming_checks.hpp"
if grep -q '_shutdown_on_timeout{true}' "$HEALTH_HPP" 2>/dev/null; then
    sed -i 's/_shutdown_on_timeout{true}/_shutdown_on_timeout{false}/' "$HEALTH_HPP"
    log "Patched: disabled watchdog shutdown on timeout"
    colcon build --symlink-install --packages-select px4_ros2_cpp \
        --cmake-args -DCMAKE_CXX_STANDARD=23 -DBUILD_TESTING=OFF \
        2>&1 | tail -3
fi

# Patch px4-ros2-interface-lib: widen VTOL state timeout from 2s to 10s.
# vtol_vehicle_status is published at ~1Hz BEST_EFFORT — messages often drop
# under Gazebo CPU load. With the default 2s timeout, toFixedwing()/toMulticopter()
# fail because _last_vtol_vehicle_status_received appears stale.
VTOL_CPP="$WS/src/px4-ros2-interface-lib/px4_ros2_cpp/src/control/vtol.cpp"
NEEDS_VTOL_REBUILD=false
if grep -q '< 2s)' "$VTOL_CPP" 2>/dev/null; then
    sed -i 's/< 2s)/< 10s)/g' "$VTOL_CPP"
    log "Patched: vtol.cpp timeout 2s -> 10s"
    NEEDS_VTOL_REBUILD=true
fi
if [ "$NEEDS_VTOL_REBUILD" = true ]; then
    colcon build --symlink-install --packages-select px4_ros2_cpp \
        --cmake-args -DCMAKE_CXX_STANDARD=23 -DBUILD_TESTING=OFF \
        2>&1 | tail -3
fi

# Force clean rebuild of volume-mounted packages to pick up HPP/CPP changes.
# The symlink-install + cached build dirs from the Docker image can mask
# source changes in volume-mounted code. Delete build/install to force recompile.
rm -rf "$WS/build/fiber_nav_mode" "$WS/install/fiber_nav_mode"
rm -rf "$WS/build/fiber_nav_fusion" "$WS/install/fiber_nav_fusion"
rm -rf "$WS/build/fiber_nav_sensors" "$WS/install/fiber_nav_sensors"

# Incremental build for volume-mounted packages (symlink-install + no build dir
# removal = faster restarts). fiber_nav_mode always rebuilds fresh (cleaned above).
MAKEFLAGS='-j1' colcon build --symlink-install \
    --packages-skip px4_msgs px4_ros2_cpp px4_ros2_py \
    --packages-skip-regex 'example_.*' \
    --cmake-args -DCMAKE_CXX_STANDARD=23 -DBUILD_TESTING=OFF \
    2>&1 | tail -5
phase "0/9" "Workspace built"
set +u; source "$WS/install/setup.bash"; set -u
cd /

# ============================================================
# Phase 1: MicroXRCE-DDS Agent (start BEFORE Gazebo to claim port 8888)
# ============================================================
phase "1/9" "Starting MicroXRCE-DDS Agent on UDP:8888..."
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
PIDS+=($!)
sleep 1
check_alive "${PIDS[-1]}" "MicroXRCE-DDS Agent"
phase "1/9" "DDS Agent running (PID ${PIDS[-1]})"

# ============================================================
# Phase 2: Gazebo + sensors + Foxglove
# ============================================================
phase "2/9" "Starting Gazebo + sensors + Foxglove..."
phase "2/9" "  headless=$HEADLESS foxglove=$FOXGLOVE world=$WORLD"

ros2 launch fiber_nav_bringup simulation.launch.py \
    backend:=gazebo \
    use_px4:=true \
    headless:="$HEADLESS" \
    foxglove:="$FOXGLOVE" \
    world:="$WORLD" \
    world_name:="$WORLD_NAME" \
    mission_config:="$RESOLVED_MISSION_CONFIG" &
PIDS+=($!)

phase "2/9" "Waiting for odometry topic (Gazebo + model spawn + bridge)..."
if wait_for_topic "/model/quadtailsitter/odometry" 120; then
    phase "2/9" "Gazebo ready — odometry publishing"
else
    fail "Gazebo did not produce /model/quadtailsitter/odometry within 120s"
fi

# ============================================================
# Phase 3: Simulated distance sensor
# ============================================================
phase "3/9" "Starting sim_distance_sensor..."
python3 "${SRC}/scripts/sim_distance_sensor.py" > /dev/null 2>&1 &
PIDS+=($!)
sleep 2
check_alive "${PIDS[-1]}" "sim_distance_sensor"
phase "3/9" "Distance sensor running (PID ${PIDS[-1]})"

# Start simulated vision odometry (Gazebo GT → PX4 VehicleOdometry).
# Provides velocity feedback for EKF when GPS is disabled.
# In the real system, fiber_vision_fusion replaces this.
python3 "${SRC}/scripts/sim_vision_odometry.py" > "${SRC}/logs/sim_vision_odom.log" 2>&1 &
PIDS+=($!)
sleep 1
check_alive "${PIDS[-1]}" "sim_vision_odometry"
phase "3/9" "Vision odometry running (PID ${PIDS[-1]})"

# ============================================================
# Phase 4: Terrain GIS node
# ============================================================
phase "4/9" "Starting terrain_gis_node..."
python3 "${SRC}/scripts/terrain_gis_node.py" > /dev/null 2>&1 &
PIDS+=($!)
sleep 2
check_alive "${PIDS[-1]}" "terrain_gis_node"
phase "4/9" "Terrain GIS node running (PID ${PIDS[-1]})"

# ============================================================
# Phase 5: PX4 SITL
# ============================================================
phase "5/9" "Starting PX4 SITL (airframe ${AIRFRAME}, quadtailsitter)..."

# Deploy latest airframe from volume-mounted source (may be newer than Docker image)
AIRFRAME_SRC="${SRC}/docker/airframes/${AIRFRAME}_gz_quadtailsitter_vision"
AIRFRAME_DST="${PX4_DIR}/etc/init.d-posix/airframes/${AIRFRAME}_gz_quadtailsitter_vision"
if [ -f "$AIRFRAME_SRC" ]; then
    cp "$AIRFRAME_SRC" "$AIRFRAME_DST"
    chmod +x "$AIRFRAME_DST"
    phase "5/9" "Airframe deployed from source"
fi

cd "${PX4_DIR}/rootfs"
rm -f dataman parameters*.bson

PX4_SYS_AUTOSTART="${AIRFRAME}" \
PX4_GZ_MODEL_NAME=quadtailsitter \
PX4_GZ_WORLD="${WORLD_NAME}" \
    "${PX4_DIR}/rootfs/../bin/px4" > /dev/null 2>&1 &
PIDS+=($!)

phase "5/9" "Waiting for PX4 vehicle status topic..."
if wait_for_topic "/fmu/out/vehicle_status_v1" 90; then
    phase "5/9" "PX4 SITL connected — vehicle status publishing"
else
    fail "PX4 did not publish /fmu/out/vehicle_status_v1 within 60s"
fi

# ============================================================
# Phase 6: Map bridge (NavSatFix + terrain GeoJSON for Foxglove Map)
# ============================================================
phase "6/9" "Starting map_bridge_node..."
python3 "${SRC}/scripts/map_bridge_node.py" > /dev/null 2>&1 &
PIDS+=($!)
sleep 2
check_alive "${PIDS[-1]}" "map_bridge_node"
phase "6/9" "Map bridge running (PID ${PIDS[-1]})"

# ============================================================
# Phase 7: Cable dynamics (virtual fiber force model)
# ============================================================
phase "7/9" "Starting cable_dynamics_node..."
ros2 run fiber_nav_sensors cable_dynamics_node \
    --ros-args \
    --params-file /root/ws/install/fiber_nav_bringup/share/fiber_nav_bringup/config/sensor_params.yaml \
    -p world_name:="${WORLD_NAME}" \
    -p model_name:=quadtailsitter \
    > /dev/null 2>&1 &
PIDS+=($!)
sleep 2
check_alive "${PIDS[-1]}" "cable_dynamics_node"
phase "7/9" "Cable dynamics running (PID ${PIDS[-1]})"

# ============================================================
# Phase 8: Auto-launch mission (optional)
# Note: TERCOM, position EKF, spool sim, and optical flow are already
# launched by simulation.launch.py → gazebo_simulation.launch.py → sensors.launch.py
# ============================================================
if [ -n "$MISSION" ]; then
    phase "8/9" "Auto-launching mission: $MISSION"

    # Config already resolved before phase 2 (for position_ekf_node overlay)
    CONFIG="$RESOLVED_MISSION_CONFIG"
    phase "8/9" "Config: $CONFIG"

    # Wait for PX4 to finish parameter loading and DDS initialization.
    # PX4's XRCE-DDS client needs time to register all topics after startup.
    # Too short → vtol_navigation_node's mode registration times out.
    sleep 15

    ros2 run fiber_nav_mode vtol_navigation_node \
        --ros-args --params-file "$CONFIG" \
        > "${SRC}/logs/vtol_mission.log" 2>&1 &
    PIDS+=($!)
    sleep 3
    check_alive "${PIDS[-1]}" "vtol_navigation_node"
    phase "8/9" "Mission node running (PID ${PIDS[-1]}), logs: logs/vtol_mission.log"
else
    phase "8/9" "No MISSION set — manual flight mode"
fi

# ============================================================
# Phase 9: Ready
# ============================================================
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  PX4 SITL READY${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "  Foxglove:   ${CYAN}ws://localhost:8765${NC}"
echo -e "  MAVLink:    ${CYAN}localhost:14550${NC}"
echo -e "  Processes:  ${#PIDS[@]} running"
if [ -n "$MISSION" ]; then
    echo -e "  Mission:    ${CYAN}${MISSION}${NC} (auto-launched)"
    echo -e "  Logs:       ${CYAN}logs/vtol_mission.log${NC}"
fi
echo -e "${GREEN}========================================${NC}"
echo ""
if [ -z "$MISSION" ]; then
    log "To run a VTOL mission manually:"
    log "  docker exec fiber-nav-px4-sitl bash -c \\"
    log "    'source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash && \\"
    log "     ros2 run fiber_nav_mode vtol_navigation_node --ros-args \\"
    log "       --params-file /root/ws/src/fiber-nav-sim/src/fiber_nav_mode/config/gps_denied_mission.yaml'"
fi
echo ""

# Wait for any child process to exit
wait -n 2>/dev/null || true
EXIT_CODE=$?
warn "A child process exited (code=$EXIT_CODE). Checking PIDs..."
# Log which processes are dead
for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null
        rc=$?
        warn "  PID $pid (index $i) is dead (exit=$rc)"
    fi
done
# Write to host-readable log
{
    echo "=== Process death at $(date) ==="
    echo "wait -n exit code: $EXIT_CODE"
    for i in "${!PIDS[@]}"; do
        pid="${PIDS[$i]}"
        if kill -0 "$pid" 2>/dev/null; then
            echo "  PID $pid (index $i): ALIVE"
        else
            echo "  PID $pid (index $i): DEAD"
        fi
    done
} >> "${SRC}/logs/entrypoint_crash.log"
warn "Shutting down..."
