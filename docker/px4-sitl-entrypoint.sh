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
#   1. Gazebo + sensors + Foxglove  (via simulation.launch.py)
#   2. MicroXRCE-DDS Agent          (PX4 <-> ROS 2 bridge)
#   3. sim_distance_sensor.py       (terrain height for PX4)
#   4. terrain_gis_node.py          (terrain GIS height queries)
#   5. PX4 SITL                     (autopilot)
#   6. map_bridge_node.py           (NavSatFix + terrain GeoJSON for Foxglove Map)
#   7. Mission auto-launch (optional, if MISSION is set)
#   8. Ready
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
MISSION="${MISSION:-}"                    # Mission to auto-launch: vtol_terrain, vtol_canyon, or empty (manual)
MISSION_CONFIG="${MISSION_CONFIG:-}"      # Override config file path (optional)
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

# ============================================================
# Phase 0: Rebuild workspace (source is volume-mounted, may be newer than image)
# ============================================================
phase "0/9" "Rebuilding workspace (symlink-install)..."
cd "$WS"
colcon build --symlink-install \
    --packages-skip px4_msgs px4_ros2_cpp px4_ros2_py \
    --packages-skip-regex 'example_.*' \
    --cmake-args -DCMAKE_CXX_STANDARD=23 \
    2>&1 | tail -5
phase "0/9" "Workspace built"
cd /

# ============================================================
# Phase 1: Gazebo + sensors + Foxglove
# ============================================================
phase "1/9" "Starting Gazebo + sensors + Foxglove..."
phase "1/9" "  headless=$HEADLESS foxglove=$FOXGLOVE world=$WORLD"

ros2 launch fiber_nav_bringup simulation.launch.py \
    backend:=gazebo \
    use_px4:=true \
    headless:="$HEADLESS" \
    foxglove:="$FOXGLOVE" \
    world:="$WORLD" \
    world_name:="$WORLD_NAME" &
PIDS+=($!)

phase "1/9" "Waiting for odometry topic (Gazebo + model spawn + bridge)..."
if wait_for_topic "/model/quadtailsitter/odometry" 120; then
    phase "1/9" "Gazebo ready — odometry publishing"
else
    fail "Gazebo did not produce /model/quadtailsitter/odometry within 120s"
fi

# ============================================================
# Phase 2: MicroXRCE-DDS Agent
# ============================================================
phase "2/9" "Starting MicroXRCE-DDS Agent on UDP:8888..."
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
PIDS+=($!)
sleep 1
check_alive "${PIDS[-1]}" "MicroXRCE-DDS Agent"
phase "2/9" "DDS Agent running (PID ${PIDS[-1]})"

# ============================================================
# Phase 3: Simulated distance sensor
# ============================================================
phase "3/9" "Starting sim_distance_sensor..."
python3 "${SRC}/scripts/sim_distance_sensor.py" > /dev/null 2>&1 &
PIDS+=($!)
sleep 2
check_alive "${PIDS[-1]}" "sim_distance_sensor"
phase "3/9" "Distance sensor running (PID ${PIDS[-1]})"

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
phase "5/9" "Starting PX4 SITL (airframe 4251, quadtailsitter)..."

cd "${PX4_DIR}/rootfs"
rm -f dataman parameters*.bson

PX4_SYS_AUTOSTART=4251 \
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
    -p world_name:="${WORLD_NAME}" \
    -p model_name:=quadtailsitter \
    > /dev/null 2>&1 &
PIDS+=($!)
sleep 2
check_alive "${PIDS[-1]}" "cable_dynamics_node"
phase "7/9" "Cable dynamics running (PID ${PIDS[-1]})"

# ============================================================
# Phase 8: Auto-launch mission (optional)
# ============================================================
if [ -n "$MISSION" ]; then
    phase "8/9" "Auto-launching mission: $MISSION"

    # Determine config file
    case "$MISSION" in
        vtol_terrain)
            CONFIG="${MISSION_CONFIG:-${SRC}/src/fiber_nav_mode/config/terrain_mission.yaml}"
            ;;
        vtol_canyon)
            CONFIG="${MISSION_CONFIG:-${SRC}/src/fiber_nav_mode/config/canyon_mission.yaml}"
            ;;
        *)
            fail "Unknown mission: $MISSION (expected: vtol_terrain, vtol_canyon)"
            ;;
    esac

    # Wait for PX4 to finish parameter loading (avoids arm rejection)
    sleep 5

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
    log "       --params-file /root/ws/src/fiber-nav-sim/src/fiber_nav_mode/config/terrain_mission.yaml'"
fi
echo ""

# Wait for any child process to exit
wait -n 2>/dev/null || true
warn "A child process exited. Shutting down..."
