#!/bin/bash
# Copyright 2026 Pavel Guzenfeld — All rights reserved.
# PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
#
# SITL Mission Test Runner
# Starts the PX4+Gazebo stack, flies a mission, validates pass/fail from the log.
#
# Usage:
#   MISSION=vtol_gps_denied ./scripts/test_sitl_mission.sh [--timeout MINUTES]
#
# Exit codes:
#   0 = PASS   (all criteria met)
#   1 = FAIL   (one or more criteria failed)
#   2 = TIMEOUT (mission did not complete within time limit)
#   3 = INFRASTRUCTURE (container failed to start or crashed)

set -euo pipefail

# --- Defaults ---
TIMEOUT_MINUTES=60
COMPOSE_FILE="docker/docker-compose.yml"
SERVICE="px4-sitl"
CONTAINER="fiber-nav-px4-sitl"
LOG_REL="logs/vtol_mission.log"
POLL_INTERVAL=10

# --- Parse args ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout) TIMEOUT_MINUTES="$2"; shift 2 ;;
        --compose-file) COMPOSE_FILE="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 3 ;;
    esac
done

# --- Colors ---
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# --- Resolve paths ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_FILE="$REPO_ROOT/$LOG_REL"

echo -e "${CYAN}${BOLD}=== SITL Mission Test ===${NC}"
echo -e "  Mission:   ${CYAN}${MISSION:-vtol_gps_denied}${NC}"
echo -e "  Timeout:   ${TIMEOUT_MINUTES} min"
echo -e "  Log:       ${LOG_FILE}"
echo ""

# --- Cleanup trap ---
cleanup() {
    echo ""
    echo -e "${YELLOW}[CLEANUP]${NC} Stopping container..."
    cd "$REPO_ROOT"
    docker compose -f "$COMPOSE_FILE" stop "$SERVICE" 2>/dev/null || true
    docker compose -f "$COMPOSE_FILE" rm -f "$SERVICE" 2>/dev/null || true
    echo -e "${YELLOW}[CLEANUP]${NC} Done."
}
trap cleanup EXIT SIGINT SIGTERM

# --- Stop any existing container (ensures fresh start) ---
echo -e "${GREEN}[START]${NC} Stopping existing container (if any)..."
cd "$REPO_ROOT"
docker compose -f "$COMPOSE_FILE" stop "$SERVICE" 2>/dev/null || true
docker compose -f "$COMPOSE_FILE" rm -f "$SERVICE" 2>/dev/null || true

# --- Clear old log ---
mkdir -p "$(dirname "$LOG_FILE")"
rm -f "$LOG_FILE"
touch "$LOG_FILE"

# --- Start container ---
echo -e "${GREEN}[START]${NC} Launching ${SERVICE} via docker compose..."
MISSION="${MISSION:-vtol_gps_denied}" \
    docker compose -f "$COMPOSE_FILE" up -d "$SERVICE"

# Wait for container to be running
for i in $(seq 1 30); do
    STATE=$(docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null || echo "missing")
    if [ "$STATE" = "running" ]; then
        echo -e "${GREEN}[START]${NC} Container running."
        break
    fi
    if [ "$i" -eq 30 ]; then
        echo -e "${RED}[INFRA]${NC} Container did not start within 30s (state: $STATE)"
        exit 3
    fi
    sleep 1
done

# --- Poll loop ---
echo -e "${GREEN}[POLL]${NC} Waiting for mission to complete (polling every ${POLL_INTERVAL}s)..."
DEADLINE=$((SECONDS + TIMEOUT_MINUTES * 60))

while [ $SECONDS -lt $DEADLINE ]; do
    # Check container health
    STATE=$(docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null || echo "missing")
    if [ "$STATE" != "running" ]; then
        echo -e "${RED}[INFRA]${NC} Container died (state: $STATE)"
        if [ -f "$LOG_FILE" ] && [ -s "$LOG_FILE" ]; then
            echo -e "${YELLOW}[LOG TAIL]${NC}"
            tail -20 "$LOG_FILE"
        fi
        exit 3
    fi

    # Check for early failure markers
    if [ -f "$LOG_FILE" ] && [ -s "$LOG_FILE" ]; then
        if grep -qiE 'FATAL|aborting mission' "$LOG_FILE" 2>/dev/null; then
            echo -e "${RED}[EARLY FAIL]${NC} Detected failure marker in log."
            tail -10 "$LOG_FILE"
            break
        fi

        # Check for mission completion
        if grep -q 'VTOL mission complete' "$LOG_FILE" 2>/dev/null; then
            echo -e "${GREEN}[DONE]${NC} Mission completed."
            break
        fi
    fi

    # Progress indicator
    WP_COUNT=$(grep -c 'WP.*accepted' "$LOG_FILE" 2>/dev/null || true)
    ELAPSED=$(( (SECONDS) / 60 ))
    REMAINING=$(( (DEADLINE - SECONDS) / 60 ))
    echo -ne "\r  ${CYAN}[${ELAPSED}m elapsed, ${REMAINING}m remaining]${NC} WPs accepted: ${WP_COUNT}  "

    sleep "$POLL_INTERVAL"
done
echo ""

# --- Timeout check ---
if [ $SECONDS -ge $DEADLINE ]; then
    echo -e "${RED}[TIMEOUT]${NC} Mission did not complete within ${TIMEOUT_MINUTES} minutes."
    if [ -f "$LOG_FILE" ] && [ -s "$LOG_FILE" ]; then
        echo -e "${YELLOW}[LOG TAIL]${NC}"
        tail -30 "$LOG_FILE"
    fi
    exit 2
fi

# --- Extract metrics ---
echo ""
echo -e "${BOLD}=== Metrics ===${NC}"

WP_COUNT=$(grep -c 'WP.*accepted' "$LOG_FILE" 2>/dev/null || true)
RETURN_DIST=$(grep -oP 'Return complete.*dist_home=\K[0-9.]+' "$LOG_FILE" 2>/dev/null | tail -1)
RETURN_DIST="${RETURN_DIST:-N/A}"
FINAL_DIST=$(grep -oP 'Over home.*d=\K[0-9.]+' "$LOG_FILE" 2>/dev/null | tail -1)
FINAL_DIST="${FINAL_DIST:-N/A}"
ABORT_COUNT=$(grep -c 'ABORT' "$LOG_FILE" 2>/dev/null || true)
GPS_DENIED=$(grep -c 'GPS-denied nav active' "$LOG_FILE" 2>/dev/null || true)
MISSION_COMPLETE=$(grep -c 'VTOL mission complete' "$LOG_FILE" 2>/dev/null || true)
STATE_TRANSITIONS=$(grep -cE 'State: \w+ -> \w+' "$LOG_FILE" 2>/dev/null || true)

echo -e "  WP accepted:        ${WP_COUNT}"
echo -e "  Return distance:    ${RETURN_DIST} m"
echo -e "  Final distance:     ${FINAL_DIST} m"
echo -e "  Abort count:        ${ABORT_COUNT}"
echo -e "  GPS-denied active:  $([ "$GPS_DENIED" -gt 0 ] && echo 'yes' || echo 'no')"
echo -e "  State transitions:  ${STATE_TRANSITIONS}"
echo -e "  Mission complete:   $([ "$MISSION_COMPLETE" -gt 0 ] && echo 'yes' || echo 'no')"

# --- Evaluate criteria ---
echo ""
echo -e "${BOLD}=== Pass/Fail Criteria ===${NC}"

PASS=true

check() {
    local num="$1"
    local name="$2"
    local ok="$3"
    if [ "$ok" = "true" ]; then
        echo -e "  ${GREEN}[PASS]${NC} #${num} ${name}"
    else
        echo -e "  ${RED}[FAIL]${NC} #${num} ${name}"
        PASS=false
    fi
}

# 1. Mission completed
check 1 "Mission completed" "$([ "$MISSION_COMPLETE" -gt 0 ] && echo true || echo false)"

# 2. All WPs accepted (>= 8)
check 2 "All WPs accepted (${WP_COUNT} >= 8)" "$([ "$WP_COUNT" -ge 8 ] && echo true || echo false)"

# 3. Return distance <= 200m
if [ "$RETURN_DIST" = "N/A" ]; then
    check 3 "Return distance (<= 200m): N/A" "false"
else
    RET_OK=$(awk "BEGIN {print ($RETURN_DIST <= 200) ? \"true\" : \"false\"}")
    check 3 "Return distance (${RETURN_DIST}m <= 200m)" "$RET_OK"
fi

# 4. Final distance <= 10m
if [ "$FINAL_DIST" = "N/A" ]; then
    check 4 "Final distance (<= 10m): N/A" "false"
else
    FIN_OK=$(awk "BEGIN {print ($FINAL_DIST <= 10) ? \"true\" : \"false\"}")
    check 4 "Final distance (${FINAL_DIST}m <= 10m)" "$FIN_OK"
fi

# 5. No aborts
check 5 "No aborts (${ABORT_COUNT} == 0)" "$([ "$ABORT_COUNT" -eq 0 ] && echo true || echo false)"

# 6. GPS-denied active
check 6 "GPS-denied nav active" "$([ "$GPS_DENIED" -gt 0 ] && echo true || echo false)"

# --- Result ---
echo ""
if [ "$PASS" = "true" ]; then
    echo -e "${GREEN}${BOLD}=== ALL CRITERIA PASSED ===${NC}"
    exit 0
else
    echo -e "${RED}${BOLD}=== SOME CRITERIA FAILED ===${NC}"
    exit 1
fi
