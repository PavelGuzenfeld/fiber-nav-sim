#!/bin/bash
# Canyon wrap-around script - teleports plane back to start when reaching end
# Creates illusion of infinite canyon

CANYON_LENGTH=2400  # meters (canyon is 2500m, wrap before end)
RESET_X=0
RESET_Z=50

echo "Canyon wrap-around active (length: ${CANYON_LENGTH}m)"
echo "Monitoring plane position..."

while true; do
    # Get current X position
    POS=$(gz topic -e -t /model/plane/odometry -n 1 2>/dev/null | grep -A1 "x:" | head -1 | awk '{print $2}')

    if [ -n "$POS" ]; then
        # Convert to integer for comparison
        POS_INT=${POS%.*}

        if [ "$POS_INT" -gt "$CANYON_LENGTH" ] 2>/dev/null; then
            echo "Plane at ${POS_INT}m - wrapping to start..."
            gz service -s /world/canyon_world/set_pose \
                --reqtype gz.msgs.Pose \
                --reptype gz.msgs.Boolean \
                --req "name: \"plane\", position: {x: ${RESET_X}, y: 0, z: ${RESET_Z}}" \
                --timeout 1000 >/dev/null 2>&1
            echo "Reset complete"
        fi
    fi

    sleep 0.5
done
