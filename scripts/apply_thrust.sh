#!/bin/bash
# Apply one-time wrench to quadtailsitter model for testing
# Usage: ./scripts/apply_thrust.sh [force_x] [duration]
#
# Note: For sustained flight, use the stabilized_flight_controller instead.
# This script is for quick manual testing of wrench application.

FORCE_X=${1:-10.0}
DURATION=${2:-5}

echo "Applying thrust: ${FORCE_X}N forward for ${DURATION}s"

# Use Gazebo service to apply wrench
gz service -s /world/canyon_world/wrench \
    --reqtype gz.msgs.EntityWrench \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "entity: { name: \"quadtailsitter\", type: MODEL } wrench: { force: { x: ${FORCE_X} } }"

echo "Thrust applied."
