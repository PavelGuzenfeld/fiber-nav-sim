#!/usr/bin/env python3
"""Canyon wrap-around - teleports plane back when reaching canyon end."""
import subprocess
import time
import re

CANYON_END = 2200  # meters
RESET_POS = "name: \"plane\", position: {x: 0, y: 0, z: 50}"

def get_position():
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-t", "/model/plane/odometry", "-n", "1"],
            capture_output=True, text=True, timeout=2
        )
        match = re.search(r'x:\s*([-\d.]+)', result.stdout)
        if match:
            return float(match.group(1))
    except:
        pass
    return None

def reset_plane():
    subprocess.run([
        "gz", "service", "-s", "/world/canyon_world/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--req", RESET_POS, "--timeout", "1000"
    ], capture_output=True)

print(f"Wrap-around active: reset at {CANYON_END}m")
while True:
    pos = get_position()
    if pos is not None and pos > CANYON_END:
        print(f"Position {pos:.0f}m > {CANYON_END}m - resetting...")
        reset_plane()
    time.sleep(0.3)
