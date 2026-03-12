#!/bin/bash
# Capture rendered video from O3DE camera topic to MP4
# Requires: O3DE GameLauncher running with ROS 2 camera publishing
#
# Usage (inside container):
#   /capture_video.sh                          # 10s from auto-detected topic
#   /capture_video.sh --duration 30            # 30 seconds
#   /capture_video.sh --topic /camera/image_raw --duration 20
#   /capture_video.sh --output /root/data/my_capture.mp4
#
# Run after Phase 0 test passes:
#   docker run --gpus all --rm -v $(pwd)/data:/root/data fiber-nav-o3de:phase0 \
#     bash -c "/o3de-entrypoint.sh && /capture_video.sh --output /root/data/capture.mp4"

set -eo pipefail

DURATION=10
TOPIC=""
OUTPUT="/root/ws/src/fiber-nav-sim/o3de_capture.mp4"
FPS=30

while [[ $# -gt 0 ]]; do
    case $1 in
        --duration) DURATION="$2"; shift 2 ;;
        --topic)    TOPIC="$2"; shift 2 ;;
        --output)   OUTPUT="$2"; shift 2 ;;
        --fps)      FPS="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--duration SEC] [--topic TOPIC] [--output FILE] [--fps N]"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

source /opt/ros/jazzy/setup.bash

# Auto-detect camera topic if not specified
if [ -z "$TOPIC" ]; then
    echo "[capture] Searching for camera/image topics..."
    ALL_TOPICS=$(ros2 topic list 2>/dev/null || true)
    TOPIC=$(echo "$ALL_TOPICS" | grep -iE "image_raw|camera.*image|/image$" | head -1)
    if [ -z "$TOPIC" ]; then
        TOPIC=$(echo "$ALL_TOPICS" | grep -iE "image|camera" | head -1)
    fi
    if [ -z "$TOPIC" ]; then
        echo "[capture] ERROR: No camera/image topic found. Available topics:"
        echo "$ALL_TOPICS"
        exit 1
    fi
fi

echo "[capture] Topic:    $TOPIC"
echo "[capture] Duration: ${DURATION}s"
echo "[capture] FPS:      $FPS"
echo "[capture] Output:   $OUTPUT"
echo ""

python3 << PYEOF
import sys
import time
import os

try:
    import cv2
except ImportError:
    print("[capture] ERROR: opencv not found. Install with: pip3 install opencv-python-headless")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoCapture(Node):
    def __init__(self, topic, output, duration, fps):
        super().__init__('video_capture')
        self.bridge = CvBridge()
        self.writer = None
        self.output = output
        self.duration = duration
        self.fps = fps
        self.frame_count = 0
        self.start_time = None
        self.done = False

        self.sub = self.create_subscription(Image, topic, self.on_image, 10)
        self.get_logger().info(f'Subscribed to {topic}, capturing for {duration}s...')

    def on_image(self, msg):
        if self.done:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Frame conversion failed: {e}')
            return

        if self.writer is None:
            h, w = frame.shape[:2]
            os.makedirs(os.path.dirname(self.output) or '.', exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.output, fourcc, self.fps, (w, h))
            self.start_time = time.time()
            self.get_logger().info(f'First frame: {w}x{h}, encoding to {self.output}')

        self.writer.write(frame)
        self.frame_count += 1

        elapsed = time.time() - self.start_time
        if self.frame_count % self.fps == 0:
            print(f'  [{elapsed:.1f}s] {self.frame_count} frames captured', flush=True)

        if elapsed >= self.duration:
            self.done = True

rclpy.init()
node = VideoCapture('${TOPIC}', '${OUTPUT}', ${DURATION}, ${FPS})

try:
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    if node.writer is not None:
        node.writer.release()
    node.destroy_node()
    rclpy.shutdown()

if node.frame_count == 0:
    print('[capture] ERROR: No frames received')
    sys.exit(1)

actual_fps = node.frame_count / (time.time() - node.start_time) if node.start_time else 0
print(f'')
print(f'[capture] Done: {node.frame_count} frames, {actual_fps:.1f} actual fps')
print(f'[capture] Saved: ${OUTPUT}')
size_mb = os.path.getsize('${OUTPUT}') / (1024 * 1024)
print(f'[capture] Size:  {size_mb:.1f} MB')
PYEOF
