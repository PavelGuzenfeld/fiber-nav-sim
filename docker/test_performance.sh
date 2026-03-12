#!/bin/bash
# O3DE Phase 0 Performance Test: Rendering Latency & Throughput
# Measures GPU rendering performance in headless Docker container.
#
# Tests:
# 1. Vulkan render frame latency (via vulkaninfo + timestamps)
# 2. ROS 2 topic throughput (messages/sec on sensor topics)
# 3. GameLauncher frame timing (from O3DE logs)
#
# Usage:
#   docker run --gpus all --rm fiber-nav-o3de:phase0 /test_performance.sh
#   docker run --gpus all --rm fiber-nav-o3de:phase0 /test_performance.sh --duration 30

set -eo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

DURATION=15
PROJECT_PATH="${PROJECT_PATH:-/opt/HeadlessTest}"
PROJECT_NAME="${PROJECT_NAME:-HeadlessTest}"

while [[ $# -gt 0 ]]; do
    case $1 in
        --duration) DURATION="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--duration SEC]"
            exit 0 ;;
        *) shift ;;
    esac
done

info()  { echo -e "${YELLOW}[INFO]${NC} $1"; }
perf()  { echo -e "${CYAN}[PERF]${NC} $1"; }
pass()  { echo -e "${GREEN}[PASS]${NC} $1"; }
fail()  { echo -e "${RED}[FAIL]${NC} $1"; }

cleanup() {
    info "Cleaning up..."
    pkill -f "${PROJECT_NAME}.GameLauncher" 2>/dev/null || true
    pkill -f "Xvfb :99" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT

echo "============================================="
echo " O3DE Phase 0: Performance Test"
echo " Duration: ${DURATION}s per measurement"
echo "============================================="
echo ""

source /opt/ros/jazzy/setup.bash

# ─── Test 1: Vulkan GPU Info & Baseline ───
info "Test 1: Vulkan GPU baseline..."

VULKAN_OUTPUT=$(vulkaninfo --summary 2>&1 || true)
GPU_NAME=$(echo "$VULKAN_OUTPUT" | grep "deviceName" | head -1 | sed 's/.*= //')
DRIVER_VER=$(echo "$VULKAN_OUTPUT" | grep "driverVersion" | head -1 | sed 's/.*= //')
API_VER=$(echo "$VULKAN_OUTPUT" | grep "apiVersion" | head -1 | sed 's/.*= //')

if [ -z "$GPU_NAME" ]; then
    fail "No Vulkan GPU detected"
    exit 1
fi

perf "GPU: $GPU_NAME"
perf "Driver: $DRIVER_VER"
perf "Vulkan API: $API_VER"

# GPU memory info via nvidia-smi
if command -v nvidia-smi &> /dev/null; then
    GPU_MEM_TOTAL=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1)
    GPU_MEM_FREE=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits 2>/dev/null | head -1)
    GPU_TEMP=$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits 2>/dev/null | head -1)
    GPU_POWER=$(nvidia-smi --query-gpu=power.draw --format=csv,noheader,nounits 2>/dev/null | head -1)
    perf "VRAM: ${GPU_MEM_FREE}/${GPU_MEM_TOTAL} MiB free"
    perf "Temperature: ${GPU_TEMP}C"
    perf "Power: ${GPU_POWER}W"
fi
echo ""

# ─── Start GameLauncher ───
info "Starting O3DE GameLauncher..."
LAUNCHER="${PROJECT_PATH}/build/linux/bin/profile/${PROJECT_NAME}.GameLauncher"

if [ ! -f "$LAUNCHER" ]; then
    fail "GameLauncher not found at $LAUNCHER"
    exit 1
fi

$LAUNCHER -console-mode \
    --regset="/O3DE/Atom/Bootstrap/CreateNativeWindow=false" \
    > /tmp/o3de_launcher.log 2>&1 &
LAUNCHER_PID=$!

info "Waiting for initialization (up to 60s)..."
STARTED=false
for i in $(seq 1 60); do
    if ! kill -0 $LAUNCHER_PID 2>/dev/null; then
        fail "GameLauncher crashed after ${i}s"
        tail -10 /tmp/o3de_launcher.log
        exit 1
    fi
    TOPICS=$(ros2 topic list 2>/dev/null || true)
    if echo "$TOPICS" | grep -qE "scan|camera|image|sensor|tf"; then
        STARTED=true
        pass "GameLauncher initialized (${i}s)"
        break
    fi
    sleep 1
done

if [ "$STARTED" = false ]; then
    fail "GameLauncher did not produce sensor topics in 60s"
    exit 1
fi
echo ""

# ─── Test 2: ROS 2 Topic Throughput ───
info "Test 2: ROS 2 topic throughput (${DURATION}s measurement)..."

ALL_TOPICS=$(ros2 topic list 2>/dev/null || true)
SENSOR_TOPICS=$(echo "$ALL_TOPICS" | grep -iE "scan|camera|image|sensor|tf|clock|cmd_vel" || true)

if [ -z "$SENSOR_TOPICS" ]; then
    fail "No sensor topics found"
else
    echo "$SENSOR_TOPICS" | while read -r topic; do
        [ -z "$topic" ] && continue
        # Measure hz for each topic
        info "  Measuring $topic..."
        HZ_OUTPUT=$(timeout ${DURATION} ros2 topic hz "$topic" 2>&1 &
            sleep $DURATION
            kill %1 2>/dev/null
            wait 2>/dev/null) || true

        # Also get message count
        COUNT_OUTPUT=$(timeout ${DURATION} ros2 topic echo "$topic" --once 2>/dev/null &
            ECHO_PID=$!
            sleep 2
            kill $ECHO_PID 2>/dev/null) || true

        # Extract rate using python
        RATE=$(python3 -c "
import subprocess, time, sys
proc = subprocess.Popen(['ros2', 'topic', 'hz', '$topic'],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(min($DURATION, 10))
proc.terminate()
output = proc.stdout.read()
for line in output.split('\n'):
    if 'average rate' in line:
        rate = line.split(':')[1].strip()
        print(rate)
        break
" 2>/dev/null || echo "N/A")
        perf "  $topic: $RATE"
    done
fi
echo ""

# ─── Test 3: GPU Utilization During Rendering ───
info "Test 3: GPU utilization during rendering (${DURATION}s sample)..."

if command -v nvidia-smi &> /dev/null; then
    # Sample GPU stats over duration
    SAMPLES=()
    for i in $(seq 1 $DURATION); do
        GPU_UTIL=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null | head -1)
        MEM_UTIL=$(nvidia-smi --query-gpu=utilization.memory --format=csv,noheader,nounits 2>/dev/null | head -1)
        MEM_USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
        echo -ne "\r  Sample $i/$DURATION: GPU ${GPU_UTIL}%, MEM ${MEM_UTIL}%, VRAM ${MEM_USED} MiB  "
        SAMPLES+=("$GPU_UTIL")
        sleep 1
    done
    echo ""

    # Calculate stats
    python3 -c "
samples = [${SAMPLES[*]// /,}]
if samples:
    avg = sum(samples) / len(samples)
    peak = max(samples)
    low = min(samples)
    print(f'  Average GPU utilization: {avg:.1f}%')
    print(f'  Peak GPU utilization:    {peak}%')
    print(f'  Minimum GPU utilization: {low}%')
" 2>/dev/null || true

    # Final snapshot
    GPU_MEM_USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
    GPU_TEMP=$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits 2>/dev/null | head -1)
    GPU_POWER=$(nvidia-smi --query-gpu=power.draw --format=csv,noheader,nounits 2>/dev/null | head -1)
    perf "VRAM used: ${GPU_MEM_USED} MiB"
    perf "Temperature: ${GPU_TEMP}C"
    perf "Power draw: ${GPU_POWER}W"
else
    info "nvidia-smi not available, skipping GPU metrics"
fi
echo ""

# ─── Test 4: O3DE Frame Timing (from logs) ───
info "Test 4: O3DE frame timing analysis..."

# Let it run a bit more to accumulate log data
sleep 5

if [ -f /tmp/o3de_launcher.log ]; then
    LOG_SIZE=$(wc -c < /tmp/o3de_launcher.log)
    LOG_LINES=$(wc -l < /tmp/o3de_launcher.log)
    perf "Log size: $LOG_SIZE bytes, $LOG_LINES lines"

    # Look for frame timing info in logs
    FRAME_INFO=$(grep -iE "frame|fps|render|tick" /tmp/o3de_launcher.log 2>/dev/null | tail -10 || true)
    if [ -n "$FRAME_INFO" ]; then
        perf "Frame timing from O3DE logs:"
        echo "$FRAME_INFO" | sed 's/^/    /'
    else
        info "No explicit frame timing in logs (normal for headless mode)"
    fi

    # Check for errors/warnings
    ERROR_COUNT=$(grep -ciE "error|fatal|crash" /tmp/o3de_launcher.log 2>/dev/null || echo "0")
    WARN_COUNT=$(grep -ciE "warning|warn" /tmp/o3de_launcher.log 2>/dev/null || echo "0")
    perf "Log errors: $ERROR_COUNT, warnings: $WARN_COUNT"
fi
echo ""

# ─── Test 5: Latency — Topic Delivery Time ───
info "Test 5: Message delivery latency..."

python3 << 'PYEOF'
import time
import sys

try:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = rclpy.create_node('latency_test')

    # Find a topic to measure
    topics = node.get_topic_names_and_types()
    target = None
    for name, types in topics:
        if any(t in name for t in ['scan', 'clock', 'tf']):
            target = (name, types[0])
            break

    if target is None:
        print("  No suitable topic for latency measurement")
        sys.exit(0)

    topic_name, topic_type = target
    print(f"  Measuring latency on {topic_name} ({topic_type})")

    # Import message type dynamically
    parts = topic_type.split('/')
    mod = __import__(f"{parts[0]}.{parts[1]}", fromlist=[parts[2]])
    msg_class = getattr(mod, parts[2])

    latencies = []
    state = {'count': 0}
    max_msgs = 50

    def callback(msg):
        count = state['count']
        recv_time = time.monotonic()
        # Use header stamp if available
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = node.get_clock().now().nanoseconds * 1e-9
            latency_ms = (now - msg_time) * 1000
            if latency_ms > 0 and latency_ms < 10000:  # sanity check
                latencies.append(latency_ms)
        state['count'] = count + 1

    sub = node.create_subscription(msg_class, topic_name, callback, 10)

    start = time.time()
    while state['count'] < max_msgs and (time.time() - start) < 15:
        rclpy.spin_once(node, timeout_sec=0.1)

    count = state['count']
    print(f"  Received {count} messages in {time.time()-start:.1f}s")
    throughput = count / (time.time() - start) if count > 0 else 0
    print(f"  Throughput: {throughput:.1f} msg/s")

    if latencies:
        avg_lat = sum(latencies) / len(latencies)
        min_lat = min(latencies)
        max_lat = max(latencies)
        p95 = sorted(latencies)[int(len(latencies)*0.95)]
        print(f"  Latency (header→recv): avg={avg_lat:.2f}ms, min={min_lat:.2f}ms, max={max_lat:.2f}ms, p95={p95:.2f}ms")
    else:
        print("  No header timestamps available for latency measurement")

    node.destroy_node()
    rclpy.shutdown()

except Exception as e:
    print(f"  Error: {e}")
PYEOF
echo ""

# ─── Summary ───
echo "============================================="
echo " Performance Test Complete"
echo "============================================="
