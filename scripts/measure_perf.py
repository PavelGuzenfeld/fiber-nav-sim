#!/usr/bin/env python3
"""Measure fusion node CPU usage and sensor-to-EKF latency.

Subscribes to fusion input/output topics and measures:
- Sensor→fusion latency (spool/vision input timestamp vs fusion output timestamp)
- Fusion→EKF latency (fusion output timestamp vs EKF acknowledgment)
- Fusion node CPU usage (from /proc/<pid>/stat)

Outputs a summary after the measurement period.
"""
import argparse
import math
import os
import re
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
import time
import csv


def get_pid_by_name(name):
    """Find PID of a ROS2 node process by name."""
    try:
        result = subprocess.run(
            ['pgrep', '-f', name], capture_output=True, text=True, timeout=5)
        pids = result.stdout.strip().split('\n')
        return [int(p) for p in pids if p]
    except Exception:
        return []


def get_cpu_percent(pid, interval=1.0):
    """Measure CPU usage of a process over an interval."""
    try:
        stat1 = open(f'/proc/{pid}/stat').read().split()
        time1 = time.time()
        time.sleep(interval)
        stat2 = open(f'/proc/{pid}/stat').read().split()
        time2 = time.time()

        # utime + stime (fields 13, 14 in /proc/pid/stat, 0-indexed)
        ticks1 = int(stat1[13]) + int(stat1[14])
        ticks2 = int(stat2[13]) + int(stat2[14])
        dt = time2 - time1
        hz = os.sysconf('SC_CLK_TCK')
        return (ticks2 - ticks1) / (dt * hz) * 100.0
    except Exception:
        return -1.0


class PerfMeasurer(Node):
    def __init__(self, duration=30.0, output_file=None):
        super().__init__('perf_measurer')
        self.duration = duration
        self.output_file = output_file
        self.start_time = None

        # Latency tracking
        self.fusion_timestamps = []  # (ros_time, wall_time) when fusion msg received
        self.spool_timestamps = []   # wall_time when spool msg received
        self.vision_timestamps = []  # wall_time when vision msg received
        self.ekf_timestamps = []     # wall_time when EKF msg received

        # For computing sensor→fusion latency
        self.last_spool_wall = None
        self.last_vision_wall = None
        self.last_fusion_wall = None
        self.sensor_to_fusion_latencies = []
        self.fusion_rate_times = []

        # Message counts
        self.spool_count = 0
        self.vision_count = 0
        self.fusion_count = 0
        self.ekf_count = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to fusion input topics
        self.create_subscription(
            Float32, '/sensors/fiber_spool/velocity', self._spool_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/sensors/vision_direction', self._vision_cb, 10)
        # Fusion output
        self.create_subscription(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
            self._fusion_cb, px4_qos)
        # EKF output
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._ekf_cb, px4_qos)

        self.timer = self.create_timer(1.0, self._check_done)
        self.get_logger().info(f'Measuring performance for {duration}s...')

    def _spool_cb(self, msg):
        self.last_spool_wall = time.time()
        self.spool_count += 1
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('Started measurement (spool data received)')

    def _vision_cb(self, msg):
        self.last_vision_wall = time.time()
        self.vision_count += 1

    def _fusion_cb(self, msg):
        now = time.time()
        self.fusion_count += 1
        self.fusion_rate_times.append(now)

        # Sensor→fusion latency: time from last sensor input to fusion output
        if self.last_spool_wall is not None:
            lat = (now - self.last_spool_wall) * 1000.0  # ms
            if lat < 200:  # filter outliers (>200ms means missed cycle)
                self.sensor_to_fusion_latencies.append(lat)

        self.last_fusion_wall = now

    def _ekf_cb(self, msg):
        self.ekf_count += 1

    def _check_done(self):
        if self.start_time is None:
            return
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self._report()
            rclpy.shutdown()

    def _report(self):
        elapsed = time.time() - self.start_time

        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  PERFORMANCE MEASUREMENT RESULTS')
        self.get_logger().info('=' * 60)

        # Message rates
        self.get_logger().info(f'\n--- Message Rates ({elapsed:.0f}s) ---')
        self.get_logger().info(
            f'Spool:   {self.spool_count:>5} msgs  '
            f'({self.spool_count/elapsed:.1f} Hz)')
        self.get_logger().info(
            f'Vision:  {self.vision_count:>5} msgs  '
            f'({self.vision_count/elapsed:.1f} Hz)')
        self.get_logger().info(
            f'Fusion:  {self.fusion_count:>5} msgs  '
            f'({self.fusion_count/elapsed:.1f} Hz)')
        self.get_logger().info(
            f'EKF:     {self.ekf_count:>5} msgs  '
            f'({self.ekf_count/elapsed:.1f} Hz)')

        # Sensor→fusion latency
        if self.sensor_to_fusion_latencies:
            lats = self.sensor_to_fusion_latencies
            import numpy as np
            lats_arr = np.array(lats)
            self.get_logger().info(f'\n--- Sensor→Fusion Latency ---')
            self.get_logger().info(f'Mean:    {lats_arr.mean():.1f} ms')
            self.get_logger().info(f'Median:  {np.median(lats_arr):.1f} ms')
            self.get_logger().info(f'P95:     {np.percentile(lats_arr, 95):.1f} ms')
            self.get_logger().info(f'Max:     {lats_arr.max():.1f} ms')
            self.get_logger().info(f'Samples: {len(lats)}')

        # Fusion output rate jitter
        if len(self.fusion_rate_times) > 1:
            import numpy as np
            dts = np.diff(self.fusion_rate_times) * 1000.0  # ms
            self.get_logger().info(f'\n--- Fusion Output Interval ---')
            self.get_logger().info(f'Mean:    {dts.mean():.1f} ms')
            self.get_logger().info(f'Std:     {dts.std():.1f} ms')
            self.get_logger().info(f'Min:     {dts.min():.1f} ms')
            self.get_logger().info(f'Max:     {dts.max():.1f} ms')

        # CPU usage (try to find fusion node PID)
        self.get_logger().info(f'\n--- CPU Usage ---')
        fusion_pids = get_pid_by_name('fiber_vision_fusion')
        if fusion_pids:
            for pid in fusion_pids:
                cpu = get_cpu_percent(pid, interval=2.0)
                self.get_logger().info(f'Fusion node (PID {pid}): {cpu:.1f}%')
        else:
            self.get_logger().info('Fusion node PID not found (not running?)')

        # Save to file
        if self.output_file and self.sensor_to_fusion_latencies:
            import numpy as np
            lats_arr = np.array(self.sensor_to_fusion_latencies)
            with open(self.output_file, 'w') as f:
                f.write('metric,value\n')
                f.write(f'duration_s,{elapsed:.1f}\n')
                f.write(f'spool_hz,{self.spool_count/elapsed:.1f}\n')
                f.write(f'vision_hz,{self.vision_count/elapsed:.1f}\n')
                f.write(f'fusion_hz,{self.fusion_count/elapsed:.1f}\n')
                f.write(f'ekf_hz,{self.ekf_count/elapsed:.1f}\n')
                f.write(f'latency_mean_ms,{lats_arr.mean():.1f}\n')
                f.write(f'latency_p95_ms,{np.percentile(lats_arr, 95):.1f}\n')
                f.write(f'latency_max_ms,{lats_arr.max():.1f}\n')
            self.get_logger().info(f'\nSaved to {self.output_file}')


def main():
    parser = argparse.ArgumentParser(description='Measure fusion performance')
    parser.add_argument('duration', nargs='?', type=float, default=30.0)
    parser.add_argument('output', nargs='?', default=None)
    args = parser.parse_args()

    rclpy.init()
    node = PerfMeasurer(args.duration, args.output)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
