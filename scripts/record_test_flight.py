#!/usr/bin/env python3
"""Record test flight data for performance analysis."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
import csv
import time
import sys

class FlightRecorder(Node):
    def __init__(self, duration=60.0, output_file='/tmp/flight_data.csv'):
        super().__init__('flight_recorder')
        self.duration = duration
        self.output_file = output_file
        self.data = []
        self.start_time = None

        # Ground truth from Gazebo
        self.gt_x = self.gt_y = self.gt_z = 0.0
        self.gt_vx = self.gt_vy = self.gt_vz = 0.0

        # EKF estimate
        self.ekf_x = self.ekf_y = self.ekf_z = 0.0
        self.ekf_vx = self.ekf_vy = self.ekf_vz = 0.0

        # Fusion output
        self.fusion_vx = self.fusion_vy = self.fusion_vz = 0.0

        # Reception counters for debugging
        self.gt_count = 0
        self.ekf_count = 0
        self.fusion_count = 0

        # QoS profile for PX4 topics (BEST_EFFORT reliability, VOLATILE durability)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(Odometry, '/model/plane/odometry', self.gt_callback, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.ekf_callback, px4_qos)
        self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.fusion_callback, px4_qos)

        # Timer for recording
        self.create_timer(0.1, self.record_callback)  # 10 Hz

        self.get_logger().info(f'Recording for {duration}s to {output_file}')

    def gt_callback(self, msg):
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y
        self.gt_z = msg.pose.pose.position.z
        self.gt_vx = msg.twist.twist.linear.x
        self.gt_vy = msg.twist.twist.linear.y
        self.gt_vz = msg.twist.twist.linear.z
        self.gt_count += 1
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('Started recording (received ground truth)')

    def ekf_callback(self, msg):
        self.ekf_x = msg.x
        self.ekf_y = msg.y
        self.ekf_z = msg.z
        self.ekf_vx = msg.vx
        self.ekf_vy = msg.vy
        self.ekf_vz = msg.vz
        self.ekf_count += 1
        if self.ekf_count == 1:
            self.get_logger().info('Receiving EKF data')

    def fusion_callback(self, msg):
        self.fusion_vx = msg.velocity[0]
        self.fusion_vy = msg.velocity[1]
        self.fusion_vz = msg.velocity[2]
        self.fusion_count += 1
        if self.fusion_count == 1:
            self.get_logger().info('Receiving fusion data')

    def record_callback(self):
        if self.start_time is None:
            return

        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.save_data()
            self.get_logger().info('Recording complete!')
            rclpy.shutdown()
            return

        self.data.append({
            'time': elapsed,
            'gt_x': self.gt_x, 'gt_y': self.gt_y, 'gt_z': self.gt_z,
            'gt_vx': self.gt_vx, 'gt_vy': self.gt_vy, 'gt_vz': self.gt_vz,
            'ekf_x': self.ekf_x, 'ekf_y': self.ekf_y, 'ekf_z': self.ekf_z,
            'ekf_vx': self.ekf_vx, 'ekf_vy': self.ekf_vy, 'ekf_vz': self.ekf_vz,
            'fusion_vx': self.fusion_vx, 'fusion_vy': self.fusion_vy, 'fusion_vz': self.fusion_vz,
        })

        if len(self.data) % 100 == 0:
            self.get_logger().info(
                f'Recorded {len(self.data)} samples, {elapsed:.1f}s | '
                f'GT:{self.gt_count} EKF:{self.ekf_count} Fusion:{self.fusion_count}'
            )

    def save_data(self):
        if not self.data:
            return
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data[0].keys())
            writer.writeheader()
            writer.writerows(self.data)
        self.get_logger().info(f'Saved {len(self.data)} samples to {self.output_file}')

def main():
    rclpy.init()
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    node = FlightRecorder(duration=duration)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
