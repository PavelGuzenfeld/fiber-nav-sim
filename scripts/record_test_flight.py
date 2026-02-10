#!/usr/bin/env python3
"""Record test flight data for performance analysis.

All output is in NED frame for direct comparison:
- GT position: converted from Gazebo ENU to NED
- GT velocity: rotated from body-frame to NED using quaternion
- EKF position/velocity: already NED from PX4
- Fusion: already NED (VehicleOdometry)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
import csv
import math
import time
import sys


def quat_rotate(qw, qx, qy, qz, vx, vy, vz):
    """Rotate vector (vx,vy,vz) by quaternion (qw,qx,qy,qz).

    Uses the cross-product shortcut: t = 2*(q_vec x v), v' = v + w*t + q_vec x t
    """
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def enu_to_ned(x, y, z):
    """Convert ENU coordinates to NED."""
    return y, x, -z


class FlightRecorder(Node):
    def __init__(self, duration=60.0, output_file='/tmp/flight_data.csv'):
        super().__init__('flight_recorder')
        self.duration = duration
        self.output_file = output_file
        self.data = []
        self.start_time = None

        # Ground truth from Gazebo (raw ENU + body-frame velocity)
        self._gt_x_enu = self._gt_y_enu = self._gt_z_enu = 0.0
        self._gt_vx_body = self._gt_vy_body = self._gt_vz_body = 0.0
        self._gt_qw = 1.0
        self._gt_qx = self._gt_qy = self._gt_qz = 0.0
        # Converted NED values
        self.gt_x = self.gt_y = self.gt_z = 0.0
        self.gt_vx = self.gt_vy = self.gt_vz = 0.0

        # EKF estimate (already NED)
        self.ekf_x = self.ekf_y = self.ekf_z = 0.0
        self.ekf_vx = self.ekf_vy = self.ekf_vz = 0.0

        # Fusion output (already NED)
        self.fusion_vx = self.fusion_vy = self.fusion_vz = 0.0
        self.fusion_px = self.fusion_py = self.fusion_pz = float('nan')

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
        self.create_subscription(Odometry, '/model/quadtailsitter/odometry', self.gt_callback, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.ekf_callback, px4_qos)
        self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.fusion_callback, px4_qos)

        # Timer for recording
        self.create_timer(0.1, self.record_callback)  # 10 Hz

        self.get_logger().info(f'Recording for {duration}s to {output_file}')

    def gt_callback(self, msg):
        # Store raw ENU position
        self._gt_x_enu = msg.pose.pose.position.x
        self._gt_y_enu = msg.pose.pose.position.y
        self._gt_z_enu = msg.pose.pose.position.z
        # Store body-frame velocity
        self._gt_vx_body = msg.twist.twist.linear.x
        self._gt_vy_body = msg.twist.twist.linear.y
        self._gt_vz_body = msg.twist.twist.linear.z
        # Store quaternion (ENU frame)
        self._gt_qw = msg.pose.pose.orientation.w
        self._gt_qx = msg.pose.pose.orientation.x
        self._gt_qy = msg.pose.pose.orientation.y
        self._gt_qz = msg.pose.pose.orientation.z

        # Convert position ENU -> NED
        self.gt_x, self.gt_y, self.gt_z = enu_to_ned(
            self._gt_x_enu, self._gt_y_enu, self._gt_z_enu)

        # Rotate velocity body -> ENU world using quaternion, then ENU -> NED
        vx_enu, vy_enu, vz_enu = quat_rotate(
            self._gt_qw, self._gt_qx, self._gt_qy, self._gt_qz,
            self._gt_vx_body, self._gt_vy_body, self._gt_vz_body)
        self.gt_vx, self.gt_vy, self.gt_vz = enu_to_ned(vx_enu, vy_enu, vz_enu)

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
        self.fusion_px = msg.position[0]
        self.fusion_py = msg.position[1]
        self.fusion_pz = msg.position[2]
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
            'fusion_px': self.fusion_px, 'fusion_py': self.fusion_py, 'fusion_pz': self.fusion_pz,
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
    output_file = sys.argv[2] if len(sys.argv) > 2 else '/tmp/flight_data.csv'
    node = FlightRecorder(duration=duration, output_file=output_file)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.save_data()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
