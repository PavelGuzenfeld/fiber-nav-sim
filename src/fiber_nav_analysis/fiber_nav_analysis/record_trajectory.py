#!/usr/bin/env python3
"""
Record ground truth and estimated trajectories for analysis.
Works with Gazebo Harmonic via ros_gz_bridge.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import csv
import os
from datetime import datetime


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # Parameters
        self.declare_parameter('output_dir', '/root/data')
        self.declare_parameter('gt_topic', '/model/quadtailsitter/odometry')
        self.declare_parameter('est_topic', '/fmu/in/vehicle_visual_odometry')

        self.output_dir = self.get_parameter('output_dir').value
        gt_topic = self.get_parameter('gt_topic').value
        est_topic = self.get_parameter('est_topic').value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.gt_file = os.path.join(self.output_dir, f'ground_truth_{timestamp}.csv')
        self.est_file = os.path.join(self.output_dir, f'estimated_{timestamp}.csv')

        # Open CSV files
        self.gt_csv = open(self.gt_file, 'w', newline='')
        self.est_csv = open(self.est_file, 'w', newline='')

        self.gt_writer = csv.writer(self.gt_csv)
        self.est_writer = csv.writer(self.est_csv)

        # Write headers
        header = ['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz']
        self.gt_writer.writerow(header)
        self.est_writer.writerow(header)

        # Ground truth from Gazebo via ros_gz_bridge
        self.gt_sub = self.create_subscription(
            Odometry,
            gt_topic,
            self.gt_callback,
            10
        )

        # Estimated from fusion node (PX4 format)
        self.est_sub = self.create_subscription(
            VehicleOdometry,
            est_topic,
            self.est_callback,
            10
        )

        # Timer for status updates
        self.gt_count = 0
        self.est_count = 0
        self.create_timer(5.0, self.status_callback)

        self.get_logger().info(f'Recording to: {self.output_dir}')
        self.get_logger().info(f'GT topic: {gt_topic}')
        self.get_logger().info(f'EST topic: {est_topic}')

    def gt_callback(self, msg: Odometry):
        """Record ground truth from Gazebo Harmonic."""
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.gt_writer.writerow([
            timestamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ])

        self.gt_count += 1

    def est_callback(self, msg: VehicleOdometry):
        """Record estimated trajectory from fusion node."""
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.est_writer.writerow([
            timestamp,
            msg.position[0] if not any(map(lambda x: x != x, msg.position)) else 0.0,
            msg.position[1] if not any(map(lambda x: x != x, msg.position)) else 0.0,
            msg.position[2] if not any(map(lambda x: x != x, msg.position)) else 0.0,
            msg.velocity[0],
            msg.velocity[1],
            msg.velocity[2],
            msg.q[0],
            msg.q[1],
            msg.q[2],
            msg.q[3],
        ])

        self.est_count += 1

    def status_callback(self):
        """Print recording status."""
        self.get_logger().info(f'Recorded: GT={self.gt_count}, EST={self.est_count}')

    def destroy_node(self):
        """Close files on shutdown."""
        self.gt_csv.close()
        self.est_csv.close()
        self.get_logger().info(f'Saved: {self.gt_file}')
        self.get_logger().info(f'Saved: {self.est_file}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
