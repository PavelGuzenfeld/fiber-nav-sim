#!/usr/bin/env python3
"""
Record ground truth and estimated trajectories for analysis.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # Parameters
        self.declare_parameter('model_name', 'plane')
        self.declare_parameter('output_dir', '/tmp/fiber_nav_data')
        self.declare_parameter('record_rate', 10.0)

        self.model_name = self.get_parameter('model_name').value
        self.output_dir = self.get_parameter('output_dir').value
        record_rate = self.get_parameter('record_rate').value

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

        # Subscribers
        self.gt_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.gt_callback,
            10
        )

        self.est_sub = self.create_subscription(
            Odometry,
            '/fmu/out/vehicle_odometry',  # PX4 odometry output
            self.est_callback,
            10
        )

        # Timer for status updates
        self.gt_count = 0
        self.est_count = 0
        self.create_timer(5.0, self.status_callback)

        self.get_logger().info(f'Recording to: {self.output_dir}')
        self.get_logger().info(f'Model: {self.model_name}')

    def gt_callback(self, msg: ModelStates):
        """Record ground truth from Gazebo."""
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.gt_writer.writerow([
            timestamp,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ])

        self.gt_count += 1

    def est_callback(self, msg: Odometry):
        """Record estimated trajectory from PX4 EKF."""
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.est_writer.writerow([
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
