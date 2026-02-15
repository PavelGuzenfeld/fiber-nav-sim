"""TERCOM vs GPS ground truth evaluator.

Subscribes to Gazebo ground truth odometry and TERCOM position estimates,
logs to CSV, and computes CEP50/CEP95 on shutdown.
"""

import csv
import math
import os
import signal
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TercomEvaluator(Node):
    def __init__(self):
        super().__init__('tercom_evaluator')

        self.declare_parameter('log_dir', '/root/ws/src/fiber-nav-sim/logs')
        self.declare_parameter('log_rate_hz', 1.0)

        log_dir = Path(self.get_parameter('log_dir').as_string())
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path_ = log_dir / f'tercom_eval_{timestamp}.csv'
        self.csv_file_ = open(self.csv_path_, 'w', newline='')
        self.csv_writer_ = csv.writer(self.csv_file_)
        self.csv_writer_.writerow([
            'time_s', 'gt_x', 'gt_y', 'gt_z',
            'tercom_x', 'tercom_y', 'tercom_sigma',
            'lrf_agl',
        ])

        self.gt_pos_ = None  # (x, y, z) from odometry
        self.tercom_pos_ = None  # (x, y, sigma)
        self.lrf_agl_ = float('nan')
        self.errors_ = []  # list of (dx, dy) for CEP computation
        self.start_time_ = self.get_clock().now()

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Odometry,
            '/model/quadtailsitter/odometry',
            self._on_odometry, 10)
        self.create_subscription(
            PointStamped,
            '/tercom/position',
            self._on_tercom, 10)
        self.create_subscription(
            LaserScan,
            '/laser_rangefinder',
            self._on_lrf, best_effort)

        rate = self.get_parameter('log_rate_hz').as_double()
        self.create_timer(1.0 / rate, self._log_tick)

        self.get_logger().info(f'TERCOM evaluator logging to {self.csv_path_}')

    def _on_odometry(self, msg: Odometry):
        p = msg.pose.pose.position
        self.gt_pos_ = (p.x, p.y, p.z)

    def _on_tercom(self, msg: PointStamped):
        # TERCOM publishes (x, y) in point, sigma encoded in z
        self.tercom_pos_ = (msg.point.x, msg.point.y, msg.point.z)

    def _on_lrf(self, msg: LaserScan):
        if msg.ranges:
            self.lrf_agl_ = msg.ranges[0]

    def _log_tick(self):
        if self.gt_pos_ is None:
            return

        elapsed = (self.get_clock().now() - self.start_time_).nanoseconds * 1e-9
        gt_x, gt_y, gt_z = self.gt_pos_

        tc_x = tc_y = tc_sigma = float('nan')
        if self.tercom_pos_ is not None:
            tc_x, tc_y, tc_sigma = self.tercom_pos_
            dx = tc_x - gt_x
            dy = tc_y - gt_y
            self.errors_.append((dx, dy))

        self.csv_writer_.writerow([
            f'{elapsed:.2f}',
            f'{gt_x:.3f}', f'{gt_y:.3f}', f'{gt_z:.3f}',
            f'{tc_x:.3f}', f'{tc_y:.3f}', f'{tc_sigma:.3f}',
            f'{self.lrf_agl_:.3f}',
        ])
        self.csv_file_.flush()

    def compute_metrics(self):
        if not self.errors_:
            self.get_logger().warn('No TERCOM fixes received — no metrics to compute')
            return

        errors = np.array(self.errors_)
        distances = np.sqrt(errors[:, 0]**2 + errors[:, 1]**2)

        cep50 = float(np.percentile(distances, 50))
        cep95 = float(np.percentile(distances, 95))
        mean_err = float(np.mean(distances))
        max_err = float(np.max(distances))
        n_fixes = len(distances)

        self.get_logger().info(
            f'TERCOM Performance ({n_fixes} fixes):\n'
            f'  CEP50:    {cep50:.1f} m\n'
            f'  CEP95:    {cep95:.1f} m\n'
            f'  Mean err: {mean_err:.1f} m\n'
            f'  Max err:  {max_err:.1f} m\n'
            f'  Log:      {self.csv_path_}')

    def destroy_node(self):
        self.compute_metrics()
        self.csv_file_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TercomEvaluator()

    def shutdown_handler(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
