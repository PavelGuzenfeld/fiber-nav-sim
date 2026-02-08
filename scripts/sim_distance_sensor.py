#!/usr/bin/env python3
"""Simulated downward distance sensor for PX4 SITL.

Reads Gazebo ground truth odometry and publishes a distance_sensor message
to PX4, enabling the terrain estimator and dist_bottom_valid. This breaks
the flying_but_ground_contact deadlock in MulticopterPositionControl.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import DistanceSensor


class SimDistanceSensor(Node):
    def __init__(self):
        super().__init__('sim_distance_sensor')

        qos_gz = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Odometry,
            '/model/quadtailsitter/odometry',
            self._odom_cb,
            qos_gz,
        )
        self.pub = self.create_publisher(
            DistanceSensor, '/fmu/in/distance_sensor', qos_px4
        )

        self.get_logger().info('Sim distance sensor started')

    def _odom_cb(self, msg):
        # Gazebo ENU: z = height above ground (positive up)
        height = msg.pose.pose.position.z
        if height < 0.0:
            height = 0.0

        ds = DistanceSensor()
        ds.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ds.min_distance = 0.1
        ds.max_distance = 50.0
        ds.current_distance = max(0.1, float(height))
        ds.variance = 0.01
        ds.signal_quality = 100
        ds.type = 0  # LASER
        ds.h_fov = 0.05
        ds.v_fov = 0.05
        ds.orientation = 25  # ROTATION_DOWNWARD_FACING
        ds.mode = 1  # MODE_ENABLED

        self.pub.publish(ds)


def main():
    rclpy.init()
    node = SimDistanceSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
