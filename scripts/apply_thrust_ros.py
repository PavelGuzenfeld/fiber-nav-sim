#!/usr/bin/env python3
"""Apply thrust to plane model via ros_gz_bridge."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from ros_gz_interfaces.msg import EntityWrench
import sys
import time


class ThrustApplier(Node):
    def __init__(self, force_x=50.0, duration=5.0):
        super().__init__('thrust_applier')
        self.force_x = force_x
        self.duration = duration

        # Publisher for wrench (bridged to Gazebo)
        self.pub = self.create_publisher(EntityWrench, '/world/canyon_world/wrench', 10)

        # Timer to publish at 10Hz
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.publish_wrench)

        self.get_logger().info(f'Applying {force_x}N thrust for {duration}s')

    def publish_wrench(self):
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            self.get_logger().info('Thrust complete')
            rclpy.shutdown()
            return

        msg = EntityWrench()
        msg.entity.name = 'plane'
        msg.entity.type = 2  # MODEL type
        msg.wrench.force.x = self.force_x
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    force = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    node = ThrustApplier(force_x=force, duration=duration)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
