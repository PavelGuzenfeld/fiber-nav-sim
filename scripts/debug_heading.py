#!/usr/bin/env python3
"""Capture velocity/direction diagnostics during flight."""
import rclpy
import math
import time
import json
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String, Float64
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition


class DiagCapture(Node):
    def __init__(self):
        super().__init__('diag_capture')
        self.gt_body_vel = None
        self.gt_pos = None
        self.vision_dir = None
        self.of_dir = None
        self.phase = '?'
        self.dir_health = 0
        self.source = 0.0

        self.create_subscription(
            Odometry, '/model/quadtailsitter/odometry', self.odom_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/sensors/vision_direction', self.dir_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/sensors/optical_flow/direction', self.of_dir_cb, 10)
        self.create_subscription(
            String, '/sensors/fusion/diagnostics', self.diag_cb, 10)
        self.create_subscription(
            Float64, '/sensors/optical_flow/source', self.source_cb, 10)

    def odom_cb(self, msg):
        t = msg.twist.twist.linear
        self.gt_body_vel = (t.x, t.y, t.z)
        p = msg.pose.pose.position
        self.gt_pos = (p.x, p.y, p.z)

    def dir_cb(self, msg):
        self.vision_dir = (msg.vector.x, msg.vector.y, msg.vector.z)

    def of_dir_cb(self, msg):
        self.of_dir = (msg.vector.x, msg.vector.y, msg.vector.z)

    def diag_cb(self, msg):
        d = json.loads(msg.data)
        self.phase = d.get('flight_phase', '?')
        self.dir_health = d.get('direction_health_pct', 0)

    def source_cb(self, msg):
        self.source = msg.data


def main():
    rclpy.init()
    node = DiagCapture()

    start = time.time()
    last_print = 0
    while time.time() - start < 15:
        rclpy.spin_once(node, timeout_sec=0.1)

        if (node.gt_body_vel and node.vision_dir
                and time.time() - last_print > 2):
            bv = node.gt_body_vel
            speed = math.sqrt(bv[0]**2 + bv[1]**2 + bv[2]**2)
            vd = node.vision_dir
            gp = node.gt_pos or (0, 0, 0)

            # Compute body-frame heading (atan2 of horizontal components)
            # Body vel heading in body frame
            bv_hdg = math.degrees(math.atan2(bv[1], bv[0]))

            # Vision direction heading in body frame
            vd_hdg = math.degrees(math.atan2(vd[1], vd[0]))

            print(f'Phase: {node.phase}  DirHealth: {node.dir_health}%  '
                  f'Source: {node.source}')
            print(f'  GT body vel:  ({bv[0]:7.2f}, {bv[1]:7.2f}, {bv[2]:7.2f})'
                  f'  speed={speed:.1f}  body_hdg={bv_hdg:.0f}deg')
            print(f'  GT pos (ENU): ({gp[0]:7.0f}, {gp[1]:7.0f}, {gp[2]:7.0f})')
            print(f'  Vision dir:   ({vd[0]:7.3f}, {vd[1]:7.3f}, {vd[2]:7.3f})'
                  f'  dir_hdg={vd_hdg:.0f}deg')
            if node.of_dir:
                od = node.of_dir
                od_hdg = math.degrees(math.atan2(od[1], od[0]))
                print(f'  OF raw dir:   ({od[0]:7.3f}, {od[1]:7.3f}, {od[2]:7.3f})'
                      f'  of_hdg={od_hdg:.0f}deg')
            print('---')
            last_print = time.time()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
