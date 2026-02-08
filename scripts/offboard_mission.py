#!/usr/bin/env python3
"""Offboard canyon mission for PX4 SITL.

Takeoff, fly back and forth along the canyon, return home, land.
Canyon axis runs along PX4 East (Y+). Walls at PX4 North (X) ±70m.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
import time
import sys

NAN = float('nan')


class OffboardMission(Node):
    def __init__(self, altitude=15.0):
        super().__init__('offboard_mission')
        self.cruise_alt = altitude  # meters AGL

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos
        )

        self.sub_status = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._status_cb, qos
        )
        self.sub_lpos = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._lpos_cb,
            qos,
        )
        self.sub_land = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self._land_cb,
            qos,
        )

        # State
        self.armed = False
        self.nav_state = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vz = 0.0
        self.heading = 0.0
        self.dist_bottom = 0.0
        self.ground_contact = False
        self.landed = False
        self.offboard_counter = 0
        self.state = 'PREFLIGHT'
        self.state_start = time.time()

        # Waypoints: (x_ned, y_ned, z_ned, hold_time_s)
        # NED frame relative to home. z negative = up.
        # Canyon runs along PX4 East (Y+). Fly out 400m, back, out 200m, back.
        self.waypoints = [
            (0.0, 200.0, -self.cruise_alt, 2.0),    # 200m East along canyon
            (0.0, 400.0, -self.cruise_alt, 3.0),    # 400m East (turnaround)
            (0.0, 200.0, -self.cruise_alt, 2.0),    # Back to 200m
            (0.0, 0.0, -self.cruise_alt, 3.0),      # Back to start
        ]
        self.wp_index = 0
        self.wp_hold_start = None
        self.wp_acceptance_radius = 5.0  # meters (larger for longer legs)

        self.timer = self.create_timer(0.05, self._loop)  # 20 Hz

    def _status_cb(self, msg):
        self.armed = msg.arming_state == 2
        self.nav_state = msg.nav_state

    def _lpos_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vz = msg.vz
        self.heading = msg.heading
        self.dist_bottom = msg.dist_bottom

    def _land_cb(self, msg):
        self.ground_contact = msg.ground_contact
        self.landed = msg.landed

    def _send_heartbeat(self, velocity=False, position=False):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard.publish(msg)

    def _send_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw=NAN):
        msg = TrajectorySetpoint()
        msg.position = [NAN, NAN, NAN]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = yaw
        msg.yawspeed = NAN
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_traj.publish(msg)

    def _send_position(self, x=NAN, y=NAN, z=NAN, yaw=NAN):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [NAN, NAN, NAN]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = yaw
        msg.yawspeed = NAN
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_traj.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def _dist_to_wp(self, wp):
        dx = wp[0] - self.x
        dy = wp[1] - self.y
        dz = wp[2] - self.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _yaw_to_wp(self, wp):
        dx = wp[0] - self.x
        dy = wp[1] - self.y
        return math.atan2(dy, dx)

    def _loop(self):
        now = time.time()
        elapsed = now - self.state_start
        alt = -self.z

        if self.state == 'PREFLIGHT':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            self.offboard_counter += 1
            if self.offboard_counter > 400:  # 20s warmup for cold PX4
                self.state = 'ARMING'
                self.state_start = now
                self.get_logger().info('Sending arm command')
                self._send_command(400, param1=1.0, param2=21196.0)

        elif self.state == 'ARMING':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            if self.armed:
                self.state = 'SET_OFFBOARD'
                self.state_start = now
                self.get_logger().info('Armed! Switching to offboard')
                self._send_command(176, param1=1.0, param2=6.0)
            elif elapsed > 10.0:
                self.get_logger().error('Failed to arm after 10s')
                self.state = 'DONE'
            elif int(elapsed * 2) % 2 == 0 and (elapsed * 2) % 2 < 0.12:
                # Retry arm command every ~1s
                self._send_command(400, param1=1.0, param2=21196.0)

        elif self.state == 'SET_OFFBOARD':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            if self.nav_state == 14:
                self.state = 'TAKEOFF'
                self.state_start = now
                self.get_logger().info(
                    f'Offboard active! Taking off to {self.cruise_alt}m'
                )
            elif elapsed > 0.5:
                self._send_command(176, param1=1.0, param2=6.0)
                if elapsed > 5.0:
                    self.get_logger().error('Failed to switch to offboard')
                    self.state = 'DONE'

        elif self.state == 'TAKEOFF':
            self._send_heartbeat(velocity=True)
            if alt >= self.cruise_alt - 0.5:
                self.state = 'NAVIGATE'
                self.state_start = now
                self.wp_index = 0
                wp = self.waypoints[0]
                self.get_logger().info(
                    f'Reached {alt:.1f}m. Starting mission with '
                    f'{len(self.waypoints)} waypoints'
                )
            else:
                self._send_velocity(vz=-2.0)
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                    self.get_logger().info(
                        f'Takeoff: {alt:.1f}m / {self.cruise_alt}m  '
                        f'vz={self.vz:.2f}'
                    )

        elif self.state == 'NAVIGATE':
            self._send_heartbeat(position=True)
            wp = self.waypoints[self.wp_index]
            dist = self._dist_to_wp(wp)
            yaw = self._yaw_to_wp(wp)

            self._send_position(wp[0], wp[1], wp[2], yaw)

            if dist < self.wp_acceptance_radius:
                if self.wp_hold_start is None:
                    self.wp_hold_start = now
                    self.get_logger().info(
                        f'WP{self.wp_index + 1} reached at '
                        f'({self.x:.1f}, {self.y:.1f}, {alt:.1f}m). '
                        f'Holding {wp[3]:.0f}s...'
                    )
                elif now - self.wp_hold_start >= wp[3]:
                    self.wp_hold_start = None
                    self.wp_index += 1
                    if self.wp_index >= len(self.waypoints):
                        self.state = 'RTL'
                        self.state_start = now
                        self.get_logger().info(
                            'All waypoints complete. Returning to land.'
                        )
                    else:
                        nwp = self.waypoints[self.wp_index]
                        self.get_logger().info(
                            f'Heading to WP{self.wp_index + 1} '
                            f'({nwp[0]:.0f}, {nwp[1]:.0f})'
                        )
            else:
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                    self.get_logger().info(
                        f'WP{self.wp_index + 1}: dist={dist:.1f}m  '
                        f'pos=({self.x:.1f},{self.y:.1f},{alt:.1f}m)'
                    )

        elif self.state == 'RTL':
            self._send_heartbeat(position=True)
            # Hold position above home, then descend
            self._send_position(0.0, 0.0, -self.cruise_alt)
            home_dist = math.sqrt(self.x**2 + self.y**2)
            if home_dist < 2.0:
                self.state = 'DESCEND'
                self.state_start = now
                self.get_logger().info('Over home. Descending...')
            elif int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self.get_logger().info(
                    f'RTL: {home_dist:.1f}m from home  '
                    f'pos=({self.x:.1f},{self.y:.1f},{alt:.1f}m)'
                )

        elif self.state == 'DESCEND':
            self._send_heartbeat(velocity=True)
            if alt < 0.5 or self.landed:
                self.get_logger().info('Landed! Mission complete.')
                self._send_command(400, param1=0.0)  # DISARM
                self.state = 'DONE'
            else:
                self._send_velocity(vz=1.0)  # descend at 1 m/s
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                    self.get_logger().info(
                        f'Descending: {alt:.1f}m  vz={self.vz:.2f}'
                    )

        elif self.state == 'DONE':
            self.get_logger().info('Done.')
            raise SystemExit(0)


def main():
    rclpy.init()
    alt = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    node = OffboardMission(alt)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
