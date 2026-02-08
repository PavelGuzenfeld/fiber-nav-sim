#!/usr/bin/env python3
"""Offboard takeoff for PX4 SITL tailsitter.

Arms the vehicle, switches to offboard mode, commands upward velocity for takeoff,
then holds position. Works around the auto takeoff deadlock for tailsitters.
"""

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


class OffboardTakeoff(Node):
    def __init__(self, target_alt=5.0):
        super().__init__('offboard_takeoff')
        self.target_alt = target_alt  # meters above ground

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

        self.armed = False
        self.nav_state = 0
        self.z = 0.0  # NED z (negative = up)
        self.vz = 0.0
        self.dist_bottom = 0.0
        self.ground_contact = False
        self.landed = False
        self.offboard_counter = 0
        self.state = 'PREFLIGHT'
        self.state_start = time.time()

        self.timer = self.create_timer(0.05, self._loop)  # 20 Hz

    def _status_cb(self, msg):
        self.armed = msg.arming_state == 2
        self.nav_state = msg.nav_state

    def _lpos_cb(self, msg):
        self.z = msg.z
        self.vz = msg.vz
        self.dist_bottom = msg.dist_bottom

    def _land_cb(self, msg):
        self.ground_contact = msg.ground_contact
        self.landed = msg.landed

    def _send_offboard_heartbeat(self, velocity=True, position=False):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard.publish(msg)

    def _send_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw=float('nan')):
        msg = TrajectorySetpoint()
        # CRITICAL: position defaults to [0,0,0] not NaN — PX4 treats non-NaN
        # as valid position constraints, creating a hard altitude ceiling
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        msg.yawspeed = float('nan')
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

    def _loop(self):
        now = time.time()
        elapsed = now - self.state_start

        if self.state == 'PREFLIGHT':
            # Send offboard heartbeats before arming (need >= 10 before switching)
            self._send_offboard_heartbeat()
            self._send_velocity(vz=-1.0)  # NED: -1 = up at 1 m/s
            self.offboard_counter += 1
            if self.offboard_counter > 80:  # 4 seconds of heartbeats for DDS discovery
                self.state = 'ARMING'
                self.state_start = now
                self.get_logger().info('Sending arm command')
                self._send_command(400, param1=1.0, param2=21196.0)  # ARM

        elif self.state == 'ARMING':
            self._send_offboard_heartbeat()
            self._send_velocity(vz=-1.0)
            if self.armed:
                self.state = 'SET_OFFBOARD'
                self.state_start = now
                self.get_logger().info('Armed! Switching to offboard mode')
                self._send_command(176, param1=1.0, param2=6.0)  # OFFBOARD

            elif elapsed > 10.0:
                self.get_logger().error('Failed to arm after 10s')
                self.state = 'DONE'

        elif self.state == 'SET_OFFBOARD':
            self._send_offboard_heartbeat()
            self._send_velocity(vz=-1.0)
            if self.nav_state == 14:  # OFFBOARD
                self.state = 'CLIMBING'
                self.state_start = now
                self.get_logger().info(f'Offboard mode active! Climbing to {self.target_alt}m')
            elif elapsed > 0.5:
                # Retry offboard switch
                self._send_command(176, param1=1.0, param2=6.0)
                if elapsed > 5.0:
                    self.get_logger().error('Failed to switch to offboard')
                    self.state = 'DONE'

        elif self.state == 'CLIMBING':
            self._send_offboard_heartbeat()
            alt = -self.z  # Convert NED to altitude
            if alt >= self.target_alt - 0.5:
                self.state = 'HOLDING'
                self.state_start = now
                self.get_logger().info(
                    f'Reached target altitude {alt:.1f}m, holding position'
                )
            else:
                self._send_velocity(vz=-2.0)  # Climb at 2 m/s
                if elapsed % 2.0 < 0.1:
                    self.get_logger().info(
                        f'Alt: {alt:.1f}m/{self.target_alt}m '
                        f'vz: {self.vz:.3f} '
                        f'dist_bot: {self.dist_bottom:.2f} '
                        f'gnd: {self.ground_contact} '
                        f'land: {self.landed}'
                    )

        elif self.state == 'HOLDING':
            self._send_offboard_heartbeat(position=True, velocity=False)
            msg = TrajectorySetpoint()
            msg.position = [float('nan'), float('nan'), -self.target_alt]
            msg.yaw = float('nan')
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.pub_traj.publish(msg)
            if elapsed > 30.0:
                self.get_logger().info('Hold complete. Landing.')
                self._send_command(21)  # LAND
                self.state = 'DONE'

        elif self.state == 'DONE':
            self.get_logger().info('Done.')
            raise SystemExit(0)


def main():
    rclpy.init()
    target = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    node = OffboardTakeoff(target)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
