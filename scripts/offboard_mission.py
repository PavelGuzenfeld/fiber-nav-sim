#!/usr/bin/env python3
"""Offboard canyon mission for PX4 SITL.

Takeoff, fly back and forth along the canyon, return home, land.
Canyon axis runs along PX4 East (Y+). Walls at PX4 North (X) +/-70m.

With --vtol flag: transition to FW after takeoff, navigate waypoints in FW mode
using velocity commands, transition back to MC for landing.
"""

import argparse
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

NAN = float('nan')

# MAV_CMD_DO_VTOL_TRANSITION
CMD_DO_VTOL_TRANSITION = 3000
VEHICLE_TYPE_MC = 1
VEHICLE_TYPE_FW = 2


class OffboardMission(Node):
    def __init__(self, altitude=15.0, vtol=False, canyon=False):
        super().__init__('offboard_mission')
        self.cruise_alt = altitude
        self.vtol = vtol

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
        self.vehicle_type = VEHICLE_TYPE_MC
        self.in_transition = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.heading = 0.0
        self.dist_bottom = 0.0
        self.ground_contact = False
        self.landed = False
        self.offboard_counter = 0
        self.state = 'PREFLIGHT'
        self.state_start = time.time()
        self.transition_cmd_sent = False

        # Waypoints: (x_ned, y_ned, z_ned, hold_time_s)
        # NED frame relative to home. z negative = up.
        # Canyon runs along PX4 East (Y+).
        if vtol and canyon:
            # VTOL FW: full canyon traversal (1200m through canyon with curve)
            # Canyon runs along NED Y+ (ENU X+). Walls at NED X +/-75m.
            # Curve starts ~1300m, gentle left turn (0.2 rad ≈ 11.5°).
            self.waypoints = [
                (0.0, 200.0, -self.cruise_alt, 0.0),     # 200m East
                (0.0, 400.0, -self.cruise_alt, 0.0),     # 400m East
                (0.0, 600.0, -self.cruise_alt, 0.0),     # 600m East
                (0.0, 800.0, -self.cruise_alt, 0.0),     # 800m East
                (0.0, 1000.0, -self.cruise_alt, 0.0),    # 1000m East
                (-20.0, 1200.0, -self.cruise_alt, 0.0),  # Pre-curve bias
                (-40.0, 1400.0, -self.cruise_alt, 0.0),  # Through curve
            ]
        elif vtol:
            # VTOL FW: one-way east path (no 180-degree turns in canyon)
            self.waypoints = [
                (0.0, 100.0, -self.cruise_alt, 0.0),   # 100m East
                (0.0, 200.0, -self.cruise_alt, 0.0),   # 200m East
                (0.0, 300.0, -self.cruise_alt, 0.0),   # 300m East
                (0.0, 400.0, -self.cruise_alt, 0.0),   # 400m East (end)
            ]
        else:
            # MC: back-and-forth pattern
            self.waypoints = [
                (0.0, 200.0, -self.cruise_alt, 2.0),   # 200m East along canyon
                (0.0, 400.0, -self.cruise_alt, 3.0),   # 400m East (turnaround)
                (0.0, 200.0, -self.cruise_alt, 2.0),   # Back to 200m
                (0.0, 0.0, -self.cruise_alt, 3.0),     # Back to start
            ]
        self.wp_index = 0
        self.wp_hold_start = None
        # FW needs larger acceptance radius (can't stop on a dime)
        self.wp_acceptance_radius = 20.0 if vtol else 5.0

        # FW cruise speed for velocity-based navigation
        self.fw_cruise_speed = 15.0

        self.timer = self.create_timer(0.05, self._loop)  # 20 Hz

    def _status_cb(self, msg):
        self.armed = msg.arming_state == 2
        self.nav_state = msg.nav_state
        self.vehicle_type = msg.vehicle_type
        self.in_transition = msg.in_transition_mode

    def _lpos_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
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

    def _horiz_dist_to_wp(self, wp):
        dx = wp[0] - self.x
        dy = wp[1] - self.y
        return math.sqrt(dx * dx + dy * dy)

    def _dist_to_wp(self, wp):
        dx = wp[0] - self.x
        dy = wp[1] - self.y
        dz = wp[2] - self.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _yaw_to_wp(self, wp):
        dx = wp[0] - self.x
        dy = wp[1] - self.y
        return math.atan2(dy, dx)

    def _velocity_toward_wp(self, wp):
        """Compute velocity vector toward waypoint at cruise speed."""
        dx = wp[0] - self.x
        dy = wp[1] - self.y
        horiz_dist = math.sqrt(dx * dx + dy * dy)
        if horiz_dist < 1.0:
            return 0.0, 0.0, 0.0
        scale = self.fw_cruise_speed / horiz_dist
        return dx * scale, dy * scale, 0.0

    def _set_state(self, new_state):
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self.state_start = time.time()
        self.transition_cmd_sent = False

    def _type_str(self):
        if self.in_transition:
            return 'TRANSITION'
        return 'MC' if self.vehicle_type == VEHICLE_TYPE_MC else 'FW'

    def _loop(self):
        now = time.time()
        elapsed = now - self.state_start
        alt = -self.z
        speed = math.sqrt(self.vx**2 + self.vy**2)

        if self.state == 'PREFLIGHT':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            self.offboard_counter += 1
            if self.offboard_counter > 400:  # 20s warmup for cold PX4
                self._set_state('ARMING')
                self._send_command(400, param1=1.0, param2=21196.0)

        elif self.state == 'ARMING':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            if self.armed:
                self._set_state('SET_OFFBOARD')
                self._send_command(176, param1=1.0, param2=6.0)
            elif elapsed > 10.0:
                self.get_logger().error('Failed to arm after 10s')
                self._set_state('DONE')
            elif int(elapsed * 2) % 2 == 0 and (elapsed * 2) % 2 < 0.12:
                self._send_command(400, param1=1.0, param2=21196.0)

        elif self.state == 'SET_OFFBOARD':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            if self.nav_state == 14:
                self._set_state('TAKEOFF')
                self.get_logger().info(
                    f'Offboard active! Taking off to {self.cruise_alt}m'
                )
            elif elapsed > 0.5:
                self._send_command(176, param1=1.0, param2=6.0)
                if elapsed > 5.0:
                    self.get_logger().error('Failed to switch to offboard')
                    self._set_state('DONE')

        elif self.state == 'TAKEOFF':
            self._send_heartbeat(velocity=True)
            if alt >= self.cruise_alt - 0.5:
                if self.vtol:
                    self._set_state('MC_HOLD_PRE')
                    self.get_logger().info(
                        f'Reached {alt:.1f}m. Holding before FW transition...'
                    )
                else:
                    self._set_state('NAVIGATE')
                    self.wp_index = 0
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

        # --- VTOL transition states ---

        elif self.state == 'MC_HOLD_PRE':
            self._send_heartbeat(position=True)
            self._send_position(self.x, self.y, -self.cruise_alt)
            if elapsed >= 5.0:
                self._set_state('TRANSITION_FW')
                self.get_logger().info('Commanding MC -> FW transition')

        elif self.state == 'TRANSITION_FW':
            self._send_heartbeat(velocity=True)
            # Point toward first waypoint during transition
            wp = self.waypoints[0]
            vx, vy, _ = self._velocity_toward_wp(wp)
            yaw = self._yaw_to_wp(wp)
            self._send_velocity(vx=vx, vy=vy, vz=0.0, yaw=yaw)

            if not self.transition_cmd_sent:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=4.0)
                self.transition_cmd_sent = True

            if elapsed > 2.0 and int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=4.0)

            if self.vehicle_type == VEHICLE_TYPE_FW and not self.in_transition:
                self._set_state('NAVIGATE')
                self.wp_index = 0
                self.get_logger().info(
                    f'FW transition complete! Alt={alt:.1f}m Speed={speed:.1f}m/s. '
                    f'Starting mission with {len(self.waypoints)} waypoints'
                )
            elif elapsed > 15.0:
                self.get_logger().warn(
                    'FW transition timeout. Starting mission in MC mode.'
                )
                self._set_state('NAVIGATE')
                self.wp_index = 0

            if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self.get_logger().info(
                    f'FW transition: {elapsed:.1f}s  alt={alt:.1f}m  '
                    f'speed={speed:.1f}m/s  type={self._type_str()}'
                )

        # --- Navigation (works in both MC and FW) ---

        elif self.state == 'NAVIGATE':
            wp = self.waypoints[self.wp_index]
            horiz_dist = self._horiz_dist_to_wp(wp)
            yaw = self._yaw_to_wp(wp)

            # Position-based navigation for both MC and FW
            # PX4 FW uses NPFG/L1 path following with position setpoints
            self._send_heartbeat(position=True)
            self._send_position(wp[0], wp[1], wp[2], yaw)

            if horiz_dist < self.wp_acceptance_radius:
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
                        if self.vtol and self.vehicle_type == VEHICLE_TYPE_FW:
                            self._set_state('TRANSITION_MC')
                            self.get_logger().info(
                                'All waypoints complete. Transitioning to MC...'
                            )
                        else:
                            self._set_state('RTL')
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
                        f'WP{self.wp_index + 1}: dist={horiz_dist:.1f}m  '
                        f'pos=({self.x:.1f},{self.y:.1f},{alt:.1f}m)  '
                        f'spd={speed:.1f}m/s  type={self._type_str()}'
                    )

        # --- VTOL back-transition ---

        elif self.state == 'TRANSITION_MC':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vx=0.0, vy=0.0, vz=0.0)

            if not self.transition_cmd_sent:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=3.0)
                self.transition_cmd_sent = True

            if elapsed > 2.0 and int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=3.0)

            if self.vehicle_type == VEHICLE_TYPE_MC and not self.in_transition:
                self._set_state('RTL')
                self.get_logger().info(
                    f'Back in MC mode! Alt={alt:.1f}m. Returning home...'
                )
            elif elapsed > 25.0:
                self.get_logger().warn(
                    'MC transition timeout. Attempting RTL anyway.'
                )
                self._set_state('RTL')

            if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self.get_logger().info(
                    f'MC transition: {elapsed:.1f}s  alt={alt:.1f}m  '
                    f'type={self._type_str()}'
                )

        # --- RTL and landing (always in MC) ---

        elif self.state == 'RTL':
            self._send_heartbeat(position=True)
            self._send_position(0.0, 0.0, -self.cruise_alt)
            home_dist = math.sqrt(self.x**2 + self.y**2)
            if home_dist < 2.0:
                self._set_state('DESCEND')
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
                self._set_state('DONE')
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
    parser = argparse.ArgumentParser(description='Offboard canyon mission')
    parser.add_argument('altitude', nargs='?', type=float, default=10.0,
                        help='Cruise altitude in meters (default: 10)')
    parser.add_argument('--vtol', action='store_true',
                        help='Enable VTOL mode: transition to FW for waypoints')
    parser.add_argument('--canyon', action='store_true',
                        help='Full canyon FW traversal (1400m with curve)')
    args = parser.parse_args()

    # VTOL needs higher altitude for transition safety
    altitude = args.altitude
    if args.vtol and altitude < 30.0:
        altitude = 30.0
    # Canyon FW needs more altitude for clearance in canyon
    if args.canyon and altitude < 50.0:
        altitude = 50.0

    rclpy.init()
    node = OffboardMission(altitude, vtol=args.vtol, canyon=args.canyon)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
