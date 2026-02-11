#!/usr/bin/env python3
# Copyright 2024-2026 Fiber Navigation. Licensed under Apache-2.0.
"""Terrain-following flight script for PX4 SITL.

Flies east at constant speed while maintaining a target AGL (above ground
level) using the sim_distance_sensor feedback (dist_bottom from
VehicleLocalPosition). Terrain height varies over the 6km terrain_world map.

The script uses velocity control with:
  - Constant eastward velocity (PX4 NED y+)
  - P-controller on AGL error for vertical velocity

It also queries the terrain_gis_node for terrain height display/comparison.

Usage:
    python3 offboard_terrain_follow.py [target_agl] [--distance DIST] [--speed SPEED]
    python3 offboard_terrain_follow.py 30          # 30m AGL, default 2000m east
    python3 offboard_terrain_follow.py 30 --distance 3000 --speed 20
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

NAN = float('nan')


class TerrainFollower(Node):
    def __init__(self, target_agl=30.0, distance=2000.0, speed=15.0):
        super().__init__('terrain_follower')
        self.target_agl = target_agl
        self.total_distance = distance
        self.cruise_speed = speed

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # PX4 publishers
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos
        )

        # PX4 subscribers
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

        # Terrain query (optional — for logging/display only)
        self.pub_terrain_query = self.create_publisher(
            Point, '/terrain/query', 10
        )
        self.sub_terrain_height = self.create_subscription(
            Float64, '/terrain/height', self._terrain_height_cb, 10
        )
        self.terrain_gz_z = None  # Latest terrain Gazebo Z from GIS node

        # State
        self.armed = False
        self.nav_state = 0
        self.x = 0.0  # NED North
        self.y = 0.0  # NED East
        self.z = 0.0  # NED Down
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.heading = 0.0
        self.dist_bottom = 0.0
        self.dist_bottom_valid = False
        self.landed = False
        self.offboard_counter = 0
        self.state = 'PREFLIGHT'
        self.state_start = time.time()
        self.log_counter = 0

        # Altitude controller gains
        self.kp_alt = 0.8  # P gain for AGL error -> vz
        self.vz_max = 3.0  # max vertical speed m/s

        self.timer = self.create_timer(0.05, self._loop)  # 20 Hz

    def _status_cb(self, msg):
        self.armed = msg.arming_state == 2
        self.nav_state = msg.nav_state

    def _lpos_cb(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
        self.heading = msg.heading
        self.dist_bottom = msg.dist_bottom
        self.dist_bottom_valid = msg.dist_bottom_valid

    def _land_cb(self, msg):
        self.landed = msg.landed

    def _terrain_height_cb(self, msg):
        self.terrain_gz_z = msg.data

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

    def _set_state(self, new_state):
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self.state_start = time.time()

    def _query_terrain(self):
        """Query terrain height at current position (ENU coords)."""
        # PX4 NED -> Gazebo ENU: enu_x = ned_y, enu_y = ned_x
        pt = Point()
        pt.x = self.y   # NED East -> ENU East
        pt.y = self.x   # NED North -> ENU North
        pt.z = 0.0
        self.pub_terrain_query.publish(pt)

    def _compute_vz(self):
        """P-controller: maintain target AGL using dist_bottom."""
        if not self.dist_bottom_valid:
            # No AGL data — hold current altitude
            return 0.0
        agl_error = self.dist_bottom - self.target_agl  # positive = too high
        vz = agl_error * self.kp_alt  # NED: positive vz = descend
        return max(-self.vz_max, min(self.vz_max, vz))

    def _loop(self):
        now = time.time()
        elapsed = now - self.state_start
        alt = -self.z  # altitude above home (NED)
        speed = math.sqrt(self.vx**2 + self.vy**2)

        if self.state == 'PREFLIGHT':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            self.offboard_counter += 1
            if self.offboard_counter > 400:  # 20s warmup
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
                    f'Offboard active! Taking off to {self.target_agl}m AGL'
                )
            elif elapsed > 0.5:
                self._send_command(176, param1=1.0, param2=6.0)
                if elapsed > 5.0:
                    self.get_logger().error('Failed to switch to offboard')
                    self._set_state('DONE')

        elif self.state == 'TAKEOFF':
            self._send_heartbeat(velocity=True)
            # Climb until dist_bottom (AGL) reaches target
            agl = self.dist_bottom if self.dist_bottom_valid else alt
            if agl >= self.target_agl - 1.0:
                self._set_state('TERRAIN_FOLLOW')
                self.get_logger().info(
                    f'Reached {agl:.1f}m AGL. Starting terrain-following '
                    f'flight east at {self.cruise_speed}m/s for '
                    f'{self.total_distance}m'
                )
            else:
                self._send_velocity(vz=-2.0)
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                    self.get_logger().info(
                        f'Takeoff: {agl:.1f}m AGL / {self.target_agl}m  '
                        f'alt={alt:.1f}m  vz={self.vz:.2f}'
                    )

        elif self.state == 'TERRAIN_FOLLOW':
            self._send_heartbeat(velocity=True)
            # Fly east (NED y+) at cruise speed, adjust altitude for terrain
            vz = self._compute_vz()
            # Yaw east: pi/2 in NED frame
            self._send_velocity(vx=0.0, vy=self.cruise_speed, vz=vz,
                                yaw=math.pi / 2.0)

            # Query terrain height periodically (every 1s)
            self.log_counter += 1
            if self.log_counter % 20 == 0:
                self._query_terrain()

            # Distance traveled east (NED y)
            east_dist = self.y
            agl = self.dist_bottom if self.dist_bottom_valid else alt

            if east_dist >= self.total_distance:
                self._set_state('RETURN')
                self.get_logger().info(
                    f'Reached {east_dist:.0f}m east. Returning home...'
                )
            elif int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                terrain_str = (f'terrain_gz={self.terrain_gz_z:.1f}m'
                               if self.terrain_gz_z is not None else
                               'terrain=N/A')
                self.get_logger().info(
                    f'TF: east={east_dist:.0f}m  agl={agl:.1f}m/'
                    f'{self.target_agl:.0f}m  alt={alt:.1f}m  '
                    f'spd={speed:.1f}m/s  vz_cmd={vz:.2f}  '
                    f'{terrain_str}'
                )

        elif self.state == 'RETURN':
            self._send_heartbeat(velocity=True)
            # Fly west (NED y-) back to home, maintain AGL
            vz = self._compute_vz()
            self._send_velocity(vx=0.0, vy=-self.cruise_speed, vz=vz,
                                yaw=-math.pi / 2.0)

            self.log_counter += 1
            if self.log_counter % 20 == 0:
                self._query_terrain()

            home_dist = math.sqrt(self.x**2 + self.y**2)
            east_dist = self.y
            agl = self.dist_bottom if self.dist_bottom_valid else alt

            if east_dist <= 10.0:
                self._set_state('RTL')
                self.get_logger().info(
                    f'Near home east axis. Switching to position RTL.'
                )
            elif elapsed > self.total_distance / self.cruise_speed + 60.0:
                self.get_logger().warn('Return timeout. Descending...')
                self._set_state('DESCEND')
            elif int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                terrain_str = (f'terrain_gz={self.terrain_gz_z:.1f}m'
                               if self.terrain_gz_z is not None else
                               'terrain=N/A')
                self.get_logger().info(
                    f'RTN: east={east_dist:.0f}m  agl={agl:.1f}m/'
                    f'{self.target_agl:.0f}m  alt={alt:.1f}m  '
                    f'spd={speed:.1f}m/s  {terrain_str}'
                )

        elif self.state == 'RTL':
            self._send_heartbeat(position=True)
            self._send_position(0.0, 0.0, -self.target_agl)
            home_dist = math.sqrt(self.x**2 + self.y**2)
            if home_dist < 5.0:
                self._set_state('DESCEND')
                self.get_logger().info('Over home. Descending...')
            elif elapsed > 60.0:
                self.get_logger().warn('RTL timeout. Descending...')
                self._set_state('DESCEND')
            elif int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self.get_logger().info(
                    f'RTL: {home_dist:.1f}m from home  '
                    f'pos=({self.x:.1f},{self.y:.1f},{alt:.1f}m)'
                )

        elif self.state == 'DESCEND':
            self._send_heartbeat(velocity=True)
            descent_timeout = self.target_agl + 30.0
            if alt < 0.5 or self.landed or elapsed > descent_timeout:
                self.get_logger().info(
                    f'Landed! alt={alt:.1f}m landed={self.landed} '
                    f'elapsed={elapsed:.0f}s'
                )
                self._send_command(400, param1=0.0)  # DISARM
                self._set_state('DONE')
            else:
                self._send_velocity(vz=1.0)
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                    self.get_logger().info(
                        f'Descending: {alt:.1f}m  vz={self.vz:.2f}'
                    )

        elif self.state == 'DONE':
            self.get_logger().info('Done.')
            raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(
        description='Terrain-following flight for PX4 SITL'
    )
    parser.add_argument(
        'agl', nargs='?', type=float, default=30.0,
        help='Target AGL in meters (default: 30)'
    )
    parser.add_argument(
        '--distance', type=float, default=2000.0,
        help='Total distance east in meters (default: 2000)'
    )
    parser.add_argument(
        '--speed', type=float, default=15.0,
        help='Cruise speed in m/s (default: 15)'
    )
    args = parser.parse_args()

    rclpy.init()
    node = TerrainFollower(
        target_agl=args.agl,
        distance=args.distance,
        speed=args.speed,
    )
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
