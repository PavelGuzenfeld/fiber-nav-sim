#!/usr/bin/env python3
"""GPS-denied flight test for PX4 SITL.

Tests fiber+vision fusion navigation without GPS updates.
Takes off with GPS enabled (for home position), then disables GPS at cruise
altitude to simulate entering a GPS-denied zone (tunnel/underground).

Uses velocity-based navigation in GPS-denied mode because without actual
fiber deployment, there's no position correction source. Position setpoints
cause divergence because the EKF position drifts without absolute reference.

State machine:
    PREFLIGHT -> ARMING -> SET_OFFBOARD -> TAKEOFF (with GPS)
    -> DISABLE_GPS -> STABILIZE -> FLY_OUT (velocity east 15s)
    -> HOLD_OUT (5s) -> FLY_BACK (velocity west 15s)
    -> HOLD_HOME (5s) -> DESCEND -> DONE

Setup:
    Start PX4 with the standard vision airframe (4251):
    docker exec CONTAINER bash -c "
        cp /root/ws/src/fiber-nav-sim/docker/airframes/4251_gz_quadtailsitter_vision \
           /root/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
    "
    Then start PX4:
    rm -f dataman parameters*.bson
    PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4
"""

import argparse
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    EstimatorStatusFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
import time

NAN = float('nan')

PX4_PARAM_CMD = '/root/PX4-Autopilot/build/px4_sitl_default/bin/px4-param'
PX4_ROOTFS = '/root/PX4-Autopilot/build/px4_sitl_default/rootfs'


def set_px4_param(name, value):
    """Set a PX4 parameter at runtime via px4-param CLI."""
    try:
        result = subprocess.run(
            [PX4_PARAM_CMD, 'set', name, str(value)],
            capture_output=True, text=True, timeout=5,
            cwd=PX4_ROOTFS)
        return result.returncode == 0
    except Exception:
        return False


class GpsDeniedTest(Node):
    def __init__(self, altitude=15.0):
        super().__init__('gps_denied_test')
        self.cruise_alt = altitude

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.sub_status = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._status_cb, qos)
        self.sub_lpos = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._lpos_cb, qos)
        self.sub_land = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._land_cb, qos)
        self.sub_ekf = self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self._ekf_cb, qos)

        # State
        self.armed = False
        self.nav_state = 0
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.heading = 0.0
        self.xy_global = False
        self.landed = False
        self.offboard_counter = 0
        self.state = 'PREFLIGHT'
        self.state_start = time.time()

        # EKF status flags
        self.cs_ev_vel = False
        self.cs_ev_pos = False
        self.cs_gnss_pos = False

        self.gps_denied_confirmed = False

        # Velocity mission parameters
        self.cruise_speed = 3.0  # m/s
        self.leg_duration = 15.0  # seconds per leg
        self.hold_duration = 5.0  # seconds hold at each end

        self.timer = self.create_timer(0.05, self._loop)  # 20 Hz

    def _status_cb(self, msg):
        self.armed = msg.arming_state == 2
        self.nav_state = msg.nav_state

    def _lpos_cb(self, msg):
        self.x, self.y, self.z = msg.x, msg.y, msg.z
        self.vx, self.vy, self.vz = msg.vx, msg.vy, msg.vz
        self.heading = msg.heading
        self.xy_global = msg.xy_global

    def _land_cb(self, msg):
        self.landed = msg.landed

    def _ekf_cb(self, msg):
        self.cs_ev_vel = msg.cs_ev_vel
        self.cs_ev_pos = msg.cs_ev_pos
        self.cs_gnss_pos = msg.cs_gnss_pos

    def _send_heartbeat(self, position=False, velocity=False):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
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

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
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

    def _log_status(self, msg=''):
        alt = -self.z
        speed = (self.vx**2 + self.vy**2)**0.5
        return (f'{msg} pos=({self.x:.1f},{self.y:.1f},{alt:.1f}m) '
                f'speed={speed:.1f}m/s ev_vel={self.cs_ev_vel} gps={self.cs_gnss_pos}')

    def _loop(self):
        now = time.time()
        elapsed = now - self.state_start
        alt = -self.z

        # Check GPS-denied status after GPS is disabled
        if not self.gps_denied_confirmed and self.cs_ev_vel:
            if not self.cs_gnss_pos and self.state not in ('PREFLIGHT', 'ARMING',
                    'SET_OFFBOARD', 'TAKEOFF'):
                self.gps_denied_confirmed = True
                self.get_logger().info(
                    '*** GPS-DENIED CONFIRMED: cs_ev_vel=true cs_gnss_pos=false ***')

        if self.state == 'PREFLIGHT':
            self._send_heartbeat(velocity=True)
            self._send_velocity(vz=-1.0)
            self.offboard_counter += 1
            if self.offboard_counter > 200:  # 10s at 20Hz
                self.get_logger().info(
                    f'Preflight done. xy_global={self.xy_global} '
                    f'ev_vel={self.cs_ev_vel} gps={self.cs_gnss_pos}')
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
            elif elapsed > 0.5:
                self._send_command(176, param1=1.0, param2=6.0)
                if elapsed > 5.0:
                    self._set_state('DONE')

        elif self.state == 'TAKEOFF':
            self._send_heartbeat(velocity=True)
            if alt >= self.cruise_alt - 0.5:
                self.get_logger().info(
                    f'Reached {alt:.1f}m. Disabling GPS...')
                self._set_state('DISABLE_GPS')
            else:
                self._send_velocity(vz=-2.0)
                if int(elapsed) % 3 == 0 and elapsed % 3 < 0.06:
                    self.get_logger().info(
                        f'Takeoff: {alt:.1f}m  ev_vel={self.cs_ev_vel} '
                        f'ev_pos={self.cs_ev_pos} gps={self.cs_gnss_pos}')

        elif self.state == 'DISABLE_GPS':
            # Hold velocity zero while disabling GPS
            self._send_heartbeat(velocity=True)
            self._send_velocity()  # vx=vy=vz=0

            if elapsed < 0.1 and not hasattr(self, '_gps_disabled'):
                self._gps_disabled = True
                self.get_logger().info('Disabling GPS fusion (EKF2_GPS_CTRL=0)...')
                ok = set_px4_param('EKF2_GPS_CTRL', 0)
                if ok:
                    self.get_logger().info('EKF2_GPS_CTRL set to 0')
                else:
                    self.get_logger().warn('px4-param failed, continuing...')
            elif not self.cs_gnss_pos:
                self.get_logger().info(self._log_status('GPS disabled.'))
                self._set_state('STABILIZE')
            elif elapsed > 15.0:
                self.get_logger().error('GPS disable timeout')
                self._set_state('DESCEND')

        elif self.state == 'STABILIZE':
            # Hold velocity zero for a few seconds to let EKF settle
            self._send_heartbeat(velocity=True)
            self._send_velocity()
            if elapsed >= 5.0:
                self.get_logger().info(self._log_status('Stabilized.'))
                if self.cs_ev_vel:
                    self._set_state('FLY_OUT')
                    self.get_logger().info(
                        f'Flying East at {self.cruise_speed}m/s for '
                        f'{self.leg_duration}s (~{self.cruise_speed*self.leg_duration:.0f}m)')
                else:
                    self.get_logger().error(
                        'Vision fusion NOT active! Cannot fly GPS-denied.')
                    self._set_state('DESCEND')

        elif self.state == 'FLY_OUT':
            # Fly East (Y+ in NED) at cruise speed
            self._send_heartbeat(velocity=True)
            self._send_velocity(vy=self.cruise_speed)
            if elapsed >= self.leg_duration:
                self.get_logger().info(self._log_status('Outbound leg complete.'))
                self._set_state('HOLD_OUT')
            elif int(elapsed) % 3 == 0 and elapsed % 3 < 0.06:
                self.get_logger().info(self._log_status(
                    f'FLY_OUT: {elapsed:.0f}/{self.leg_duration:.0f}s'))

        elif self.state == 'HOLD_OUT':
            # Hold position (velocity zero)
            self._send_heartbeat(velocity=True)
            self._send_velocity()
            if elapsed >= self.hold_duration:
                self.get_logger().info(self._log_status('Hold complete.'))
                self._set_state('FLY_BACK')
                self.get_logger().info(
                    f'Flying West at {self.cruise_speed}m/s for {self.leg_duration}s')

        elif self.state == 'FLY_BACK':
            # Fly West (Y- in NED) at cruise speed
            self._send_heartbeat(velocity=True)
            self._send_velocity(vy=-self.cruise_speed)
            if elapsed >= self.leg_duration:
                self.get_logger().info(self._log_status('Return leg complete.'))
                self._set_state('HOLD_HOME')
            elif int(elapsed) % 3 == 0 and elapsed % 3 < 0.06:
                self.get_logger().info(self._log_status(
                    f'FLY_BACK: {elapsed:.0f}/{self.leg_duration:.0f}s'))

        elif self.state == 'HOLD_HOME':
            # Hold before descending
            self._send_heartbeat(velocity=True)
            self._send_velocity()
            if elapsed >= self.hold_duration:
                self.get_logger().info(self._log_status('Home hold complete.'))
                self._set_state('DESCEND')

        elif self.state == 'DESCEND':
            self._send_heartbeat(velocity=True)
            # In GPS-denied mode, EKF altitude drifts so alt < 0.5 is unreliable.
            # Use time-based descent: 20s at 1m/s = 20m, enough from 15m cruise alt.
            # Also check landed flag (accelerometer-based, works without GPS).
            if self.landed or elapsed > 20.0:
                self.get_logger().info(
                    f'GPS-denied test complete! '
                    f'gps_denied_confirmed={self.gps_denied_confirmed} '
                    f'landed={self.landed}')
                self._send_command(400, param1=0.0)  # disarm
                self._set_state('DONE')
            else:
                self._send_velocity(vz=1.0)
                if int(elapsed) % 5 == 0 and elapsed % 5 < 0.06:
                    self.get_logger().info(
                        f'Descending: {elapsed:.0f}s  alt_est={alt:.1f}m  '
                        f'landed={self.landed}')

        elif self.state == 'DONE':
            raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(description='GPS-denied flight test')
    parser.add_argument('altitude', nargs='?', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = GpsDeniedTest(args.altitude)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
