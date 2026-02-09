#!/usr/bin/env python3
"""VTOL transition test for PX4 SITL quadtailsitter.

State machine:
  PREFLIGHT -> ARM -> SET_OFFBOARD -> TAKEOFF(30m) -> MC_HOLD(5s)
  -> TRANSITION_FW -> FW_CRUISE(30s east along canyon)
  -> TRANSITION_MC -> MC_HOLD_2(5s) -> DESCEND -> DONE

Transition commands use VehicleCommand 3000:
  param1=4.0 -> MC to FW
  param1=3.0 -> FW to MC

Monitor VehicleStatus.vehicle_type:
  1 = multicopter (MC)
  2 = fixed-wing (FW)
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

# Transition command ID (MAV_CMD_DO_VTOL_TRANSITION)
CMD_DO_VTOL_TRANSITION = 3000
VEHICLE_TYPE_MC = 1
VEHICLE_TYPE_FW = 2


class OffboardTransitionTest(Node):
    def __init__(self, takeoff_alt=30.0):
        super().__init__('offboard_transition_test')
        self.takeoff_alt = takeoff_alt

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

        # Vehicle state
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

        # State machine
        self.offboard_counter = 0
        self.state = 'PREFLIGHT'
        self.state_start = time.time()
        self.transition_cmd_sent = False

        # FW cruise: fly east along canyon (Y+ in NED)
        self.fw_cruise_speed = 15.0  # m/s
        self.fw_cruise_duration = 30.0  # seconds

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
            elif elapsed > 0.5:
                self._send_command(176, param1=1.0, param2=6.0)
                if elapsed > 5.0:
                    self.get_logger().error('Failed to switch to offboard')
                    self._set_state('DONE')

        elif self.state == 'TAKEOFF':
            self._send_heartbeat(velocity=True)
            if alt >= self.takeoff_alt - 0.5:
                self._set_state('MC_HOLD')
                self.get_logger().info(
                    f'Reached {alt:.1f}m. Holding in MC before transition...'
                )
            else:
                self._send_velocity(vz=-2.0)
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                    self.get_logger().info(
                        f'Takeoff: {alt:.1f}m / {self.takeoff_alt}m  '
                        f'vz={self.vz:.2f}  type={self._type_str()}'
                    )

        elif self.state == 'MC_HOLD':
            # Hold position in MC mode for 5s before transition
            self._send_heartbeat(position=True)
            self._send_position(self.x, self.y, -self.takeoff_alt)
            if elapsed >= 5.0:
                self._set_state('TRANSITION_FW')
                self.get_logger().info('Commanding MC -> FW transition')

        elif self.state == 'TRANSITION_FW':
            # Must keep sending offboard heartbeat + setpoint during transition
            self._send_heartbeat(velocity=True)
            # Command forward velocity (East along canyon) during transition
            # Yaw east (pi/2) so the tailsitter pitches forward toward east
            self._send_velocity(vx=0.0, vy=self.fw_cruise_speed, vz=0.0,
                                yaw=math.pi / 2)

            if not self.transition_cmd_sent:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=4.0)
                self.transition_cmd_sent = True

            # Resend transition command every 2s in case first was missed
            if elapsed > 2.0 and int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=4.0)

            if self.vehicle_type == VEHICLE_TYPE_FW and not self.in_transition:
                self._set_state('FW_CRUISE')
                self.get_logger().info(
                    f'FW transition complete! Alt={alt:.1f}m Speed={speed:.1f}m/s'
                )
            elif elapsed > 15.0:
                self.get_logger().warn(
                    'Transition timeout (15s). Aborting to MC.'
                )
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=3.0)
                self._set_state('MC_HOLD_2')

            if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self.get_logger().info(
                    f'FW transition: {elapsed:.1f}s  alt={alt:.1f}m  '
                    f'speed={speed:.1f}m/s  type={self._type_str()}'
                )

        elif self.state == 'FW_CRUISE':
            # Fly east along canyon in FW mode
            self._send_heartbeat(velocity=True)
            self._send_velocity(vx=0.0, vy=self.fw_cruise_speed, vz=0.0,
                                yaw=math.pi / 2)

            if elapsed >= self.fw_cruise_duration:
                self._set_state('TRANSITION_MC')
                self.get_logger().info(
                    f'FW cruise complete. Flew {self.y:.0f}m east. '
                    f'Transitioning back to MC...'
                )

            if int(elapsed) % 5 == 0 and elapsed % 5 < 0.06:
                self.get_logger().info(
                    f'FW cruise: {elapsed:.0f}/{self.fw_cruise_duration:.0f}s  '
                    f'alt={alt:.1f}m  speed={speed:.1f}m/s  '
                    f'pos=({self.x:.1f},{self.y:.1f})'
                )

        elif self.state == 'TRANSITION_MC':
            # Transition back to MC
            self._send_heartbeat(velocity=True)
            # Slow down and hold altitude
            self._send_velocity(vx=0.0, vy=0.0, vz=0.0)

            if not self.transition_cmd_sent:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=3.0)
                self.transition_cmd_sent = True

            if elapsed > 2.0 and int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self._send_command(CMD_DO_VTOL_TRANSITION, param1=3.0)

            if self.vehicle_type == VEHICLE_TYPE_MC and not self.in_transition:
                self._set_state('MC_HOLD_2')
                self.get_logger().info(
                    f'Back in MC mode! Alt={alt:.1f}m'
                )
            elif elapsed > 15.0:
                self.get_logger().warn(
                    'MC transition timeout (15s). Attempting descent anyway.'
                )
                self._set_state('MC_HOLD_2')

            if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                self.get_logger().info(
                    f'MC transition: {elapsed:.1f}s  alt={alt:.1f}m  '
                    f'speed={speed:.1f}m/s  type={self._type_str()}'
                )

        elif self.state == 'MC_HOLD_2':
            # Hold in MC for 5s after back-transition
            self._send_heartbeat(position=True)
            self._send_position(self.x, self.y, -self.takeoff_alt)
            if elapsed >= 5.0:
                self._set_state('DESCEND')
                self.get_logger().info('MC hold complete. Descending...')

        elif self.state == 'DESCEND':
            self._send_heartbeat(velocity=True)
            if alt < 0.5 or self.landed:
                self.get_logger().info(
                    'Landed! Full transition cycle complete.'
                )
                self._send_command(400, param1=0.0)  # DISARM
                self._set_state('DONE')
            else:
                self._send_velocity(vz=1.0)  # descend at 1 m/s
                if int(elapsed) % 2 == 0 and elapsed % 2 < 0.06:
                    self.get_logger().info(
                        f'Descending: {alt:.1f}m  vz={self.vz:.2f}  '
                        f'type={self._type_str()}'
                    )

        elif self.state == 'DONE':
            self.get_logger().info('Done.')
            raise SystemExit(0)


def main():
    rclpy.init()
    alt = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    node = OffboardTransitionTest(alt)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
