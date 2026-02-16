#!/usr/bin/env python3
"""Simulated vision odometry for PX4 SITL GPS-denied flight.

Reads Gazebo ground truth odometry and publishes VehicleOdometry to PX4,
providing velocity feedback for the EKF when GPS is disabled.

In the real system, the fiber_vision_fusion node provides this from
the fiber optic spool sensor + vision direction camera. For SITL testing,
this script uses Gazebo ground truth with optional noise.

Velocity is rotated from Gazebo body frame (FLU) to NED world frame.
Position is sent as NaN (velocity-only fusion).

CRITICAL: Uses PX4's own timestamp (from vehicle_local_position) for
VehicleOdometry messages. PX4 SITL uses simulation time (starts at 0),
while ROS wall clock is POSIX epoch — mismatched timestamps cause PX4's
EKF to reject all external vision data.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition


def quat_rotate(q, v):
    """Rotate vector v by quaternion q = [x, y, z, w]."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # q * v * q_conj using Hamilton product
    t = [
        2.0 * (qy * vz - qz * vy),
        2.0 * (qz * vx - qx * vz),
        2.0 * (qx * vy - qy * vx),
    ]
    return [
        vx + qw * t[0] + qy * t[2] - qz * t[1],
        vy + qw * t[1] + qz * t[0] - qx * t[2],
        vz + qw * t[2] + qx * t[1] - qy * t[0],
    ]


class SimVisionOdometry(Node):
    def __init__(self):
        super().__init__('sim_vision_odometry')

        self.declare_parameter('velocity_variance', 0.01)
        self.declare_parameter('velocity_noise_std', 0.0)

        self.vel_var = self.get_parameter('velocity_variance').value
        self.noise_std = self.get_parameter('velocity_noise_std').value

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

        # Subscribe to Gazebo ground truth odometry
        self.sub = self.create_subscription(
            Odometry,
            '/model/quadtailsitter/odometry',
            self._odom_cb,
            qos_gz,
        )

        # Subscribe to PX4 local position for timestamp synchronization.
        # PX4 SITL uses simulation time (starts at 0), while ROS uses wall clock.
        # We must use PX4's timestamp or the EKF rejects our messages.
        self._px4_timestamp_us = 0
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._px4_time_cb,
            qos_px4,
        )

        self.pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_px4
        )

        self._log_count = 0

        self.get_logger().info(
            f'Sim vision odometry: vel_var={self.vel_var}, '
            f'noise_std={self.noise_std}'
        )

    def _px4_time_cb(self, msg: VehicleLocalPosition):
        """Track PX4's internal timestamp for synchronization."""
        self._px4_timestamp_us = msg.timestamp

    def _odom_cb(self, msg: Odometry):
        # Don't publish until we have PX4's time reference
        if self._px4_timestamp_us == 0:
            return

        # Body-frame velocity from Gazebo (FLU: X-forward, Y-left, Z-up)
        vx_body = msg.twist.twist.linear.x
        vy_body = msg.twist.twist.linear.y
        vz_body = msg.twist.twist.linear.z

        # Orientation quaternion from Gazebo (ENU world frame)
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        # Rotate body velocity to ENU world frame
        v_enu = quat_rotate(quat, [vx_body, vy_body, vz_body])

        # Convert ENU to NED
        # Gazebo world frame: X-axis, Y-axis, Z-up (ENU)
        # NED: X=North, Y=East, Z=Down
        vn = v_enu[1]   # North = ENU_y
        ve = v_enu[0]   # East = ENU_x
        vd = -v_enu[2]  # Down = -ENU_z

        # Periodic diagnostic log (every 100 messages ≈ 2s at 50Hz)
        self._log_count += 1
        if self._log_count % 100 == 0:
            spd_h = math.sqrt(vn*vn + ve*ve)
            hdg = math.degrees(math.atan2(ve, vn))
            body_spd = math.sqrt(vx_body**2 + vy_body**2 + vz_body**2)
            self.get_logger().info(
                f'body=({vx_body:.1f},{vy_body:.1f},{vz_body:.1f}) '
                f'ned=({vn:.1f},{ve:.1f},{vd:.1f}) '
                f'spd={spd_h:.1f} hdg={hdg:.0f} body_spd={body_spd:.1f} '
                f'px4_t={self._px4_timestamp_us}'
            )

        # Add optional noise
        if self.noise_std > 0:
            vn += np.random.normal(0, self.noise_std)
            ve += np.random.normal(0, self.noise_std)
            vd += np.random.normal(0, self.noise_std)

        # Build VehicleOdometry message
        odom = VehicleOdometry()

        # Use PX4's timestamp — critical for EKF acceptance in SITL lockstep mode
        odom.timestamp = self._px4_timestamp_us
        odom.timestamp_sample = self._px4_timestamp_us

        # Velocity in NED frame
        odom.velocity = [float(vn), float(ve), float(vd)]
        odom.velocity_variance = [
            float(self.vel_var),
            float(self.vel_var),
            float(self.vel_var),
        ]

        # Position: NaN = not provided (velocity-only fusion)
        nan = float('nan')
        odom.position = [nan, nan, nan]
        odom.position_variance = [0.0, 0.0, 0.0]

        # Attitude: NaN = not provided
        odom.q = [nan, nan, nan, nan]

        # Angular velocity: NaN = not provided
        odom.angular_velocity = [nan, nan, nan]

        # Frame references
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # Quality (0-255, required for PX4 EKF to accept)
        odom.quality = 100

        self.pub.publish(odom)


def main():
    rclpy.init()
    node = SimVisionOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
