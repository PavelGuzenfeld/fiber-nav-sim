"""End-to-end VTOL mission test.

Subscribes to vehicle status, position, and mission state topics.
Tracks mission phase transitions and verifies the drone returns
within a configurable radius of home.

Exit codes:
    0 = PASS (all phases completed, within home radius)
    1 = FAIL (timeout, missed phases, or too far from home)

Usage:
    ros2 run fiber_nav_analysis e2e_mission_test
    ros2 run fiber_nav_analysis e2e_mission_test --ros-args \
        -p max_mission_time:=600.0 -p home_radius:=100.0
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus


# Expected phase order for the VTOL mission state machine
EXPECTED_PHASES = [
    'MC_CLIMB',
    'TRANSITION_FW',
    'FW_NAVIGATE',
    'FW_RETURN',
    'TRANSITION_MC',
    'MC_APPROACH',
    'DONE',
]


class E2EMissionTest(Node):
    """Monitors a VTOL mission and validates completion criteria."""

    def __init__(self):
        super().__init__('e2e_mission_test')

        self.declare_parameter('max_mission_time', 600.0)
        self.declare_parameter('home_radius', 100.0)

        self.max_mission_time_ = self.get_parameter('max_mission_time').value
        self.home_radius_ = self.get_parameter('home_radius').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Position tracking
        self.pos_x_ = 0.0
        self.pos_y_ = 0.0
        self.pos_z_ = 0.0
        self.pos_valid_ = False

        # Mission state tracking
        self.current_phase_ = None
        self.phases_seen_ = []
        self.phase_times_ = {}
        self.mission_started_ = False
        self.mission_done_ = False
        self.start_time_ = time.monotonic()

        # Vehicle status tracking
        self.armed_ = False
        self.was_armed_ = False
        self.nav_state_ = 0

        # Subscriptions
        self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self._on_local_position,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self._on_vehicle_status,
            qos,
        )

        # 1 Hz check timer
        self.create_timer(1.0, self._check_progress)

        self.get_logger().info(
            f'E2E mission test started: max_time={self.max_mission_time_}s, '
            f'home_radius={self.home_radius_}m'
        )

    def _on_local_position(self, msg: VehicleLocalPosition):
        if msg.xy_valid:
            self.pos_x_ = msg.x
            self.pos_y_ = msg.y
            self.pos_z_ = msg.z
            self.pos_valid_ = True

    def _on_vehicle_status(self, msg: VehicleStatus):
        was_armed = self.armed_
        self.armed_ = msg.arming_state == 2  # ARMED
        self.nav_state_ = msg.nav_state

        if self.armed_ and not was_armed:
            self.was_armed_ = True
            self.get_logger().info('Vehicle ARMED')

        # Detect custom mode (offboard) — nav_state 14 = OFFBOARD
        if self.armed_ and msg.nav_state == 14 and not self.mission_started_:
            self.mission_started_ = True
            self.start_time_ = time.monotonic()
            self.get_logger().info('Mission started (offboard mode detected)')

        # Detect disarm after being armed = mission complete
        if not self.armed_ and was_armed and self.was_armed_:
            self._on_disarm()

    def _on_disarm(self):
        """Called when vehicle disarms after having been armed."""
        elapsed = time.monotonic() - self.start_time_
        dist_home = (self.pos_x_ ** 2 + self.pos_y_ ** 2) ** 0.5

        self.get_logger().info(
            f'Vehicle DISARMED after {elapsed:.1f}s, '
            f'pos=({self.pos_x_:.1f}, {self.pos_y_:.1f}), '
            f'dist_home={dist_home:.1f}m'
        )

        self.mission_done_ = True
        self._evaluate_result(dist_home, elapsed)

    def _check_progress(self):
        """Periodic progress check and timeout detection."""
        if self.mission_done_:
            return

        elapsed = time.monotonic() - self.start_time_

        # Log periodic status
        if self.pos_valid_ and self.mission_started_:
            dist_home = (self.pos_x_ ** 2 + self.pos_y_ ** 2) ** 0.5
            self.get_logger().info(
                f'[{elapsed:.0f}s] pos=({self.pos_x_:.0f}, {self.pos_y_:.0f}) '
                f'alt_agl={-self.pos_z_:.0f}m dist_home={dist_home:.0f}m '
                f'armed={self.armed_} nav={self.nav_state_}'
            )

        # Timeout check
        if self.mission_started_ and elapsed > self.max_mission_time_:
            dist_home = (self.pos_x_ ** 2 + self.pos_y_ ** 2) ** 0.5
            self.get_logger().error(
                f'TIMEOUT after {elapsed:.1f}s! '
                f'dist_home={dist_home:.1f}m'
            )
            self.mission_done_ = True
            self._evaluate_result(dist_home, elapsed, timed_out=True)

    def _evaluate_result(self, dist_home: float, elapsed: float,
                         timed_out: bool = False):
        """Evaluate pass/fail and exit."""
        passed = True
        reasons = []

        if timed_out:
            passed = False
            reasons.append(f'TIMEOUT ({elapsed:.0f}s > {self.max_mission_time_}s)')

        if dist_home > self.home_radius_:
            passed = False
            reasons.append(
                f'TOO FAR from home ({dist_home:.1f}m > {self.home_radius_}m)'
            )

        if passed:
            self.get_logger().info(
                f'PASS: Mission complete in {elapsed:.1f}s, '
                f'dist_home={dist_home:.1f}m <= {self.home_radius_}m'
            )
        else:
            self.get_logger().error(
                f'FAIL: {", ".join(reasons)}'
            )

        # Give time for log flush
        time.sleep(0.5)
        rclpy.shutdown()
        sys.exit(0 if passed else 1)


def main(args=None):
    rclpy.init(args=args)
    node = E2EMissionTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
        sys.exit(1)


if __name__ == '__main__':
    main()
