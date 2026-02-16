#!/usr/bin/env python3
"""Gimbal integration test — validates gimbal behavior across VTOL flight phases.

Records gimbal commands, actual joint positions, saturation feedback, and vehicle
attitude during a VTOL mission. Validates per-phase criteria and outputs structured
results.

Usage (inside simulation container, alongside a VTOL mission):
    python3 test_gimbal_integration.py [--timeout 600] [--output /path/to/results.json]

The test subscribes passively — it does NOT control the vehicle. Start a VTOL mission
separately (e.g., via MISSION=vtol_gps_denied env var or manual script).
"""

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from enum import IntEnum, auto
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# Try importing px4_msgs for phase detection; fall back to odometry-only
try:
    from px4_msgs.msg import VehicleStatus
    HAS_PX4_MSGS = True
except ImportError:
    HAS_PX4_MSGS = False


class FlightPhase(IntEnum):
    """Detected flight phases based on vehicle behavior."""
    UNKNOWN = 0
    MC_CLIMB = auto()
    TRANSITION_FW = auto()
    FW_NAVIGATE = auto()
    FW_RETURN = auto()
    TRANSITION_MC = auto()
    MC_APPROACH = auto()
    LANDED = auto()


@dataclass
class Sample:
    """Single timestep of gimbal + vehicle data."""
    t: float               # seconds since start
    phase: int             # FlightPhase
    # Gimbal commands
    yaw_cmd: float = 0.0
    pitch_cmd: float = 0.0
    # Gimbal actual (from joint state publisher)
    yaw_actual: float = 0.0
    pitch_actual: float = 0.0
    # Saturation feedback
    yaw_sat: float = 0.0
    pitch_sat: float = 0.0
    # Vehicle attitude (gravity vector components)
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    # Vehicle velocity (for phase detection)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


@dataclass
class PhaseResult:
    """Validation result for a single flight phase."""
    phase: str
    passed: bool
    criteria: dict = field(default_factory=dict)
    sample_count: int = 0
    duration_s: float = 0.0


class GimbalRecorder(Node):
    """Records gimbal and vehicle data during flight."""

    def __init__(self, timeout_s: float = 600.0,
                 world_name: str = 'terrain_world'):
        super().__init__('gimbal_integration_test')
        self.timeout_s = timeout_s
        self.world_name = world_name
        self.start_time = time.monotonic()
        self.samples: list[Sample] = []
        self.done = False

        # Latest values (updated by callbacks)
        self._yaw_cmd = 0.0
        self._pitch_cmd = 0.0
        self._yaw_actual = 0.0
        self._pitch_actual = 0.0
        self._yaw_sat = 0.0
        self._pitch_sat = 0.0
        self._gx = 0.0
        self._gy = 0.0
        self._gz = -1.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._vehicle_type = 1  # MC
        self._arming_state = 1  # disarmed
        self._nav_state = 0

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Gimbal command subscriptions
        self.create_subscription(
            Float64, '/gimbal/cmd_pos', self._on_yaw_cmd, qos_best_effort)
        self.create_subscription(
            Float64, '/gimbal/pitch_cmd_pos', self._on_pitch_cmd, qos_best_effort)

        # Gimbal saturation subscriptions
        self.create_subscription(
            Float64, '/gimbal/saturation', self._on_yaw_sat, qos_best_effort)
        self.create_subscription(
            Float64, '/gimbal/pitch_saturation', self._on_pitch_sat, qos_best_effort)

        # Joint state feedback (actual gimbal positions from Gazebo)
        # The ros_gz_bridge publishes to the full Gazebo topic path
        joint_state_topic = (
            f'/world/{world_name}/model/quadtailsitter/joint_state')
        self.create_subscription(
            JointState, joint_state_topic, self._on_joint_state,
            qos_best_effort)
        self.get_logger().info(f'Subscribing to joint state: {joint_state_topic}')

        # Vehicle odometry (for gravity vector + velocity)
        self.create_subscription(
            Odometry, '/model/quadtailsitter/odometry',
            self._on_odometry, qos_best_effort)

        # PX4 vehicle status (for phase detection)
        if HAS_PX4_MSGS:
            qos_px4 = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                VehicleStatus, '/fmu/out/vehicle_status_v1',
                self._on_vehicle_status, qos_px4)

        # Sample at 10 Hz
        self.create_timer(0.1, self._sample_tick)

        self.get_logger().info(
            f'Gimbal integration test started (timeout={timeout_s}s)')

    def _on_yaw_cmd(self, msg):
        self._yaw_cmd = msg.data

    def _on_pitch_cmd(self, msg):
        self._pitch_cmd = msg.data

    def _on_yaw_sat(self, msg):
        self._yaw_sat = msg.data

    def _on_pitch_sat(self, msg):
        self._pitch_sat = msg.data

    def _on_joint_state(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if 'roll' in name or 'yaw' in name:
                    self._yaw_actual = msg.position[i]
                elif 'pitch' in name:
                    self._pitch_actual = msg.position[i]

    def _on_odometry(self, msg):
        q = msg.pose.pose.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        # Gravity in body frame (same math as gimbal_math.hpp)
        self._gx = 2.0 * (w * y - x * z)
        self._gy = -2.0 * (y * z + w * x)
        self._gz = 2.0 * (x * x + y * y) - 1.0
        # Velocity in body frame
        v = msg.twist.twist.linear
        self._vx = v.x
        self._vy = v.y
        self._vz = v.z

    def _on_vehicle_status(self, msg):
        self._vehicle_type = msg.vehicle_type
        self._arming_state = msg.arming_state
        self._nav_state = msg.nav_state

    def _detect_phase(self) -> FlightPhase:
        """Detect flight phase from vehicle state."""
        speed = math.sqrt(self._vx**2 + self._vy**2 + self._vz**2)

        # If disarmed or very slow on ground
        if self._arming_state != 2 and speed < 1.0:
            return FlightPhase.LANDED

        # Use gravity vector to detect orientation
        # gx > 0.5 → mostly FW (belly down)
        # gx < 0.3 → mostly MC (belly horizontal/forward)
        if self._gx < 0.3:
            # MC mode — Gazebo odometry twist is body-frame ENU,
            # so climbing in hover = positive vz (body Z ≈ world Z)
            if self._vz > 0.5:
                return FlightPhase.MC_CLIMB
            return FlightPhase.MC_APPROACH
        elif self._gx < 0.7:
            # Transitioning
            if speed < 10.0:
                return FlightPhase.TRANSITION_FW
            return FlightPhase.TRANSITION_MC
        else:
            # FW mode
            return FlightPhase.FW_NAVIGATE

    def _sample_tick(self):
        elapsed = time.monotonic() - self.start_time
        if elapsed > self.timeout_s:
            self.get_logger().warn(f'Timeout reached ({self.timeout_s}s)')
            self.done = True
            return

        phase = self._detect_phase()
        sample = Sample(
            t=elapsed,
            phase=int(phase),
            yaw_cmd=self._yaw_cmd,
            pitch_cmd=self._pitch_cmd,
            yaw_actual=self._yaw_actual,
            pitch_actual=self._pitch_actual,
            yaw_sat=self._yaw_sat,
            pitch_sat=self._pitch_sat,
            gx=self._gx,
            gy=self._gy,
            gz=self._gz,
            vx=self._vx,
            vy=self._vy,
            vz=self._vz,
        )
        self.samples.append(sample)

        # Check if mission completed (returned to landed after being airborne)
        if (len(self.samples) > 100
                and phase == FlightPhase.LANDED
                and any(s.phase == int(FlightPhase.FW_NAVIGATE)
                        for s in self.samples)):
            self.get_logger().info('Mission completed — landed after FW flight')
            self.done = True


class PhaseValidator:
    """Validates gimbal behavior criteria per flight phase."""

    # Thresholds
    MC_CMD_MAX = 0.05          # rad — gimbal should be near zero in MC
    TRANSITION_SPIKE_MAX = 0.3  # rad/sample — no command spikes during transitions
    FW_NADIR_ERROR_MAX = 5.0    # degrees — RMS nadir error in FW cruise
    FW_TURN_ERROR_MAX = 15.0    # degrees — max error during turns
    SETTLE_TIME = 5.0           # seconds — gimbal settle time after transition
    SETTLE_THRESHOLD = 0.05     # rad — considered settled

    def __init__(self, samples: list[Sample]):
        self.samples = samples
        self.results: list[PhaseResult] = []

    def validate_all(self) -> list[PhaseResult]:
        """Run all phase validations."""
        self.results = []

        # Group samples by phase
        phases = {}
        for s in self.samples:
            phase = FlightPhase(s.phase)
            if phase not in phases:
                phases[phase] = []
            phases[phase].append(s)

        # Validate each phase that has data
        if FlightPhase.MC_CLIMB in phases:
            self.results.append(self._validate_mc(
                phases[FlightPhase.MC_CLIMB], 'MC_CLIMB'))

        if FlightPhase.TRANSITION_FW in phases:
            self.results.append(self._validate_transition(
                phases[FlightPhase.TRANSITION_FW], 'TRANSITION_FW'))

        if FlightPhase.FW_NAVIGATE in phases:
            self.results.append(self._validate_fw_cruise(
                phases[FlightPhase.FW_NAVIGATE], 'FW_NAVIGATE'))

        if FlightPhase.TRANSITION_MC in phases:
            self.results.append(self._validate_transition(
                phases[FlightPhase.TRANSITION_MC], 'TRANSITION_MC'))

        if FlightPhase.MC_APPROACH in phases:
            self.results.append(self._validate_mc(
                phases[FlightPhase.MC_APPROACH], 'MC_APPROACH'))

        # Overall gimbal tracking accuracy (all FW samples)
        fw_samples = phases.get(FlightPhase.FW_NAVIGATE, [])
        if fw_samples:
            self.results.append(self._validate_tracking(fw_samples))

        return self.results

    def _validate_mc(self, samples: list[Sample], phase_name: str) -> PhaseResult:
        """MC phases: gimbal commands should be near zero."""
        yaw_max = max(abs(s.yaw_cmd) for s in samples)
        pitch_max = max(abs(s.pitch_cmd) for s in samples)
        yaw_ok = yaw_max < self.MC_CMD_MAX
        pitch_ok = pitch_max < self.MC_CMD_MAX
        duration = samples[-1].t - samples[0].t if len(samples) > 1 else 0.0

        return PhaseResult(
            phase=phase_name,
            passed=yaw_ok and pitch_ok,
            criteria={
                'yaw_max_cmd': round(yaw_max, 4),
                'pitch_max_cmd': round(pitch_max, 4),
                'yaw_ok': yaw_ok,
                'pitch_ok': pitch_ok,
                'threshold': self.MC_CMD_MAX,
            },
            sample_count=len(samples),
            duration_s=round(duration, 1),
        )

    def _validate_transition(self, samples: list[Sample],
                             phase_name: str) -> PhaseResult:
        """Transition phases: no command spikes between consecutive samples."""
        max_yaw_delta = 0.0
        max_pitch_delta = 0.0
        spike_count = 0

        for i in range(1, len(samples)):
            yaw_delta = abs(samples[i].yaw_cmd - samples[i-1].yaw_cmd)
            pitch_delta = abs(samples[i].pitch_cmd - samples[i-1].pitch_cmd)
            max_yaw_delta = max(max_yaw_delta, yaw_delta)
            max_pitch_delta = max(max_pitch_delta, pitch_delta)
            if yaw_delta > self.TRANSITION_SPIKE_MAX:
                spike_count += 1
            if pitch_delta > self.TRANSITION_SPIKE_MAX:
                spike_count += 1

        passed = spike_count == 0
        duration = samples[-1].t - samples[0].t if len(samples) > 1 else 0.0

        return PhaseResult(
            phase=phase_name,
            passed=passed,
            criteria={
                'max_yaw_delta': round(max_yaw_delta, 4),
                'max_pitch_delta': round(max_pitch_delta, 4),
                'spike_count': spike_count,
                'threshold': self.TRANSITION_SPIKE_MAX,
            },
            sample_count=len(samples),
            duration_s=round(duration, 1),
        )

    def _validate_fw_cruise(self, samples: list[Sample],
                            phase_name: str) -> PhaseResult:
        """FW cruise: camera nadir error should be small."""
        if not samples:
            return PhaseResult(phase=phase_name, passed=False,
                               criteria={'error': 'no samples'})

        # Nadir error = angle between gravity vector and body X axis
        # If gimbal were perfect, camera points along gravity.
        # Error = angle between commanded gimbal direction and actual.
        errors_deg = []
        for s in samples:
            # Tracking error = difference between command and actual position
            yaw_err = abs(s.yaw_cmd - s.yaw_actual)
            pitch_err = abs(s.pitch_cmd - s.pitch_actual)
            total_err = math.sqrt(yaw_err**2 + pitch_err**2)
            errors_deg.append(math.degrees(total_err))

        rms_error = math.sqrt(sum(e**2 for e in errors_deg) / len(errors_deg))
        max_error = max(errors_deg)
        passed = rms_error < self.FW_NADIR_ERROR_MAX
        duration = samples[-1].t - samples[0].t if len(samples) > 1 else 0.0

        # Check saturation behavior
        max_yaw_sat = max(s.yaw_sat for s in samples)
        max_pitch_sat = max(s.pitch_sat for s in samples)

        return PhaseResult(
            phase=phase_name,
            passed=passed,
            criteria={
                'rms_error_deg': round(rms_error, 2),
                'max_error_deg': round(max_error, 2),
                'threshold_deg': self.FW_NADIR_ERROR_MAX,
                'max_yaw_saturation': round(max_yaw_sat, 3),
                'max_pitch_saturation': round(max_pitch_sat, 3),
            },
            sample_count=len(samples),
            duration_s=round(duration, 1),
        )

    def _validate_tracking(self, fw_samples: list[Sample]) -> PhaseResult:
        """Overall gimbal tracking: actual joint angle follows commanded."""
        yaw_errors = [abs(s.yaw_cmd - s.yaw_actual) for s in fw_samples]
        pitch_errors = [abs(s.pitch_cmd - s.pitch_actual) for s in fw_samples]

        yaw_rms = math.sqrt(sum(e**2 for e in yaw_errors) / len(yaw_errors))
        pitch_rms = math.sqrt(sum(e**2 for e in pitch_errors) / len(pitch_errors))

        yaw_max = max(yaw_errors)
        pitch_max = max(pitch_errors)

        # Tracking is good if RMS < 3° and max < 10°
        yaw_rms_deg = math.degrees(yaw_rms)
        pitch_rms_deg = math.degrees(pitch_rms)
        yaw_max_deg = math.degrees(yaw_max)
        pitch_max_deg = math.degrees(pitch_max)

        passed = (yaw_rms_deg < 3.0 and pitch_rms_deg < 3.0
                  and yaw_max_deg < 10.0 and pitch_max_deg < 10.0)

        return PhaseResult(
            phase='GIMBAL_TRACKING',
            passed=passed,
            criteria={
                'yaw_rms_deg': round(yaw_rms_deg, 2),
                'pitch_rms_deg': round(pitch_rms_deg, 2),
                'yaw_max_deg': round(yaw_max_deg, 2),
                'pitch_max_deg': round(pitch_max_deg, 2),
            },
            sample_count=len(fw_samples),
        )


def print_results(results: list[PhaseResult]):
    """Print formatted test results."""
    print('\n' + '=' * 60)
    print('GIMBAL INTEGRATION TEST RESULTS')
    print('=' * 60)

    all_passed = True
    for r in results:
        status = 'PASS' if r.passed else 'FAIL'
        marker = '+' if r.passed else 'X'
        print(f'\n[{marker}] {r.phase}: {status}')
        print(f'    Samples: {r.sample_count}, Duration: {r.duration_s}s')
        for k, v in r.criteria.items():
            print(f'    {k}: {v}')
        if not r.passed:
            all_passed = False

    print('\n' + '=' * 60)
    overall = 'ALL PASSED' if all_passed else 'SOME FAILED'
    print(f'Overall: {overall} ({sum(1 for r in results if r.passed)}'
          f'/{len(results)} phases passed)')
    print('=' * 60 + '\n')

    return all_passed


def save_results(results: list[PhaseResult], samples: list[Sample],
                 output_path: str):
    """Save results and raw data to JSON."""
    data = {
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'total_samples': len(samples),
        'duration_s': samples[-1].t if samples else 0.0,
        'results': [
            {
                'phase': r.phase,
                'passed': r.passed,
                'criteria': r.criteria,
                'sample_count': r.sample_count,
                'duration_s': r.duration_s,
            }
            for r in results
        ],
        'samples': [
            {
                't': round(s.t, 3),
                'phase': s.phase,
                'yaw_cmd': round(s.yaw_cmd, 4),
                'pitch_cmd': round(s.pitch_cmd, 4),
                'yaw_actual': round(s.yaw_actual, 4),
                'pitch_actual': round(s.pitch_actual, 4),
                'yaw_sat': round(s.yaw_sat, 4),
                'pitch_sat': round(s.pitch_sat, 4),
                'gx': round(s.gx, 4),
                'gy': round(s.gy, 4),
                'gz': round(s.gz, 4),
            }
            for s in samples[::5]  # Save every 5th sample (2 Hz) to limit size
        ],
    }

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f'Results saved to {output_path}')


def main():
    parser = argparse.ArgumentParser(description='Gimbal integration test')
    parser.add_argument('--timeout', type=float, default=600.0,
                        help='Test timeout in seconds')
    parser.add_argument('--output', type=str,
                        default='/root/ws/src/fiber-nav-sim/logs/gimbal_test_results.json',
                        help='Output file path')
    parser.add_argument('--world-name', type=str, default='terrain_world',
                        help='Gazebo world name (for joint state topic)')
    args = parser.parse_args()

    rclpy.init()
    recorder = GimbalRecorder(timeout_s=args.timeout,
                              world_name=args.world_name)

    try:
        while rclpy.ok() and not recorder.done:
            rclpy.spin_once(recorder, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\nInterrupted by user')

    recorder.destroy_node()

    if not recorder.samples:
        print('ERROR: No samples recorded')
        rclpy.shutdown()
        sys.exit(1)

    print(f'\nRecorded {len(recorder.samples)} samples over '
          f'{recorder.samples[-1].t:.1f}s')

    # Validate
    validator = PhaseValidator(recorder.samples)
    results = validator.validate_all()

    if not results:
        print('WARNING: No phases detected for validation')
        rclpy.shutdown()
        sys.exit(1)

    # Output
    all_passed = print_results(results)
    save_results(results, recorder.samples, args.output)

    rclpy.shutdown()
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
