#!/usr/bin/env python3
"""Analyze PX4 SITL flight data and compute performance metrics.

Handles coordinate frame conversion:
- Ground truth (GT) position: Gazebo world frame (ENU: X=East, Y=North, Z=Up)
- Ground truth (GT) velocity: Gazebo body frame (FLU: X=Forward, Y=Left, Z=Up)
- GT quaternion: body (FLU) → world (ENU) rotation
- EKF estimate: PX4 local NED frame (X=North, Y=East, Z=Down)
- Fusion output: PX4 NED frame (velocity only)

Position conversion: NED_x = GT_y, NED_y = GT_x, NED_z = -GT_z
Velocity conversion: rotate FLU body velocity to ENU world frame using GT quaternion,
                     then convert ENU → NED
"""
import csv
import sys
import math
from pathlib import Path


def load_data(filepath):
    """Load flight data from CSV."""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({k: float(v) for k, v in row.items()})
    return data


def has_quaternion(data):
    """Check if data includes GT quaternion columns."""
    return data and 'gt_qw' in data[0]


def quat_rotate(qw, qx, qy, qz, vx, vy, vz):
    """Rotate vector (vx,vy,vz) by quaternion (qw,qx,qy,qz).

    Computes q * v * q^-1 where v is treated as a pure quaternion (0, vx, vy, vz).
    """
    # q * v (quaternion * pure quaternion)
    t0 = -qx * vx - qy * vy - qz * vz
    t1 = qw * vx + qy * vz - qz * vy
    t2 = qw * vy + qz * vx - qx * vz
    t3 = qw * vz + qx * vy - qy * vx

    # (q * v) * q^-1 = (q * v) * q_conj (for unit quaternions)
    rx = t1 * qw - t0 * qx - t2 * qz + t3 * qy
    ry = t2 * qw - t0 * qy - t3 * qx + t1 * qz
    rz = t3 * qw - t0 * qz - t1 * qy + t2 * qx

    return rx, ry, rz


def convert_gt_to_ned(data):
    """Convert GT positions/velocities from Gazebo ENU to PX4 NED.

    Position: world frame ENU, direct axis swap.
    Velocity: if quaternion available, rotate body (FLU) → world (ENU) first.
              Otherwise, assume world frame (may be inaccurate).
    """
    use_quat = has_quaternion(data)

    for row in data:
        # Position: ENU → NED (world frame, direct swap)
        enu_x, enu_y, enu_z = row['gt_x'], row['gt_y'], row['gt_z']
        row['gt_ned_x'] = enu_y   # North = ENU_Y
        row['gt_ned_y'] = enu_x   # East = ENU_X
        row['gt_ned_z'] = -enu_z  # Down = -ENU_Z

        # Velocity: body FLU → world ENU → NED
        bvx, bvy, bvz = row['gt_vx'], row['gt_vy'], row['gt_vz']

        if use_quat:
            # Rotate body-frame velocity to world frame using GT quaternion
            qw, qx, qy, qz = row['gt_qw'], row['gt_qx'], row['gt_qy'], row['gt_qz']
            enu_vx, enu_vy, enu_vz = quat_rotate(qw, qx, qy, qz, bvx, bvy, bvz)
        else:
            # Fallback: assume velocity is already in world frame
            enu_vx, enu_vy, enu_vz = bvx, bvy, bvz

        # ENU → NED
        row['gt_ned_vx'] = enu_vy   # North = ENU_VY
        row['gt_ned_vy'] = enu_vx   # East = ENU_VX
        row['gt_ned_vz'] = -enu_vz  # Down = -ENU_VZ

    return data


def align_origins(data):
    """Align GT and EKF origins by subtracting initial positions."""
    if not data:
        return data

    gt_x0 = data[0]['gt_ned_x']
    gt_y0 = data[0]['gt_ned_y']
    gt_z0 = data[0]['gt_ned_z']
    ekf_x0 = data[0]['ekf_x']
    ekf_y0 = data[0]['ekf_y']
    ekf_z0 = data[0]['ekf_z']

    for row in data:
        row['gt_rel_x'] = row['gt_ned_x'] - gt_x0
        row['gt_rel_y'] = row['gt_ned_y'] - gt_y0
        row['gt_rel_z'] = row['gt_ned_z'] - gt_z0
        row['ekf_rel_x'] = row['ekf_x'] - ekf_x0
        row['ekf_rel_y'] = row['ekf_y'] - ekf_y0
        row['ekf_rel_z'] = row['ekf_z'] - ekf_z0
    return data


def compute_position_error(data):
    """Compute position error between EKF and GT (both in NED, origin-aligned)."""
    if not data:
        return {'mean': 0.0, 'max': 0.0, 'final': 0.0}

    errors = []
    for row in data:
        err = math.sqrt(
            (row['ekf_rel_x'] - row['gt_rel_x']) ** 2 +
            (row['ekf_rel_y'] - row['gt_rel_y']) ** 2 +
            (row['ekf_rel_z'] - row['gt_rel_z']) ** 2
        )
        errors.append(err)

    return {
        'mean': sum(errors) / len(errors),
        'max': max(errors),
        'final': errors[-1],
        'errors': errors,
    }


def compute_speed_rmse(data):
    """Compute speed RMSE (scalar, frame-invariant)."""
    if not data:
        return {'ekf_gt': 0.0, 'fusion_gt': 0.0, 'fusion_available': False}

    ekf_errors_sq = []
    fusion_errors_sq = []
    fusion_available = False

    for row in data:
        # GT speed (scalar magnitude — frame invariant)
        gt_speed = math.sqrt(row['gt_vx']**2 + row['gt_vy']**2 + row['gt_vz']**2)

        # EKF speed
        ekf_speed = math.sqrt(row['ekf_vx']**2 + row['ekf_vy']**2 + row['ekf_vz']**2)
        ekf_errors_sq.append((ekf_speed - gt_speed) ** 2)

        # Fusion speed (only if fusion data available)
        fus_speed = math.sqrt(row['fusion_vx']**2 + row['fusion_vy']**2 + row['fusion_vz']**2)
        if fus_speed > 0.0 or fusion_available:
            fusion_available = True
            fusion_errors_sq.append((fus_speed - gt_speed) ** 2)

    ekf_rmse = math.sqrt(sum(ekf_errors_sq) / len(ekf_errors_sq))
    fusion_rmse = math.sqrt(sum(fusion_errors_sq) / len(fusion_errors_sq)) if fusion_errors_sq else 0.0

    return {
        'ekf_gt': ekf_rmse,
        'fusion_gt': fusion_rmse,
        'fusion_available': fusion_available,
    }


def compute_velocity_rmse_ned(data):
    """Compute velocity RMSE in NED frame (EKF vs GT, Fusion vs GT)."""
    if not data:
        return {'ekf_gt': 0.0, 'fusion_gt': 0.0, 'fusion_available': False}

    ekf_errors_sq = []
    fusion_errors_sq = []
    fusion_available = False

    for row in data:
        # GT velocity in NED (properly rotated from body frame if quaternion available)
        gt_vx, gt_vy, gt_vz = row['gt_ned_vx'], row['gt_ned_vy'], row['gt_ned_vz']

        # EKF velocity error (already in NED)
        ekf_err = (
            (row['ekf_vx'] - gt_vx) ** 2 +
            (row['ekf_vy'] - gt_vy) ** 2 +
            (row['ekf_vz'] - gt_vz) ** 2
        )
        ekf_errors_sq.append(ekf_err)

        # Fusion velocity error (in NED)
        fus_speed = math.sqrt(row['fusion_vx']**2 + row['fusion_vy']**2 + row['fusion_vz']**2)
        if fus_speed > 0.0 or fusion_available:
            fusion_available = True
            fus_err = (
                (row['fusion_vx'] - gt_vx) ** 2 +
                (row['fusion_vy'] - gt_vy) ** 2 +
                (row['fusion_vz'] - gt_vz) ** 2
            )
            fusion_errors_sq.append(fus_err)

    ekf_rmse = math.sqrt(sum(ekf_errors_sq) / len(ekf_errors_sq))
    fusion_rmse = math.sqrt(sum(fusion_errors_sq) / len(fusion_errors_sq)) if fusion_errors_sq else 0.0

    return {
        'ekf_gt': ekf_rmse,
        'fusion_gt': fusion_rmse,
        'fusion_available': fusion_available,
    }


def compute_statistics(data):
    """Compute general statistics."""
    if not data:
        return {}

    duration = data[-1]['time'] - data[0]['time']
    sample_count = len(data)
    sample_rate = sample_count / duration if duration > 0 else 0

    # Average speed (scalar, frame invariant)
    speeds = [
        math.sqrt(row['gt_vx']**2 + row['gt_vy']**2 + row['gt_vz']**2)
        for row in data
    ]
    avg_speed = sum(speeds) / len(speeds) if speeds else 0
    max_speed = max(speeds) if speeds else 0

    # Total distance traveled (integrate GT speed)
    total_dist = 0.0
    prev_time = data[0]['time']
    for row in data[1:]:
        dt = row['time'] - prev_time
        speed = math.sqrt(row['gt_vx']**2 + row['gt_vy']**2 + row['gt_vz']**2)
        total_dist += speed * dt
        prev_time = row['time']

    return {
        'duration': duration,
        'samples': sample_count,
        'sample_rate': sample_rate,
        'avg_speed': avg_speed,
        'max_speed': max_speed,
        'total_distance': total_dist,
    }


def compute_ekf_position_drift(data):
    """Compute EKF position drift per 1000m traveled."""
    if not data:
        return {'total': 0.0, 'per_1000m': 0.0}

    # Total GT distance traveled
    total_dist = 0.0
    prev_time = data[0]['time']
    for row in data[1:]:
        dt = row['time'] - prev_time
        speed = math.sqrt(row['gt_vx']**2 + row['gt_vy']**2 + row['gt_vz']**2)
        total_dist += speed * dt
        prev_time = row['time']

    # Final position error (EKF vs GT, origin-aligned)
    last = data[-1]
    final_err = math.sqrt(
        (last['ekf_rel_x'] - last['gt_rel_x']) ** 2 +
        (last['ekf_rel_y'] - last['gt_rel_y']) ** 2 +
        (last['ekf_rel_z'] - last['gt_rel_z']) ** 2
    )

    per_1000m = (final_err / total_dist * 1000) if total_dist > 0 else 0

    return {
        'total': final_err,
        'distance_traveled': total_dist,
        'per_1000m': per_1000m,
    }


def main():
    if len(sys.argv) < 2:
        filepath = '/tmp/flight_data.csv'
    else:
        filepath = sys.argv[1]

    if not Path(filepath).exists():
        print(f"Error: File not found: {filepath}")
        sys.exit(1)

    print(f"Analyzing: {filepath}\n")

    data = load_data(filepath)
    if not data:
        print("Error: No data in file")
        sys.exit(1)

    use_quat = has_quaternion(data)

    # Convert frames and align origins
    data = convert_gt_to_ned(data)
    data = align_origins(data)

    # Compute metrics
    stats = compute_statistics(data)
    pos_err = compute_position_error(data)
    speed_rmse = compute_speed_rmse(data)
    vel_rmse = compute_velocity_rmse_ned(data)
    drift = compute_ekf_position_drift(data)

    # Print results
    print("=" * 60)
    print("PX4 SITL PERFORMANCE TEST")
    print("=" * 60)

    print("\n--- Flight Statistics ---")
    print(f"  Duration:          {stats['duration']:.1f} s")
    print(f"  Samples:           {stats['samples']}")
    print(f"  Sample rate:       {stats['sample_rate']:.1f} Hz")
    print(f"  Avg speed:         {stats['avg_speed']:.2f} m/s")
    print(f"  Max speed:         {stats['max_speed']:.2f} m/s")
    print(f"  Total distance:    {stats['total_distance']:.0f} m")

    print("\n--- EKF Position Accuracy (vs Ground Truth) ---")
    print(f"  Mean error:        {pos_err['mean']:.2f} m")
    print(f"  Max error:         {pos_err['max']:.2f} m")
    print(f"  Final error:       {pos_err['final']:.2f} m")
    print(f"  Drift per 1000m:   {drift['per_1000m']:.2f} m")

    pos_target = 5.0
    pos_status = "PASS" if pos_err['mean'] < pos_target else "FAIL"
    print(f"  Target (<{pos_target} m mean): {pos_status}")

    print("\n--- Speed RMSE (scalar, frame-invariant) ---")
    print(f"  EKF vs GT:         {speed_rmse['ekf_gt']:.3f} m/s")
    if speed_rmse['fusion_available']:
        print(f"  Fusion vs GT:      {speed_rmse['fusion_gt']:.3f} m/s")
    else:
        print(f"  Fusion vs GT:      N/A (no fusion data)")

    vel_frame = "body→NED via quaternion" if use_quat else "assumed world frame (may be inaccurate)"
    print(f"\n--- Velocity RMSE (3D vector, NED frame — {vel_frame}) ---")
    print(f"  EKF vs GT:         {vel_rmse['ekf_gt']:.3f} m/s")
    if vel_rmse['fusion_available']:
        print(f"  Fusion vs GT:      {vel_rmse['fusion_gt']:.3f} m/s")

    vel_target = 0.5
    vel_status = "PASS" if vel_rmse['ekf_gt'] < vel_target else "FAIL"
    print(f"  EKF target (<{vel_target} m/s): {vel_status}")

    print("\n" + "=" * 60)
    print("PASS/FAIL SUMMARY")
    print("=" * 60)
    print(f"  EKF position < 5m mean:      {pos_status}")
    print(f"  EKF velocity < 0.5 m/s RMSE: {vel_status}")
    drift_status = "PASS" if drift['per_1000m'] < 10.0 else "FAIL"
    print(f"  EKF drift < 10 m/1000m:      {drift_status}")
    if speed_rmse['fusion_available']:
        fus_status = "PASS" if speed_rmse['fusion_gt'] < 0.5 else "FAIL"
        print(f"  Fusion speed < 0.5 m/s RMSE: {fus_status}")
    print("=" * 60)

    if not use_quat:
        print(f"\nWarning: No GT quaternion in data. Velocity RMSE uses assumed")
        print(f"world-frame conversion which may be inaccurate if GT twist is")
        print(f"in body frame. Re-record with updated recorder for accurate results.")
        print(f"Speed RMSE remains valid (frame-invariant).")


if __name__ == '__main__':
    main()
