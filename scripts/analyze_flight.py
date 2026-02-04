#!/usr/bin/env python3
"""Analyze flight data and compute performance metrics."""
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


def compute_velocity_rmse(data):
    """Compute velocity RMSE between fusion output and ground truth."""
    if not data:
        return {'fusion_gt': 0.0, 'ekf_gt': 0.0, 'ekf_available': False}

    fusion_errors_sq = []
    ekf_errors_sq = []
    ekf_available = False

    for row in data:
        # Ground truth velocity
        gt_vx, gt_vy, gt_vz = row['gt_vx'], row['gt_vy'], row['gt_vz']

        # Fusion velocity error
        fus_err = (
            (row['fusion_vx'] - gt_vx) ** 2 +
            (row['fusion_vy'] - gt_vy) ** 2 +
            (row['fusion_vz'] - gt_vz) ** 2
        )
        fusion_errors_sq.append(fus_err)

        # EKF velocity error (only if EKF data is available)
        if row['ekf_vx'] != 0.0 or row['ekf_vy'] != 0.0 or row['ekf_vz'] != 0.0:
            ekf_available = True
            ekf_err = (
                (row['ekf_vx'] - gt_vx) ** 2 +
                (row['ekf_vy'] - gt_vy) ** 2 +
                (row['ekf_vz'] - gt_vz) ** 2
            )
            ekf_errors_sq.append(ekf_err)

    fusion_rmse = math.sqrt(sum(fusion_errors_sq) / len(fusion_errors_sq))
    ekf_rmse = math.sqrt(sum(ekf_errors_sq) / len(ekf_errors_sq)) if ekf_errors_sq else 0.0

    return {'fusion_gt': fusion_rmse, 'ekf_gt': ekf_rmse, 'ekf_available': ekf_available}


def compute_position_drift(data):
    """Compute position drift by integrating fusion velocity."""
    if not data:
        return {'total': 0.0, 'per_1000m': 0.0, 'distance_traveled': 0.0}

    # Get initial and final positions
    start = data[0]
    end = data[-1]

    # Ground truth distance traveled
    gt_dist = math.sqrt(
        (end['gt_x'] - start['gt_x']) ** 2 +
        (end['gt_y'] - start['gt_y']) ** 2 +
        (end['gt_z'] - start['gt_z']) ** 2
    )

    # Integrate fusion velocity to get estimated position
    fusion_x, fusion_y, fusion_z = start['gt_x'], start['gt_y'], start['gt_z']
    prev_time = start['time']

    for row in data[1:]:
        dt = row['time'] - prev_time
        fusion_x += row['fusion_vx'] * dt
        fusion_y += row['fusion_vy'] * dt
        fusion_z += row['fusion_vz'] * dt
        prev_time = row['time']

    # Position error from integrated fusion
    fusion_err = math.sqrt(
        (fusion_x - end['gt_x']) ** 2 +
        (fusion_y - end['gt_y']) ** 2 +
        (fusion_z - end['gt_z']) ** 2
    )

    # Drift per 1000m
    drift_per_1000m = (fusion_err / gt_dist * 1000) if gt_dist > 0 else 0

    return {
        'total': fusion_err,
        'distance_traveled': gt_dist,
        'per_1000m': drift_per_1000m
    }


def compute_statistics(data):
    """Compute general statistics."""
    if not data:
        return {}

    duration = data[-1]['time'] - data[0]['time']
    sample_count = len(data)
    sample_rate = sample_count / duration if duration > 0 else 0

    # Average speed
    speeds = [
        math.sqrt(row['gt_vx']**2 + row['gt_vy']**2 + row['gt_vz']**2)
        for row in data
    ]
    avg_speed = sum(speeds) / len(speeds) if speeds else 0
    max_speed = max(speeds) if speeds else 0

    return {
        'duration': duration,
        'samples': sample_count,
        'sample_rate': sample_rate,
        'avg_speed': avg_speed,
        'max_speed': max_speed
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

    # Compute metrics
    stats = compute_statistics(data)
    vel_rmse = compute_velocity_rmse(data)
    drift = compute_position_drift(data)

    # Print results
    print("=" * 60)
    print("FLIGHT STATISTICS")
    print("=" * 60)
    print(f"  Duration:      {stats['duration']:.1f} s")
    print(f"  Samples:       {stats['samples']}")
    print(f"  Sample rate:   {stats['sample_rate']:.1f} Hz")
    print(f"  Avg speed:     {stats['avg_speed']:.2f} m/s")
    print(f"  Max speed:     {stats['max_speed']:.2f} m/s")
    print(f"  Distance:      {drift['distance_traveled']:.1f} m")

    print("\n" + "=" * 60)
    print("VELOCITY RMSE")
    print("=" * 60)
    print(f"  Fusion vs GT:  {vel_rmse['fusion_gt']:.3f} m/s")
    if vel_rmse['ekf_available']:
        print(f"  EKF vs GT:     {vel_rmse['ekf_gt']:.3f} m/s")
    else:
        print(f"  EKF vs GT:     N/A (no EKF data)")

    # Target evaluation
    vel_target = 0.5
    fusion_status = "PASS" if vel_rmse['fusion_gt'] < vel_target else "FAIL"
    print(f"  Fusion target (<{vel_target} m/s): {fusion_status}")

    print("\n" + "=" * 60)
    print("POSITION DRIFT")
    print("=" * 60)
    print(f"  Total drift:   {drift['total']:.2f} m")
    print(f"  Per 1000m:     {drift['per_1000m']:.2f} m")

    # Target evaluation
    drift_target = 10.0
    drift_status = "PASS" if drift['per_1000m'] < drift_target else "FAIL"
    print(f"  Target (<{drift_target} m/1000m): {drift_status}")

    print("\n" + "=" * 60)
    print("SUCCESS CRITERIA (Fusion Algorithm)")
    print("=" * 60)
    print(f"  Velocity RMSE < 0.5 m/s:     {fusion_status}")
    print(f"  Drift < 10 m/1000m:          {drift_status}")
    print("=" * 60)

    if not vel_rmse['ekf_available']:
        print("\nNote: EKF data not available. Benchmarks are for fusion algorithm only.")
        print("For full EKF integration, PX4 needs proper Gazebo integration (not SIH).")


if __name__ == '__main__':
    main()
