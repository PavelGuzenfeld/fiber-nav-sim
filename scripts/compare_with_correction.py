#!/usr/bin/env python3
"""
4-way comparison: GPS vs Fiber+Vision vs Fiber+Vision+Landmarks vs IMU-only.

Shows how landmark-based corrections reduce drift in the Fiber+Vision approach.
"""
import csv
import math
import sys
import random
from pathlib import Path


def load_data(filepath):
    """Load flight data from CSV."""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({k: float(v) for k, v in row.items()})
    return data


def simulate_gps(data, pos_noise=3.0, vel_noise=0.15):
    """Simulate GPS measurements."""
    results = []
    for row in data:
        results.append({
            'time': row['time'],
            'gps_x': row['gt_x'] + random.gauss(0, pos_noise),
            'gps_y': row['gt_y'] + random.gauss(0, pos_noise),
            'gps_z': row['gt_z'] + random.gauss(0, pos_noise),
            'gps_vx': row['gt_vx'] + random.gauss(0, vel_noise),
            'gps_vy': row['gt_vy'] + random.gauss(0, vel_noise),
            'gps_vz': row['gt_vz'] + random.gauss(0, vel_noise),
        })
    return results


def simulate_imu_only(data, accel_bias=0.02, accel_noise=0.1):
    """Simulate IMU-only dead reckoning with quadratic drift."""
    if len(data) < 2:
        return []

    imu_vx, imu_vy, imu_vz = data[0]['gt_vx'], data[0]['gt_vy'], data[0]['gt_vz']
    imu_x, imu_y, imu_z = data[0]['gt_x'], data[0]['gt_y'], data[0]['gt_z']

    results = []
    prev_time = data[0]['time']

    for i, row in enumerate(data):
        dt = row['time'] - prev_time if i > 0 else 0.1
        prev_time = row['time']

        if i > 0:
            prev = data[i-1]
            true_ax = (row['gt_vx'] - prev['gt_vx']) / dt if dt > 0 else 0
            true_ay = (row['gt_vy'] - prev['gt_vy']) / dt if dt > 0 else 0
            true_az = (row['gt_vz'] - prev['gt_vz']) / dt if dt > 0 else 0

            meas_ax = true_ax + accel_bias + random.gauss(0, accel_noise)
            meas_ay = true_ay + accel_bias * 0.5 + random.gauss(0, accel_noise)
            meas_az = true_az + accel_bias * 0.3 + random.gauss(0, accel_noise)

            imu_vx += meas_ax * dt
            imu_vy += meas_ay * dt
            imu_vz += meas_az * dt
            imu_x += imu_vx * dt
            imu_y += imu_vy * dt
            imu_z += imu_vz * dt

        results.append({
            'time': row['time'],
            'imu_vx': imu_vx, 'imu_vy': imu_vy, 'imu_vz': imu_vz,
            'imu_x': imu_x, 'imu_y': imu_y, 'imu_z': imu_z,
        })

    return results


def integrate_fusion_position(data):
    """Integrate fusion velocity to get position (no landmarks)."""
    if len(data) < 2:
        return []

    fx, fy, fz = data[0]['gt_x'], data[0]['gt_y'], data[0]['gt_z']
    results = [{'time': data[0]['time'], 'fusion_x': fx, 'fusion_y': fy, 'fusion_z': fz}]
    prev_time = data[0]['time']

    for row in data[1:]:
        dt = row['time'] - prev_time
        fx += row['fusion_vx'] * dt
        fy += row['fusion_vy'] * dt
        fz += row['fusion_vz'] * dt
        prev_time = row['time']
        results.append({'time': row['time'], 'fusion_x': fx, 'fusion_y': fy, 'fusion_z': fz})

    return results


def integrate_with_landmarks(data, landmark_interval=200, correction_gain=0.8):
    """
    Integrate fusion velocity with landmark corrections.

    Every `landmark_interval` meters, apply a position correction
    that reduces accumulated error.
    """
    if len(data) < 2:
        return []

    lx, ly, lz = data[0]['gt_x'], data[0]['gt_y'], data[0]['gt_z']
    results = [{'time': data[0]['time'], 'lm_x': lx, 'lm_y': ly, 'lm_z': lz}]
    prev_time = data[0]['time']
    last_landmark = 0
    corrections = 0

    for row in data[1:]:
        dt = row['time'] - prev_time

        # Integrate velocity
        lx += row['fusion_vx'] * dt
        ly += row['fusion_vy'] * dt
        lz += row['fusion_vz'] * dt

        # Check for landmark
        gt_x = row['gt_x']
        current_landmark = int(gt_x / landmark_interval) * landmark_interval

        if current_landmark > last_landmark and current_landmark > 0:
            # Apply correction toward ground truth (simulating landmark detection)
            error_x = row['gt_x'] - lx
            error_y = row['gt_y'] - ly
            error_z = row['gt_z'] - lz

            lx += correction_gain * error_x
            ly += correction_gain * error_y
            lz += correction_gain * error_z

            last_landmark = current_landmark
            corrections += 1

        prev_time = row['time']
        results.append({'time': row['time'], 'lm_x': lx, 'lm_y': ly, 'lm_z': lz})

    return results, corrections


def compute_metrics(data, gps_data, imu_data, fusion_pos, landmark_pos):
    """Compute comparison metrics for all four methods."""
    n = len(data)

    # Position error at end
    end = data[-1]
    gt_x, gt_y, gt_z = end['gt_x'], end['gt_y'], end['gt_z']

    gps_pos_err = math.sqrt(
        (gps_data[-1]['gps_x'] - gt_x)**2 +
        (gps_data[-1]['gps_y'] - gt_y)**2 +
        (gps_data[-1]['gps_z'] - gt_z)**2
    )

    imu_pos_err = math.sqrt(
        (imu_data[-1]['imu_x'] - gt_x)**2 +
        (imu_data[-1]['imu_y'] - gt_y)**2 +
        (imu_data[-1]['imu_z'] - gt_z)**2
    )

    fusion_pos_err = math.sqrt(
        (fusion_pos[-1]['fusion_x'] - gt_x)**2 +
        (fusion_pos[-1]['fusion_y'] - gt_y)**2 +
        (fusion_pos[-1]['fusion_z'] - gt_z)**2
    )

    landmark_pos_err = math.sqrt(
        (landmark_pos[-1]['lm_x'] - gt_x)**2 +
        (landmark_pos[-1]['lm_y'] - gt_y)**2 +
        (landmark_pos[-1]['lm_z'] - gt_z)**2
    )

    # Distance traveled
    start = data[0]
    gt_dist = math.sqrt(
        (end['gt_x'] - start['gt_x'])**2 +
        (end['gt_y'] - start['gt_y'])**2 +
        (end['gt_z'] - start['gt_z'])**2
    )

    return {
        'gps_pos_err': gps_pos_err,
        'imu_pos_err': imu_pos_err,
        'fusion_pos_err': fusion_pos_err,
        'landmark_pos_err': landmark_pos_err,
        'gt_distance': gt_dist,
        'duration': end['time'] - start['time'],
    }


def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else '/tmp/flight_data.csv'

    if not Path(filepath).exists():
        print(f"Error: File not found: {filepath}")
        sys.exit(1)

    print(f"Analyzing: {filepath}\n")

    data = load_data(filepath)
    if not data:
        print("Error: No data")
        sys.exit(1)

    # Simulate all methods
    random.seed(42)
    gps_data = simulate_gps(data)
    imu_data = simulate_imu_only(data)
    fusion_pos = integrate_fusion_position(data)
    landmark_pos, n_corrections = integrate_with_landmarks(data)

    metrics = compute_metrics(data, gps_data, imu_data, fusion_pos, landmark_pos)

    print("=" * 80)
    print("4-WAY COMPARISON: GPS vs Fiber+Vision vs Fiber+Vision+Landmarks vs IMU-only")
    print("=" * 80)
    print(f"  Flight duration: {metrics['duration']:.0f}s ({metrics['duration']/60:.1f} min)")
    print(f"  Distance traveled: {metrics['gt_distance']/1000:.1f} km")
    print(f"  Landmark corrections: {n_corrections}")
    print()

    print("-" * 80)
    print("POSITION ERROR COMPARISON")
    print("-" * 80)
    print(f"  {'Method':<25} {'Error (m)':<12} {'% Distance':<12} {'Improvement':<15}")
    print(f"  {'-'*25} {'-'*12} {'-'*12} {'-'*15}")
    print(f"  {'GPS':<25} {metrics['gps_pos_err']:<12.1f} {100*metrics['gps_pos_err']/metrics['gt_distance']:<12.3f} {'(reference)':<15}")
    print(f"  {'Fiber+Vision':<25} {metrics['fusion_pos_err']:<12.1f} {100*metrics['fusion_pos_err']/metrics['gt_distance']:<12.2f} {'':<15}")
    print(f"  {'Fiber+Vision+Landmarks':<25} {metrics['landmark_pos_err']:<12.1f} {100*metrics['landmark_pos_err']/metrics['gt_distance']:<12.2f} {metrics['fusion_pos_err']/metrics['landmark_pos_err']:.1f}x better")
    print(f"  {'IMU-only':<25} {metrics['imu_pos_err']:<12.1f} {100*metrics['imu_pos_err']/metrics['gt_distance']:<12.1f} {'':<15}")
    print()

    improvement = metrics['fusion_pos_err'] / metrics['landmark_pos_err']
    print("=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print()
    print(f"  Landmark corrections reduce position error by {improvement:.1f}x")
    print(f"  Final error with landmarks: {metrics['landmark_pos_err']:.1f}m "
          f"({100*metrics['landmark_pos_err']/metrics['gt_distance']:.2f}% of distance)")
    print()
    print("  Position Error Comparison:")
    print(f"    GPS:                      {metrics['gps_pos_err']:>8.1f}m")
    print(f"    Fiber+Vision+Landmarks:   {metrics['landmark_pos_err']:>8.1f}m  <-- Best GPS-denied option")
    print(f"    Fiber+Vision (no fix):    {metrics['fusion_pos_err']:>8.1f}m")
    print(f"    IMU-only:                 {metrics['imu_pos_err']:>8.1f}m")
    print("=" * 80)


if __name__ == '__main__':
    main()
