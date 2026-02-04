#!/usr/bin/env python3
"""Compare fiber+vision fusion vs IMU-only dead reckoning."""
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


def simulate_imu_only(data, accel_bias=0.02, gyro_bias=0.001, accel_noise=0.1):
    """
    Simulate IMU-only dead reckoning from ground truth.

    Typical MEMS IMU specs:
    - Accelerometer bias: 10-50 mg (0.1-0.5 m/s²)
    - Accelerometer noise: 0.1-0.5 m/s²/√Hz
    - Gyro bias: 1-10 °/hr

    Args:
        accel_bias: Constant accelerometer bias (m/s²)
        gyro_bias: Gyro drift rate (rad/s)
        accel_noise: Accelerometer noise std dev (m/s²)
    """
    if len(data) < 2:
        return []

    # Initialize from ground truth
    imu_vx = data[0]['gt_vx']
    imu_vy = data[0]['gt_vy']
    imu_vz = data[0]['gt_vz']
    imu_x = data[0]['gt_x']
    imu_y = data[0]['gt_y']
    imu_z = data[0]['gt_z']

    results = []
    prev_time = data[0]['time']

    for i, row in enumerate(data):
        dt = row['time'] - prev_time if i > 0 else 0.1
        prev_time = row['time']

        if i > 0:
            # Calculate "true" acceleration from ground truth velocity change
            prev = data[i-1]
            true_ax = (row['gt_vx'] - prev['gt_vx']) / dt if dt > 0 else 0
            true_ay = (row['gt_vy'] - prev['gt_vy']) / dt if dt > 0 else 0
            true_az = (row['gt_vz'] - prev['gt_vz']) / dt if dt > 0 else 0

            # Add IMU errors
            meas_ax = true_ax + accel_bias + random.gauss(0, accel_noise)
            meas_ay = true_ay + accel_bias * 0.5 + random.gauss(0, accel_noise)
            meas_az = true_az + accel_bias * 0.3 + random.gauss(0, accel_noise)

            # Integrate to get velocity
            imu_vx += meas_ax * dt
            imu_vy += meas_ay * dt
            imu_vz += meas_az * dt

            # Integrate to get position
            imu_x += imu_vx * dt
            imu_y += imu_vy * dt
            imu_z += imu_vz * dt

        results.append({
            'time': row['time'],
            'imu_vx': imu_vx, 'imu_vy': imu_vy, 'imu_vz': imu_vz,
            'imu_x': imu_x, 'imu_y': imu_y, 'imu_z': imu_z,
            'gt_vx': row['gt_vx'], 'gt_vy': row['gt_vy'], 'gt_vz': row['gt_vz'],
            'gt_x': row['gt_x'], 'gt_y': row['gt_y'], 'gt_z': row['gt_z'],
            'fusion_vx': row['fusion_vx'], 'fusion_vy': row['fusion_vy'], 'fusion_vz': row['fusion_vz'],
        })

    return results


def compute_metrics(results, data):
    """Compute comparison metrics."""
    if not results:
        return {}

    # Velocity RMSE
    imu_vel_err_sq = []
    fusion_vel_err_sq = []

    for r in results:
        imu_err = (
            (r['imu_vx'] - r['gt_vx'])**2 +
            (r['imu_vy'] - r['gt_vy'])**2 +
            (r['imu_vz'] - r['gt_vz'])**2
        )
        imu_vel_err_sq.append(imu_err)

        fusion_err = (
            (r['fusion_vx'] - r['gt_vx'])**2 +
            (r['fusion_vy'] - r['gt_vy'])**2 +
            (r['fusion_vz'] - r['gt_vz'])**2
        )
        fusion_vel_err_sq.append(fusion_err)

    imu_vel_rmse = math.sqrt(sum(imu_vel_err_sq) / len(imu_vel_err_sq))
    fusion_vel_rmse = math.sqrt(sum(fusion_vel_err_sq) / len(fusion_vel_err_sq))

    # Position drift (integrated)
    start = results[0]
    end = results[-1]

    gt_dist = math.sqrt(
        (end['gt_x'] - start['gt_x'])**2 +
        (end['gt_y'] - start['gt_y'])**2 +
        (end['gt_z'] - start['gt_z'])**2
    )

    imu_pos_err = math.sqrt(
        (end['imu_x'] - end['gt_x'])**2 +
        (end['imu_y'] - end['gt_y'])**2 +
        (end['imu_z'] - end['gt_z'])**2
    )

    # Integrate fusion velocity for position
    fusion_x, fusion_y, fusion_z = start['gt_x'], start['gt_y'], start['gt_z']
    prev_time = start['time']
    for r in results[1:]:
        dt = r['time'] - prev_time
        fusion_x += r['fusion_vx'] * dt
        fusion_y += r['fusion_vy'] * dt
        fusion_z += r['fusion_vz'] * dt
        prev_time = r['time']

    fusion_pos_err = math.sqrt(
        (fusion_x - end['gt_x'])**2 +
        (fusion_y - end['gt_y'])**2 +
        (fusion_z - end['gt_z'])**2
    )

    return {
        'imu_vel_rmse': imu_vel_rmse,
        'fusion_vel_rmse': fusion_vel_rmse,
        'imu_pos_err': imu_pos_err,
        'fusion_pos_err': fusion_pos_err,
        'gt_distance': gt_dist,
        'imu_drift_per_1000m': (imu_pos_err / gt_dist * 1000) if gt_dist > 0 else 0,
        'fusion_drift_per_1000m': (fusion_pos_err / gt_dist * 1000) if gt_dist > 0 else 0,
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

    # Run IMU simulation with typical consumer-grade MEMS specs
    print("Simulating IMU-only dead reckoning (consumer MEMS specs)...")
    print("  Accel bias: 20 mg (0.02 m/s²)")
    print("  Accel noise: 0.1 m/s²")
    print()

    results = simulate_imu_only(data, accel_bias=0.02, accel_noise=0.1)
    metrics = compute_metrics(results, data)

    print("=" * 70)
    print("COMPARISON: FIBER+VISION FUSION vs IMU-ONLY")
    print("=" * 70)
    print(f"  Flight duration: {metrics['duration']:.1f}s")
    print(f"  Distance traveled: {metrics['gt_distance']:.1f}m")
    print()

    print("-" * 70)
    print("VELOCITY RMSE")
    print("-" * 70)
    print(f"  {'Method':<25} {'RMSE (m/s)':<15} {'Improvement':<15}")
    print(f"  {'-'*25} {'-'*15} {'-'*15}")
    print(f"  {'IMU-only':<25} {metrics['imu_vel_rmse']:<15.3f} {'(baseline)':<15}")
    print(f"  {'Fiber+Vision Fusion':<25} {metrics['fusion_vel_rmse']:<15.3f} {metrics['imu_vel_rmse']/metrics['fusion_vel_rmse']:.1f}x better")
    print()

    print("-" * 70)
    print("POSITION DRIFT (per 1000m)")
    print("-" * 70)
    print(f"  {'Method':<25} {'Drift (m)':<15} {'Improvement':<15}")
    print(f"  {'-'*25} {'-'*15} {'-'*15}")
    print(f"  {'IMU-only':<25} {metrics['imu_drift_per_1000m']:<15.1f} {'(baseline)':<15}")
    print(f"  {'Fiber+Vision Fusion':<25} {metrics['fusion_drift_per_1000m']:<15.1f} {metrics['imu_drift_per_1000m']/metrics['fusion_drift_per_1000m']:.1f}x better")
    print()

    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    vel_improvement = metrics['imu_vel_rmse'] / metrics['fusion_vel_rmse']
    drift_improvement = metrics['imu_drift_per_1000m'] / metrics['fusion_drift_per_1000m']
    print(f"  Fiber+Vision fusion provides:")
    print(f"    - {vel_improvement:.1f}x better velocity accuracy")
    print(f"    - {drift_improvement:.1f}x less position drift")
    print()
    print("  IMU-only drift grows quadratically with time (double integration).")
    print("  Fiber sensor provides direct velocity measurement, avoiding this.")
    print("=" * 70)


def show_drift_over_time(data):
    """Show how drift accumulates over time for both methods."""
    print("\n" + "=" * 70)
    print("DRIFT ACCUMULATION OVER TIME")
    print("=" * 70)
    print(f"  {'Time (s)':<12} {'IMU Drift (m)':<18} {'Fusion Drift (m)':<18} {'Ratio':<10}")
    print(f"  {'-'*12} {'-'*18} {'-'*18} {'-'*10}")

    # Simulate at different time points
    checkpoints = [5, 10, 15, 20, 25, 30]

    for t in checkpoints:
        # Find data up to time t
        subset = [d for d in data if d['time'] <= t]
        if len(subset) < 2:
            continue

        results = simulate_imu_only(subset, accel_bias=0.02, accel_noise=0.1)
        if not results:
            continue

        start = results[0]
        end = results[-1]

        # IMU position error
        imu_err = math.sqrt(
            (end['imu_x'] - end['gt_x'])**2 +
            (end['imu_y'] - end['gt_y'])**2 +
            (end['imu_z'] - end['gt_z'])**2
        )

        # Fusion position error (integrated)
        fx, fy, fz = start['gt_x'], start['gt_y'], start['gt_z']
        prev_time = start['time']
        for r in results[1:]:
            dt = r['time'] - prev_time
            fx += r['fusion_vx'] * dt
            fy += r['fusion_vy'] * dt
            fz += r['fusion_vz'] * dt
            prev_time = r['time']

        fusion_err = math.sqrt(
            (fx - end['gt_x'])**2 +
            (fy - end['gt_y'])**2 +
            (fz - end['gt_z'])**2
        )

        ratio = imu_err / fusion_err if fusion_err > 0 else 0
        print(f"  {t:<12} {imu_err:<18.2f} {fusion_err:<18.2f} {ratio:<10.1f}x")

    print()
    print("  Note: IMU drift grows ~t² (quadratic), fusion drift grows ~t (linear)")
    print("=" * 70)


if __name__ == '__main__':
    main()
    # Also show drift over time
    data = load_data('/tmp/flight_data.csv')
    show_drift_over_time(data)
