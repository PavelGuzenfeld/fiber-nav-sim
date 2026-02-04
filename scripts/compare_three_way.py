#!/usr/bin/env python3
"""3-way comparison: GPS vs Fiber+Vision vs IMU-only."""
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
    """
    Simulate GPS measurements.

    Typical civilian GPS specs:
    - Position accuracy: 2-5m CEP (we use 3m std dev)
    - Velocity accuracy: 0.1-0.3 m/s (we use 0.15 m/s)
    - No drift accumulation (absolute measurement)
    """
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
    """
    Simulate IMU-only dead reckoning.

    Typical MEMS IMU specs:
    - Accelerometer bias: 10-50 mg (0.1-0.5 m/s²)
    - Accelerometer noise: 0.1-0.5 m/s²
    - Drift grows quadratically with time
    """
    if len(data) < 2:
        return []

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
    """Integrate fusion velocity to get position."""
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


def compute_metrics(data, gps_data, imu_data, fusion_pos):
    """Compute comparison metrics for all three methods."""
    n = len(data)

    # Velocity RMSE
    gps_vel_err = []
    imu_vel_err = []
    fusion_vel_err = []

    for i in range(n):
        row = data[i]
        gt_vx, gt_vy, gt_vz = row['gt_vx'], row['gt_vy'], row['gt_vz']

        gps_vel_err.append(
            (gps_data[i]['gps_vx'] - gt_vx)**2 +
            (gps_data[i]['gps_vy'] - gt_vy)**2 +
            (gps_data[i]['gps_vz'] - gt_vz)**2
        )

        imu_vel_err.append(
            (imu_data[i]['imu_vx'] - gt_vx)**2 +
            (imu_data[i]['imu_vy'] - gt_vy)**2 +
            (imu_data[i]['imu_vz'] - gt_vz)**2
        )

        fusion_vel_err.append(
            (row['fusion_vx'] - gt_vx)**2 +
            (row['fusion_vy'] - gt_vy)**2 +
            (row['fusion_vz'] - gt_vz)**2
        )

    gps_vel_rmse = math.sqrt(sum(gps_vel_err) / n)
    imu_vel_rmse = math.sqrt(sum(imu_vel_err) / n)
    fusion_vel_rmse = math.sqrt(sum(fusion_vel_err) / n)

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

    # Distance traveled
    start = data[0]
    gt_dist = math.sqrt(
        (end['gt_x'] - start['gt_x'])**2 +
        (end['gt_y'] - start['gt_y'])**2 +
        (end['gt_z'] - start['gt_z'])**2
    )

    return {
        'gps_vel_rmse': gps_vel_rmse,
        'imu_vel_rmse': imu_vel_rmse,
        'fusion_vel_rmse': fusion_vel_rmse,
        'gps_pos_err': gps_pos_err,
        'imu_pos_err': imu_pos_err,
        'fusion_pos_err': fusion_pos_err,
        'gt_distance': gt_dist,
        'duration': end['time'] - start['time'],
    }


def drift_over_time(data, checkpoints=[5, 10, 15, 20, 25, 30]):
    """Calculate drift at different time points."""
    results = []

    for t in checkpoints:
        subset = [d for d in data if d['time'] <= t]
        if len(subset) < 2:
            continue

        gps_data = simulate_gps(subset)
        imu_data = simulate_imu_only(subset)
        fusion_pos = integrate_fusion_position(subset)

        end = subset[-1]
        start = subset[0]
        gt_x, gt_y, gt_z = end['gt_x'], end['gt_y'], end['gt_z']

        gps_err = math.sqrt(
            (gps_data[-1]['gps_x'] - gt_x)**2 +
            (gps_data[-1]['gps_y'] - gt_y)**2 +
            (gps_data[-1]['gps_z'] - gt_z)**2
        )

        imu_err = math.sqrt(
            (imu_data[-1]['imu_x'] - gt_x)**2 +
            (imu_data[-1]['imu_y'] - gt_y)**2 +
            (imu_data[-1]['imu_z'] - gt_z)**2
        )

        fusion_err = math.sqrt(
            (fusion_pos[-1]['fusion_x'] - gt_x)**2 +
            (fusion_pos[-1]['fusion_y'] - gt_y)**2 +
            (fusion_pos[-1]['fusion_z'] - gt_z)**2
        )

        gt_dist = math.sqrt(
            (end['gt_x'] - start['gt_x'])**2 +
            (end['gt_y'] - start['gt_y'])**2 +
            (end['gt_z'] - start['gt_z'])**2
        )

        results.append({
            'time': t,
            'gps_err': gps_err,
            'imu_err': imu_err,
            'fusion_err': fusion_err,
            'distance': gt_dist,
        })

    return results


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

    # Simulate all three methods
    random.seed(42)  # Reproducible results
    gps_data = simulate_gps(data)
    imu_data = simulate_imu_only(data)
    fusion_pos = integrate_fusion_position(data)

    metrics = compute_metrics(data, gps_data, imu_data, fusion_pos)

    print("=" * 75)
    print("3-WAY COMPARISON: GPS vs FIBER+VISION vs IMU-ONLY")
    print("=" * 75)
    print(f"  Flight duration: {metrics['duration']:.1f}s")
    print(f"  Distance traveled: {metrics['gt_distance']:.1f}m")
    print()

    print("Sensor characteristics:")
    print("  GPS:          Position noise ~3m, Velocity noise ~0.15 m/s, No drift")
    print("  Fiber+Vision: Spool noise ~0.1 m/s, Vision drift, Linear position drift")
    print("  IMU-only:     Accel bias 20mg, Quadratic position drift")
    print()

    print("-" * 75)
    print("VELOCITY RMSE")
    print("-" * 75)
    print(f"  {'Method':<20} {'RMSE (m/s)':<15} {'vs GPS':<15} {'Notes':<20}")
    print(f"  {'-'*20} {'-'*15} {'-'*15} {'-'*20}")
    print(f"  {'GPS':<20} {metrics['gps_vel_rmse']:<15.3f} {'(reference)':<15} {'Best when available':<20}")
    print(f"  {'Fiber+Vision':<20} {metrics['fusion_vel_rmse']:<15.3f} {metrics['fusion_vel_rmse']/metrics['gps_vel_rmse']:.1f}x worse      {'GPS-denied capable':<20}")
    print(f"  {'IMU-only':<20} {metrics['imu_vel_rmse']:<15.3f} {metrics['imu_vel_rmse']/metrics['gps_vel_rmse']:.1f}x worse      {'Drifts over time':<20}")
    print()

    print("-" * 75)
    print(f"POSITION ERROR @ {metrics['duration']:.0f}s")
    print("-" * 75)
    print(f"  {'Method':<20} {'Error (m)':<15} {'vs GPS':<15} {'Drift behavior':<20}")
    print(f"  {'-'*20} {'-'*15} {'-'*15} {'-'*20}")
    print(f"  {'GPS':<20} {metrics['gps_pos_err']:<15.2f} {'(reference)':<15} {'Constant (no drift)':<20}")
    print(f"  {'Fiber+Vision':<20} {metrics['fusion_pos_err']:<15.2f} {metrics['fusion_pos_err']/metrics['gps_pos_err']:.1f}x worse      {'Linear (~t)':<20}")
    print(f"  {'IMU-only':<20} {metrics['imu_pos_err']:<15.2f} {metrics['imu_pos_err']/metrics['gps_pos_err']:.1f}x worse      {'Quadratic (~t²)':<20}")
    print()

    # Drift over time
    drift_results = drift_over_time(data)

    print("-" * 75)
    print("POSITION ERROR OVER TIME")
    print("-" * 75)
    print(f"  {'Time':<8} {'GPS (m)':<12} {'Fiber (m)':<12} {'IMU (m)':<12} {'IMU/Fiber':<12}")
    print(f"  {'-'*8} {'-'*12} {'-'*12} {'-'*12} {'-'*12}")
    for r in drift_results:
        ratio = r['imu_err'] / r['fusion_err'] if r['fusion_err'] > 0 else 0
        print(f"  {r['time']:<8.0f} {r['gps_err']:<12.2f} {r['fusion_err']:<12.2f} {r['imu_err']:<12.2f} {ratio:<12.1f}x")
    print()

    print("=" * 75)
    print("SUMMARY")
    print("=" * 75)
    print()
    print("  ┌─────────────────┬────────────┬──────────────┬─────────────┐")
    print("  │ Method          │ Vel RMSE   │ Pos Error    │ Use Case    │")
    print("  ├─────────────────┼────────────┼──────────────┼─────────────┤")
    print(f"  │ GPS             │ {metrics['gps_vel_rmse']:.2f} m/s   │ {metrics['gps_pos_err']:.1f}m (const) │ Best choice │")
    print(f"  │ Fiber+Vision    │ {metrics['fusion_vel_rmse']:.2f} m/s   │ {metrics['fusion_pos_err']:.1f}m (linear)│ GPS-denied  │")
    print(f"  │ IMU-only        │ {metrics['imu_vel_rmse']:.2f} m/s   │ {metrics['imu_pos_err']:.1f}m (quad)  │ Short-term  │")
    print("  └─────────────────┴────────────┴──────────────┴─────────────┘")
    print()
    print("  Key insights:")
    print("  • GPS is best when available (no drift, ~3m accuracy)")
    print("  • Fiber+Vision is viable GPS alternative (linear drift)")
    print("  • IMU-only degrades rapidly (quadratic drift)")
    print()
    print("  For GPS-denied navigation over long distances,")
    print("  Fiber+Vision fusion significantly outperforms IMU-only.")
    print("=" * 75)


if __name__ == '__main__':
    main()
