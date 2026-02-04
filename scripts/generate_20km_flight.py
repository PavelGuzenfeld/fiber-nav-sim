#!/usr/bin/env python3
"""
Generate synthetic 20km flight data for 3-way comparison.

Simulates realistic fixed-wing flight with:
- Ground truth trajectory (straight line with minor variations)
- Fusion velocity with spool + vision sensor noise
- Typical flight parameters: 20 m/s cruise, 20 km distance
"""

import csv
import math
import random
import sys
from pathlib import Path


def generate_flight_data(
    distance_km: float = 20.0,
    cruise_speed: float = 20.0,  # m/s
    dt: float = 0.02,  # 50 Hz sampling
    spool_noise: float = 0.1,  # m/s std dev
    vision_drift_rate: float = 0.001,  # rad/s
    output_file: str = '/tmp/flight_data.csv'
):
    """Generate synthetic flight data for analysis."""

    distance_m = distance_km * 1000
    duration = distance_m / cruise_speed
    n_samples = int(duration / dt)

    print(f"Generating {distance_km} km flight data...")
    print(f"  Cruise speed: {cruise_speed} m/s")
    print(f"  Duration: {duration:.0f} s ({duration/60:.1f} min)")
    print(f"  Samples: {n_samples}")

    # Initialize state
    x, y, z = 0.0, 0.0, 50.0  # Start at 50m altitude
    vx, vy, vz = cruise_speed, 0.0, 0.0

    # Vision direction starts perfect
    vision_yaw_error = 0.0

    data = []

    for i in range(n_samples):
        t = i * dt

        # Add minor flight path variations (gentle weaving, not circling)
        # Small lateral oscillation (10m amplitude over 200s period)
        lateral_var = 5.0 * math.sin(2 * math.pi * t / 200)
        altitude_var = 2.0 * math.sin(2 * math.pi * t / 60)  # Altitude variations

        # Ground truth velocity - primarily forward with minor variations
        gt_vx = cruise_speed + random.gauss(0, 0.2)  # Forward with noise
        gt_vy = 0.5 * math.cos(2 * math.pi * t / 200) + random.gauss(0, 0.1)  # Weave
        gt_vz = 0.1 * math.cos(2 * math.pi * t / 30) + random.gauss(0, 0.05)

        # Heading is the direction of travel
        heading = math.atan2(gt_vy, gt_vx)

        # Update ground truth position
        x += gt_vx * dt
        y += gt_vy * dt
        z = 50.0 + altitude_var

        # Simulate spool sensor (scalar speed with noise)
        gt_speed = math.sqrt(gt_vx**2 + gt_vy**2 + gt_vz**2)
        spool_speed = gt_speed + random.gauss(0, spool_noise)

        # Simulate vision direction (unit vector with drift)
        vision_yaw_error += random.gauss(0, vision_drift_rate * dt)
        vision_yaw_error = max(-0.1, min(0.1, vision_yaw_error))  # Clamp drift

        # Vision direction (ground truth + error)
        vision_heading = heading + vision_yaw_error
        dir_x = math.cos(vision_heading)
        dir_y = math.sin(vision_heading)
        dir_z = gt_vz / (gt_speed + 0.001)  # Small vertical component

        # Normalize direction
        dir_mag = math.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
        dir_x /= dir_mag
        dir_y /= dir_mag
        dir_z /= dir_mag

        # Fusion output: spool_speed * direction
        slack_factor = 1.05
        fusion_vx = (spool_speed / slack_factor) * dir_x
        fusion_vy = (spool_speed / slack_factor) * dir_y
        fusion_vz = (spool_speed / slack_factor) * dir_z

        data.append({
            'time': t,
            'gt_x': x, 'gt_y': y, 'gt_z': z,
            'gt_vx': gt_vx, 'gt_vy': gt_vy, 'gt_vz': gt_vz,
            'spool_speed': spool_speed,
            'dir_x': dir_x, 'dir_y': dir_y, 'dir_z': dir_z,
            'fusion_vx': fusion_vx, 'fusion_vy': fusion_vy, 'fusion_vz': fusion_vz,
        })

    # Write to CSV
    with open(output_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=data[0].keys())
        writer.writeheader()
        writer.writerows(data)

    print(f"\nFlight data saved to: {output_file}")
    print(f"  Final position: ({x:.0f}, {y:.0f}, {z:.0f}) m")
    print(f"  Total distance: {math.sqrt(x**2 + y**2):.0f} m")

    return output_file


def simulate_landmark_correction(data, landmarks_every_m=200, correction_gain=0.8):
    """
    Simulate landmark-based position correction.

    When the plane passes a landmark, apply a position correction
    that reduces the accumulated error.
    """
    if len(data) < 2:
        return data

    corrected = []
    correction_x, correction_y, correction_z = 0.0, 0.0, 0.0
    last_landmark_x = 0
    corrections_applied = 0

    for row in data:
        gt_x = row['gt_x']

        # Check if we passed a landmark
        current_landmark = int(gt_x / landmarks_every_m) * landmarks_every_m
        if current_landmark > last_landmark_x and current_landmark > 0:
            # Simulate landmark detection - calculate error and correct
            # In real system, this would come from visual detection
            integrated_x = row.get('integrated_x', gt_x)
            integrated_y = row.get('integrated_y', row['gt_y'])
            integrated_z = row.get('integrated_z', row['gt_z'])

            error_x = gt_x - integrated_x
            error_y = row['gt_y'] - integrated_y
            error_z = row['gt_z'] - integrated_z

            correction_x += correction_gain * error_x
            correction_y += correction_gain * error_y
            correction_z += correction_gain * error_z

            last_landmark_x = current_landmark
            corrections_applied += 1

        # Apply accumulated corrections
        corrected_row = row.copy()
        corrected_row['corrected_vx'] = row['fusion_vx']
        corrected_row['corrected_vy'] = row['fusion_vy']
        corrected_row['corrected_vz'] = row['fusion_vz']
        corrected_row['correction_x'] = correction_x
        corrected_row['correction_y'] = correction_y
        corrected_row['correction_z'] = correction_z
        corrected.append(corrected_row)

    print(f"  Landmark corrections applied: {corrections_applied}")
    return corrected


def main():
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    output = sys.argv[2] if len(sys.argv) > 2 else '/tmp/flight_data.csv'
    use_correction = '--correction' in sys.argv

    random.seed(42)  # Reproducible
    generate_flight_data(distance_km=distance, output_file=output)

    if use_correction:
        print("\nApplying landmark correction simulation...")
        import csv
        with open(output, 'r') as f:
            data = list(csv.DictReader(f))
            data = [{k: float(v) for k, v in row.items()} for row in data]

        corrected = simulate_landmark_correction(data)

        output_corrected = output.replace('.csv', '_corrected.csv')
        with open(output_corrected, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=corrected[0].keys())
            writer.writeheader()
            writer.writerows(corrected)
        print(f"Corrected data saved to: {output_corrected}")


if __name__ == '__main__':
    main()
