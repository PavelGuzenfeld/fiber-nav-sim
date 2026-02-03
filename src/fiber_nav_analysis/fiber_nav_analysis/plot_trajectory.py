#!/usr/bin/env python3
"""
Plot recorded trajectories and compute drift metrics.
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path


def load_trajectory(filepath: str) -> pd.DataFrame:
    """Load trajectory from CSV file."""
    df = pd.read_csv(filepath)
    return df


def compute_path_length(df: pd.DataFrame) -> float:
    """Compute total path length from position data."""
    pos = df[['x', 'y', 'z']].values
    diffs = np.diff(pos, axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    return np.sum(distances)


def compute_position_error(gt: pd.DataFrame, est: pd.DataFrame) -> np.ndarray:
    """Compute position error at each timestamp."""
    # Interpolate estimated to ground truth timestamps
    est_interp = pd.DataFrame()
    for col in ['x', 'y', 'z']:
        est_interp[col] = np.interp(gt['timestamp'], est['timestamp'], est[col])

    # Compute error
    error = np.sqrt(
        (gt['x'] - est_interp['x'])**2 +
        (gt['y'] - est_interp['y'])**2 +
        (gt['z'] - est_interp['z'])**2
    )
    return error.values


def compute_velocity_rmse(gt: pd.DataFrame, est: pd.DataFrame) -> float:
    """Compute velocity RMSE."""
    est_interp = pd.DataFrame()
    for col in ['vx', 'vy', 'vz']:
        est_interp[col] = np.interp(gt['timestamp'], est['timestamp'], est[col])

    error_sq = (
        (gt['vx'] - est_interp['vx'])**2 +
        (gt['vy'] - est_interp['vy'])**2 +
        (gt['vz'] - est_interp['vz'])**2
    )
    return np.sqrt(np.mean(error_sq))


def drift_per_1000m(gt: pd.DataFrame, est: pd.DataFrame) -> float:
    """Compute position drift per 1000m traveled."""
    path_length = compute_path_length(gt)
    final_error = compute_position_error(gt, est)[-1]
    return (final_error / path_length) * 1000


def plot_2d_trajectory(gt: pd.DataFrame, est: pd.DataFrame, output_path: str = None):
    """Plot 2D trajectory comparison (X-Y plane)."""
    fig, ax = plt.subplots(figsize=(12, 8))

    ax.plot(gt['x'], gt['y'], 'b-', label='Ground Truth', linewidth=2)
    ax.plot(est['x'], est['y'], 'r--', label='Estimated', linewidth=2)

    ax.set_xlabel('X (North) [m]')
    ax.set_ylabel('Y (East) [m]')
    ax.set_title('2D Trajectory Comparison')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {output_path}')
    else:
        plt.show()

    plt.close()


def plot_3d_trajectory(gt: pd.DataFrame, est: pd.DataFrame, output_path: str = None):
    """Plot 3D trajectory comparison."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(gt['x'], gt['y'], -gt['z'], 'b-', label='Ground Truth', linewidth=2)
    ax.plot(est['x'], est['y'], -est['z'], 'r--', label='Estimated', linewidth=2)

    ax.set_xlabel('X (North) [m]')
    ax.set_ylabel('Y (East) [m]')
    ax.set_zlabel('Altitude [m]')
    ax.set_title('3D Trajectory Comparison')
    ax.legend()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {output_path}')
    else:
        plt.show()

    plt.close()


def plot_error_over_distance(gt: pd.DataFrame, est: pd.DataFrame, output_path: str = None):
    """Plot position error vs distance traveled."""
    fig, ax = plt.subplots(figsize=(12, 6))

    # Compute cumulative distance
    pos = gt[['x', 'y', 'z']].values
    diffs = np.diff(pos, axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    cumulative_distance = np.concatenate([[0], np.cumsum(distances)])

    # Compute error at each point
    error = compute_position_error(gt, est)

    ax.plot(cumulative_distance, error, 'b-', linewidth=2)
    ax.set_xlabel('Distance Traveled [m]')
    ax.set_ylabel('Position Error [m]')
    ax.set_title('Position Error vs Distance Traveled')
    ax.grid(True)

    # Add drift rate annotation
    drift = drift_per_1000m(gt, est)
    ax.annotate(f'Drift: {drift:.2f} m/km',
                xy=(0.95, 0.95), xycoords='axes fraction',
                ha='right', va='top',
                fontsize=12, bbox=dict(boxstyle='round', facecolor='wheat'))

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {output_path}')
    else:
        plt.show()

    plt.close()


def plot_velocity_comparison(gt: pd.DataFrame, est: pd.DataFrame, output_path: str = None):
    """Plot velocity comparison over time."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    t_gt = gt['timestamp'] - gt['timestamp'].iloc[0]
    t_est = est['timestamp'] - est['timestamp'].iloc[0]

    for i, (col, label) in enumerate([('vx', 'North'), ('vy', 'East'), ('vz', 'Down')]):
        axes[i].plot(t_gt, gt[col], 'b-', label='Ground Truth', alpha=0.7)
        axes[i].plot(t_est, est[col], 'r-', label='Estimated', alpha=0.7)
        axes[i].set_ylabel(f'{label} Vel [m/s]')
        axes[i].legend()
        axes[i].grid(True)

    axes[-1].set_xlabel('Time [s]')
    axes[0].set_title('Velocity Comparison')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {output_path}')
    else:
        plt.show()

    plt.close()


def generate_report(gt: pd.DataFrame, est: pd.DataFrame) -> str:
    """Generate text report with metrics."""
    path_length = compute_path_length(gt)
    drift = drift_per_1000m(gt, est)
    vel_rmse = compute_velocity_rmse(gt, est)
    final_error = compute_position_error(gt, est)[-1]

    report = f"""
=== Fiber Navigation Simulation Report ===

Flight Statistics:
  Total Distance: {path_length:.1f} m
  Duration: {gt['timestamp'].iloc[-1] - gt['timestamp'].iloc[0]:.1f} s

Position Accuracy:
  Final Position Error: {final_error:.2f} m
  Drift per 1000m: {drift:.2f} m/km

Velocity Accuracy:
  Velocity RMSE: {vel_rmse:.3f} m/s

Target Metrics:
  Drift < 10 m/km: {'PASS' if drift < 10 else 'FAIL'}
  Vel RMSE < 0.5 m/s: {'PASS' if vel_rmse < 0.5 else 'FAIL'}
"""
    return report


def main():
    parser = argparse.ArgumentParser(description='Plot fiber navigation trajectories')
    parser.add_argument('--gt', required=True, help='Ground truth CSV file')
    parser.add_argument('--est', required=True, help='Estimated trajectory CSV file')
    parser.add_argument('--output-dir', default='.', help='Output directory for plots')
    parser.add_argument('--show', action='store_true', help='Show plots interactively')
    args = parser.parse_args()

    # Load data
    gt = load_trajectory(args.gt)
    est = load_trajectory(args.est)

    print(f'Loaded GT: {len(gt)} samples')
    print(f'Loaded EST: {len(est)} samples')

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate plots
    output_path = None if args.show else str(output_dir / 'trajectory_2d.png')
    plot_2d_trajectory(gt, est, output_path)

    output_path = None if args.show else str(output_dir / 'trajectory_3d.png')
    plot_3d_trajectory(gt, est, output_path)

    output_path = None if args.show else str(output_dir / 'error_vs_distance.png')
    plot_error_over_distance(gt, est, output_path)

    output_path = None if args.show else str(output_dir / 'velocity_comparison.png')
    plot_velocity_comparison(gt, est, output_path)

    # Generate report
    report = generate_report(gt, est)
    print(report)

    report_path = output_dir / 'report.txt'
    with open(report_path, 'w') as f:
        f.write(report)
    print(f'Saved: {report_path}')


if __name__ == '__main__':
    main()
