#!/usr/bin/env python3
"""
Compute navigation metrics from trajectory data.
"""

import argparse
import numpy as np
import pandas as pd
import json
from pathlib import Path


def load_trajectory(filepath: str) -> pd.DataFrame:
    """Load trajectory from CSV file."""
    return pd.read_csv(filepath)


def compute_path_length(df: pd.DataFrame) -> float:
    """Compute total path length."""
    pos = df[['x', 'y', 'z']].values
    diffs = np.diff(pos, axis=0)
    distances = np.linalg.norm(diffs, axis=1)
    return float(np.sum(distances))


def compute_drift_per_1000m(gt: pd.DataFrame, est: pd.DataFrame) -> float:
    """Compute position drift per 1000m traveled."""
    # Interpolate estimated to ground truth timestamps
    est_final = {}
    for col in ['x', 'y', 'z']:
        est_final[col] = np.interp(gt['timestamp'].iloc[-1], est['timestamp'], est[col])

    final_error = np.sqrt(
        (gt['x'].iloc[-1] - est_final['x'])**2 +
        (gt['y'].iloc[-1] - est_final['y'])**2 +
        (gt['z'].iloc[-1] - est_final['z'])**2
    )

    path_length = compute_path_length(gt)
    return float((final_error / path_length) * 1000)


def compute_velocity_rmse(gt: pd.DataFrame, est: pd.DataFrame) -> float:
    """Compute velocity RMSE."""
    # Interpolate estimated to ground truth timestamps
    est_interp = {}
    for col in ['vx', 'vy', 'vz']:
        est_interp[col] = np.interp(gt['timestamp'], est['timestamp'], est[col])

    error_sq = (
        (gt['vx'].values - est_interp['vx'])**2 +
        (gt['vy'].values - est_interp['vy'])**2 +
        (gt['vz'].values - est_interp['vz'])**2
    )
    return float(np.sqrt(np.mean(error_sq)))


def compute_max_error(gt: pd.DataFrame, est: pd.DataFrame) -> float:
    """Compute maximum position error."""
    est_interp = {}
    for col in ['x', 'y', 'z']:
        est_interp[col] = np.interp(gt['timestamp'], est['timestamp'], est[col])

    error = np.sqrt(
        (gt['x'].values - est_interp['x'])**2 +
        (gt['y'].values - est_interp['y'])**2 +
        (gt['z'].values - est_interp['z'])**2
    )
    return float(np.max(error))


def compute_mean_error(gt: pd.DataFrame, est: pd.DataFrame) -> float:
    """Compute mean position error."""
    est_interp = {}
    for col in ['x', 'y', 'z']:
        est_interp[col] = np.interp(gt['timestamp'], est['timestamp'], est[col])

    error = np.sqrt(
        (gt['x'].values - est_interp['x'])**2 +
        (gt['y'].values - est_interp['y'])**2 +
        (gt['z'].values - est_interp['z'])**2
    )
    return float(np.mean(error))


def compute_all_metrics(gt: pd.DataFrame, est: pd.DataFrame) -> dict:
    """Compute all metrics and return as dictionary."""
    duration = gt['timestamp'].iloc[-1] - gt['timestamp'].iloc[0]
    path_length = compute_path_length(gt)

    metrics = {
        'flight': {
            'duration_s': float(duration),
            'path_length_m': path_length,
            'avg_speed_m_s': path_length / duration,
        },
        'position': {
            'drift_per_1000m': compute_drift_per_1000m(gt, est),
            'max_error_m': compute_max_error(gt, est),
            'mean_error_m': compute_mean_error(gt, est),
        },
        'velocity': {
            'rmse_m_s': compute_velocity_rmse(gt, est),
        },
        'pass_fail': {
            'drift_under_10m_km': compute_drift_per_1000m(gt, est) < 10.0,
            'velocity_rmse_under_0_5': compute_velocity_rmse(gt, est) < 0.5,
        }
    }

    return metrics


def main():
    parser = argparse.ArgumentParser(description='Compute navigation metrics')
    parser.add_argument('--gt', required=True, help='Ground truth CSV file')
    parser.add_argument('--est', required=True, help='Estimated trajectory CSV file')
    parser.add_argument('--output', help='Output JSON file (optional)')
    parser.add_argument('--format', choices=['json', 'text'], default='text',
                        help='Output format')
    args = parser.parse_args()

    # Load data
    gt = load_trajectory(args.gt)
    est = load_trajectory(args.est)

    # Compute metrics
    metrics = compute_all_metrics(gt, est)

    # Output
    if args.format == 'json':
        output = json.dumps(metrics, indent=2)
    else:
        output = f"""
Navigation Metrics
==================

Flight:
  Duration: {metrics['flight']['duration_s']:.1f} s
  Path Length: {metrics['flight']['path_length_m']:.1f} m
  Avg Speed: {metrics['flight']['avg_speed_m_s']:.1f} m/s

Position Accuracy:
  Drift per 1000m: {metrics['position']['drift_per_1000m']:.2f} m/km
  Max Error: {metrics['position']['max_error_m']:.2f} m
  Mean Error: {metrics['position']['mean_error_m']:.2f} m

Velocity Accuracy:
  RMSE: {metrics['velocity']['rmse_m_s']:.3f} m/s

Pass/Fail:
  Drift < 10 m/km: {'PASS' if metrics['pass_fail']['drift_under_10m_km'] else 'FAIL'}
  Vel RMSE < 0.5 m/s: {'PASS' if metrics['pass_fail']['velocity_rmse_under_0_5'] else 'FAIL'}
"""

    print(output)

    if args.output:
        with open(args.output, 'w') as f:
            if args.format == 'json':
                f.write(output)
            else:
                f.write(output)
        print(f'\nSaved to: {args.output}')


if __name__ == '__main__':
    main()
