#!/usr/bin/env python3
"""Unit tests for analysis scripts."""
import sys
import math
import unittest
from pathlib import Path

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent))

from analyze_flight import compute_velocity_rmse, compute_position_drift, compute_statistics
from compare_three_way import simulate_gps, simulate_imu_only, integrate_fusion_position


class TestAnalyzeFlight(unittest.TestCase):
    """Tests for analyze_flight.py functions."""

    def setUp(self):
        """Create sample flight data."""
        self.data = [
            {'time': 0.0, 'gt_x': 0, 'gt_y': 0, 'gt_z': 0,
             'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
             'fusion_vx': 10.1, 'fusion_vy': 0.05, 'fusion_vz': -0.02},
            {'time': 1.0, 'gt_x': 10, 'gt_y': 0, 'gt_z': 0,
             'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
             'fusion_vx': 9.9, 'fusion_vy': -0.03, 'fusion_vz': 0.01},
            {'time': 2.0, 'gt_x': 20, 'gt_y': 0, 'gt_z': 0,
             'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
             'fusion_vx': 10.0, 'fusion_vy': 0.02, 'fusion_vz': 0.0},
        ]

    def test_velocity_rmse_perfect_match(self):
        """Test RMSE is zero when fusion matches ground truth exactly."""
        perfect_data = [
            {'time': 0.0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
             'fusion_vx': 10, 'fusion_vy': 0, 'fusion_vz': 0}
        ]
        result = compute_velocity_rmse(perfect_data)
        self.assertAlmostEqual(result['fusion_gt'], 0.0, places=6)

    def test_velocity_rmse_with_error(self):
        """Test RMSE computation with known error."""
        # Single sample with 1 m/s error in X
        error_data = [
            {'time': 0.0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
             'fusion_vx': 11, 'fusion_vy': 0, 'fusion_vz': 0}
        ]
        result = compute_velocity_rmse(error_data)
        self.assertAlmostEqual(result['fusion_gt'], 1.0, places=6)

    def test_ekf_not_available(self):
        """Test that EKF is marked unavailable when all zeros."""
        result = compute_velocity_rmse(self.data)
        self.assertFalse(result['ekf_available'])

    def test_statistics_computation(self):
        """Test statistics calculation."""
        result = compute_statistics(self.data)
        self.assertAlmostEqual(result['duration'], 2.0, places=1)
        self.assertEqual(result['samples'], 3)
        self.assertAlmostEqual(result['avg_speed'], 10.0, places=1)

    def test_position_drift_computation(self):
        """Test position drift calculation."""
        result = compute_position_drift(self.data)
        self.assertGreater(result['distance_traveled'], 0)
        # Drift should be small for nearly-matching velocities
        self.assertLess(result['total'], 1.0)


class TestCompareThreeWay(unittest.TestCase):
    """Tests for compare_three_way.py functions."""

    def setUp(self):
        """Create sample data."""
        self.data = []
        for i in range(100):
            t = i * 0.1
            self.data.append({
                'time': t,
                'gt_x': 10 * t, 'gt_y': 0, 'gt_z': 0,
                'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
                'fusion_vx': 10.0 + (i % 3 - 1) * 0.1,
                'fusion_vy': 0.0,
                'fusion_vz': 0.0,
            })

    def test_gps_simulation_adds_noise(self):
        """Test GPS simulation adds position noise."""
        gps_data = simulate_gps(self.data, pos_noise=3.0, vel_noise=0.15)

        # GPS position should differ from ground truth
        errors = []
        for gps, gt in zip(gps_data, self.data):
            err = abs(gps['gps_x'] - gt['gt_x'])
            errors.append(err)

        avg_error = sum(errors) / len(errors)
        self.assertGreater(avg_error, 0.5)  # Should have some noise
        self.assertLess(avg_error, 10.0)    # But not too much

    def test_imu_simulation_drifts(self):
        """Test IMU simulation accumulates drift."""
        imu_data = simulate_imu_only(self.data, accel_bias=0.02, accel_noise=0.1)

        # IMU error should grow over time
        early_err = abs(imu_data[10]['imu_x'] - self.data[10]['gt_x'])
        late_err = abs(imu_data[-1]['imu_x'] - self.data[-1]['gt_x'])

        self.assertGreater(late_err, early_err)

    def test_fusion_integration(self):
        """Test fusion position integration."""
        fusion_pos = integrate_fusion_position(self.data)

        self.assertEqual(len(fusion_pos), len(self.data))
        # Final position should be close to ground truth
        final_err = abs(fusion_pos[-1]['fusion_x'] - self.data[-1]['gt_x'])
        self.assertLess(final_err, 5.0)  # Within 5m over 10s


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error handling."""

    def test_empty_data(self):
        """Test handling of empty data."""
        result = compute_velocity_rmse([])
        self.assertEqual(result['fusion_gt'], 0.0)
        self.assertFalse(result['ekf_available'])

    def test_single_sample(self):
        """Test handling of single sample."""
        single = [{'time': 0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
                   'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
                   'fusion_vx': 10, 'fusion_vy': 0, 'fusion_vz': 0}]
        result = compute_velocity_rmse(single)
        self.assertAlmostEqual(result['fusion_gt'], 0.0, places=6)


if __name__ == '__main__':
    unittest.main()
