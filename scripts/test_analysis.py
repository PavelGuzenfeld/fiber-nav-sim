#!/usr/bin/env python3
"""Unit tests for analysis scripts."""
import sys
import math
import unittest
from pathlib import Path

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent))

from analyze_flight import compute_speed_rmse, compute_ekf_position_drift, compute_statistics, compute_fusion_position_error


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

    def test_speed_rmse_perfect_match(self):
        """Test RMSE is zero when fusion matches ground truth exactly."""
        perfect_data = [
            {'time': 0.0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
             'fusion_vx': 10, 'fusion_vy': 0, 'fusion_vz': 0}
        ]
        result = compute_speed_rmse(perfect_data)
        self.assertAlmostEqual(result['fusion_gt'], 0.0, places=6)

    def test_speed_rmse_with_error(self):
        """Test RMSE computation with known error."""
        # Single sample with 1 m/s error in X (scalar: |11|-|10| = 1)
        error_data = [
            {'time': 0.0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
             'fusion_vx': 11, 'fusion_vy': 0, 'fusion_vz': 0}
        ]
        result = compute_speed_rmse(error_data)
        self.assertAlmostEqual(result['fusion_gt'], 1.0, places=6)

    def test_fusion_available(self):
        """Test that fusion is marked available when non-zero."""
        result = compute_speed_rmse(self.data)
        self.assertTrue(result['fusion_available'])

    def test_statistics_computation(self):
        """Test statistics calculation."""
        result = compute_statistics(self.data)
        self.assertAlmostEqual(result['duration'], 2.0, places=1)
        self.assertEqual(result['samples'], 3)
        self.assertAlmostEqual(result['avg_speed'], 10.0, places=1)

    def test_position_drift_computation(self):
        """Test position drift calculation."""
        # Create data with gt_rel and ekf_rel fields (as produced by align_origins)
        drift_data = [
            {'time': 0.0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'gt_rel_x': 0, 'gt_rel_y': 0, 'gt_rel_z': 0,
             'ekf_rel_x': 0, 'ekf_rel_y': 0, 'ekf_rel_z': 0},
            {'time': 1.0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'gt_rel_x': 10, 'gt_rel_y': 0, 'gt_rel_z': 0,
             'ekf_rel_x': 10.5, 'ekf_rel_y': 0, 'ekf_rel_z': 0},
            {'time': 2.0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
             'gt_rel_x': 20, 'gt_rel_y': 0, 'gt_rel_z': 0,
             'ekf_rel_x': 21.0, 'ekf_rel_y': 0, 'ekf_rel_z': 0},
        ]
        result = compute_ekf_position_drift(drift_data)
        self.assertGreater(result['distance_traveled'], 0)
        # Final EKF position error = 1.0m over 20m traveled
        self.assertAlmostEqual(result['total'], 1.0, places=1)



class TestFusionPositionError(unittest.TestCase):
    """Tests for compute_fusion_position_error function."""

    def test_perfect_position(self):
        """Test zero error when fusion position matches GT."""
        data = [
            {'fusion_px': 10.0, 'fusion_py': 0.0, 'gt_rel_x': 10.0, 'gt_rel_y': 0.0},
            {'fusion_px': 20.0, 'fusion_py': 0.0, 'gt_rel_x': 20.0, 'gt_rel_y': 0.0},
        ]
        result = compute_fusion_position_error(data)
        self.assertTrue(result['available'])
        self.assertAlmostEqual(result['mean'], 0.0, places=6)

    def test_known_error(self):
        """Test error computation with known offset."""
        data = [
            {'fusion_px': 11.0, 'fusion_py': 0.0, 'gt_rel_x': 10.0, 'gt_rel_y': 0.0},
            {'fusion_px': 23.0, 'fusion_py': 0.0, 'gt_rel_x': 20.0, 'gt_rel_y': 0.0},
        ]
        result = compute_fusion_position_error(data)
        self.assertTrue(result['available'])
        self.assertAlmostEqual(result['mean'], 2.0, places=6)  # (1+3)/2
        self.assertAlmostEqual(result['max'], 3.0, places=6)
        self.assertAlmostEqual(result['final'], 3.0, places=6)

    def test_nan_skipped(self):
        """Test that NaN fusion positions are skipped."""
        data = [
            {'fusion_px': float('nan'), 'fusion_py': float('nan'), 'gt_rel_x': 0, 'gt_rel_y': 0},
            {'fusion_px': 10.0, 'fusion_py': 0.0, 'gt_rel_x': 10.0, 'gt_rel_y': 0.0},
        ]
        result = compute_fusion_position_error(data)
        self.assertTrue(result['available'])
        self.assertEqual(result['count'], 1)
        self.assertAlmostEqual(result['mean'], 0.0, places=6)

    def test_all_nan(self):
        """Test all NaN returns not available."""
        data = [
            {'fusion_px': float('nan'), 'fusion_py': float('nan'), 'gt_rel_x': 0, 'gt_rel_y': 0},
        ]
        result = compute_fusion_position_error(data)
        self.assertFalse(result['available'])

    def test_no_fusion_columns(self):
        """Test data without fusion position columns."""
        data = [{'gt_rel_x': 0, 'gt_rel_y': 0}]
        result = compute_fusion_position_error(data)
        self.assertFalse(result['available'])


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error handling."""

    def test_empty_data(self):
        """Test handling of empty data."""
        result = compute_speed_rmse([])
        self.assertEqual(result['fusion_gt'], 0.0)
        self.assertFalse(result['fusion_available'])

    def test_single_sample(self):
        """Test handling of single sample."""
        single = [{'time': 0, 'gt_vx': 10, 'gt_vy': 0, 'gt_vz': 0,
                   'ekf_vx': 0, 'ekf_vy': 0, 'ekf_vz': 0,
                   'fusion_vx': 10, 'fusion_vy': 0, 'fusion_vz': 0}]
        result = compute_speed_rmse(single)
        self.assertAlmostEqual(result['fusion_gt'], 0.0, places=6)


if __name__ == '__main__':
    unittest.main()
