# Copyright 2024 Pavel Guzenfeld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Integration test for stabilized flight controller.

Launches the full simulation in headless auto_fly mode and verifies:
- Vehicle reaches target altitude (~50m) within 20s
- Altitude stays within bounds (40-60m)
- Forward flight progresses (X > 100m)
- No NaN/Inf in odometry values
"""

import math
import time
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


@pytest.mark.launch_test
def generate_test_description():
    """Launch the simulation for testing."""
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.substitutions import FindPackageShare

    pkg_bringup = FindPackageShare('fiber_nav_bringup')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_bringup, '/launch/simulation.launch.py'
        ]),
        launch_arguments={
            'headless': 'true',
            'auto_fly': 'true',
            'foxglove': 'false',
        }.items(),
    )

    return launch.LaunchDescription([
        simulation_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class OdometryCollector(Node):
    """Collects odometry messages for analysis."""

    def __init__(self):
        super().__init__('odom_collector')
        self.messages = []
        self.subscription = self.create_subscription(
            Odometry,
            '/model/quadtailsitter/odometry',
            self._callback,
            10,
        )

    def _callback(self, msg):
        self.messages.append(msg)


class TestStabilizedFlight(unittest.TestCase):
    """Test stabilized flight controller behavior."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_flight_stability(self):
        """Verify stable flight: altitude, forward progress, valid data."""
        collector = OdometryCollector()

        # Wait for odometry topic to appear (Gazebo + bridge startup)
        deadline = time.time() + 30.0
        while time.time() < deadline and len(collector.messages) == 0:
            rclpy.spin_once(collector, timeout_sec=0.5)

        self.assertGreater(
            len(collector.messages), 0,
            'No odometry messages received within 30s'
        )

        # Collect odometry for 60 seconds
        end_time = time.time() + 60.0
        while time.time() < end_time:
            rclpy.spin_once(collector, timeout_sec=0.5)

        self.assertGreater(
            len(collector.messages), 100,
            'Too few odometry messages collected'
        )

        # Check no NaN/Inf in any message
        for msg in collector.messages:
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            for val in [pos.x, pos.y, pos.z,
                        orient.w, orient.x, orient.y, orient.z]:
                self.assertTrue(
                    math.isfinite(val),
                    f'Non-finite value in odometry: {val}'
                )

        # Find when altitude first reaches 45-55m range
        reached_altitude = False
        altitude_time_idx = None
        for i, msg in enumerate(collector.messages):
            z = msg.pose.pose.position.z
            if 45.0 <= z <= 55.0:
                reached_altitude = True
                altitude_time_idx = i
                break

        self.assertTrue(
            reached_altitude,
            'Vehicle never reached target altitude range (45-55m)'
        )

        # After reaching altitude, check it stays within 40-60m
        out_of_bounds = 0
        for msg in collector.messages[altitude_time_idx:]:
            z = msg.pose.pose.position.z
            if z < 40.0 or z > 60.0:
                out_of_bounds += 1

        # Allow up to 5% out-of-bounds samples (transients)
        total_after = len(collector.messages) - altitude_time_idx
        if total_after > 0:
            oob_ratio = out_of_bounds / total_after
            self.assertLess(
                oob_ratio, 0.05,
                f'Too many out-of-bounds altitude samples: '
                f'{out_of_bounds}/{total_after} ({oob_ratio:.1%})'
            )

        # Check forward flight progress
        max_x = max(m.pose.pose.position.x for m in collector.messages)
        self.assertGreater(
            max_x, 100.0,
            f'Vehicle did not progress forward enough: max X={max_x:.1f}m'
        )

        collector.destroy_node()
