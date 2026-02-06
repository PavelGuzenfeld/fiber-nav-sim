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

"""
Launch full fiber navigation simulation stack.

Combines:
- Gazebo Harmonic simulation
- PX4 SITL autopilot
- MicroXRCE-DDS agent
- Sensor simulators
- Fiber vision fusion
- Optional custom flight mode

This is the complete simulation for EKF integration testing.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='canyon_harmonic',
        description='World file name'
    )

    use_custom_mode_arg = DeclareLaunchArgument(
        'use_custom_mode',
        default_value='false',
        description='Launch custom flight mode node (hold or canyon mission)'
    )

    custom_mode_arg = DeclareLaunchArgument(
        'custom_mode',
        default_value='hold',
        description='Which custom mode to launch: hold or canyon'
    )

    # Package path
    pkg_bringup = FindPackageShare('fiber_nav_bringup')

    # Gazebo simulation launch (with use_px4=true)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'world': LaunchConfiguration('world'),
            'use_px4': 'true',  # Use PX4 instead of mock attitude
        }.items()
    )

    # PX4 SITL launch (delayed to let Gazebo start first)
    px4_launch = TimerAction(
        period=3.0,  # Wait for Gazebo to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_bringup, 'launch', 'px4_sitl.launch.py']
                    )
                )
            )
        ]
    )

    # Custom flight mode (delayed to let PX4 + DDS come up)
    custom_mode_launch = TimerAction(
        period=15.0,  # Wait for PX4 + DDS to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_bringup, 'launch', 'custom_mode.launch.py']
                    )
                ),
                launch_arguments={
                    'mode': LaunchConfiguration('custom_mode'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('use_custom_mode'))
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        headless_arg,
        world_arg,
        use_custom_mode_arg,
        custom_mode_arg,

        # Gazebo + sensors + fusion
        simulation_launch,

        # PX4 SITL (delayed)
        px4_launch,

        # Custom mode (delayed, optional)
        custom_mode_launch,
    ])
