# Copyright 2026 Pavel Guzenfeld
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
Dispatcher launch file for fiber navigation simulation.

Selects the simulation backend (Gazebo or O3DE) via the 'backend' argument,
then delegates to the appropriate backend-specific launch file.

All existing launch commands continue to work unchanged:
  ros2 launch fiber_nav_bringup simulation.launch.py
  ros2 launch fiber_nav_bringup simulation.launch.py backend:=gazebo
  ros2 launch fiber_nav_bringup simulation.launch.py backend:=o3de

Both backends include sensors.launch.py for shared sensor/fusion nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('fiber_nav_bringup')

    # Backend selection
    backend_arg = DeclareLaunchArgument(
        'backend', default_value='gazebo',
        choices=['gazebo', 'o3de', 'colosseum'],
        description='Simulation backend: gazebo (default), o3de, or colosseum')

    # Common arguments (forwarded to both backends)
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run simulation without GUI')

    use_px4_arg = DeclareLaunchArgument(
        'use_px4', default_value='false',
        description='PX4 controls motors (disables flight_controller and mock att)')

    world_arg = DeclareLaunchArgument(
        'world', default_value='canyon_harmonic',
        description='World file name (without .sdf extension)')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='canyon_world',
        description='Gazebo internal world name (must match <world name="..."> in SDF)')

    auto_fly_arg = DeclareLaunchArgument(
        'auto_fly', default_value='false',
        description='Automatically apply thrust to fly')

    target_altitude_arg = DeclareLaunchArgument(
        'target_altitude', default_value='50.0',
        description='Target altitude (m) for stabilized flight controller')

    target_speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='15.0',
        description='Target forward speed (m/s) for stabilized flight controller')

    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch Foxglove bridge for web visualization')

    mission_config_arg = DeclareLaunchArgument(
        'mission_config', default_value='',
        description='Optional mission-specific YAML config overlay for EKF/TERCOM nodes')

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='32.5')

    # Common launch arguments to forward
    common_args = {
        'headless': LaunchConfiguration('headless'),
        'use_px4': LaunchConfiguration('use_px4'),
        'world': LaunchConfiguration('world'),
        'world_name': LaunchConfiguration('world_name'),
        'auto_fly': LaunchConfiguration('auto_fly'),
        'target_altitude': LaunchConfiguration('target_altitude'),
        'target_speed': LaunchConfiguration('target_speed'),
        'foxglove': LaunchConfiguration('foxglove'),
        'mission_config': LaunchConfiguration('mission_config'),
        'spawn_x': LaunchConfiguration('spawn_x'),
        'spawn_y': LaunchConfiguration('spawn_y'),
        'spawn_z': LaunchConfiguration('spawn_z'),
    }

    # Gazebo backend
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'gazebo_simulation.launch.py'])
        ),
        launch_arguments=common_args.items(),
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('backend'), 'gazebo'))
    )

    # O3DE backend
    o3de_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'o3de_simulation.launch.py'])
        ),
        launch_arguments=common_args.items(),
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('backend'), 'o3de'))
    )

    # Colosseum backend
    colosseum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'colosseum_simulation.launch.py'])
        ),
        launch_arguments=common_args.items(),
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('backend'), 'colosseum'))
    )

    return LaunchDescription([
        # Arguments
        backend_arg,
        headless_arg,
        use_px4_arg,
        world_arg,
        world_name_arg,
        auto_fly_arg,
        target_altitude_arg,
        target_speed_arg,
        foxglove_arg,
        mission_config_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,

        # Backend dispatch
        gazebo_launch,
        o3de_launch,
        colosseum_launch,
    ])
