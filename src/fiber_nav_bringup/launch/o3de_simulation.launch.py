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
O3DE simulation backend for fiber navigation.

Launches O3DE-specific components:
- Xvfb virtual framebuffer (headless display for O3DE)
- O3DE GameLauncher with Vulkan rendering (TerrainLevel)
- (No ros_gz_bridge needed — O3DE ROS 2 Gem publishes directly)

Then includes sensors.launch.py for shared sensor/fusion nodes.

O3DE ROS 2 Gem publishes directly to ROS 2 topics:
  /camera_image_color (sensor_msgs/Image) — 1920x1080 @ 30Hz
  /camera_image_depth (sensor_msgs/Image)
  /camera_info (sensor_msgs/CameraInfo)
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments (same interface as gazebo_simulation.launch.py for dispatcher compat)
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run O3DE without native window (always true for now)')

    use_px4_arg = DeclareLaunchArgument(
        'use_px4', default_value='false',
        description='PX4 controls motors')

    world_arg = DeclareLaunchArgument(
        'world', default_value='terrain_world',
        description='World file (unused by O3DE — level is set in O3DE project)')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='terrain_world',
        description='World name (passed to sensors for stabilized flight controller)')

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
        description='Optional mission-specific YAML config overlay')

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='50.0')

    pkg_bringup = FindPackageShare('fiber_nav_bringup')

    # Start Xvfb (O3DE requires XCB display even in console/headless mode)
    xvfb = ExecuteProcess(
        cmd=['Xvfb', ':99', '-screen', '0', '1920x1080x24', '+extension', 'GLX'],
        output='log',
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    # O3DE GameLauncher
    project_path = os.environ.get('PROJECT_PATH', '/opt/HeadlessTest')
    project_name = os.environ.get('PROJECT_NAME', 'HeadlessTest')
    launcher_path = f'{project_path}/build/linux/bin/profile/{project_name}.GameLauncher'

    o3de_launcher = ExecuteProcess(
        cmd=[
            launcher_path,
            '-console-mode',
            '--regset=/O3DE/Atom/Bootstrap/CreateNativeWindow=false',
        ],
        output='screen',
        additional_env={'DISPLAY': ':99'},
    )

    o3de_info = LogInfo(
        msg='O3DE backend: TerrainLevel with quadtailsitter + PX4Bridge')

    # Include shared sensor nodes (delayed 10s to let O3DE initialize)
    sensors_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_bringup, 'launch', 'sensors.launch.py'])
                ),
                launch_arguments={
                    'use_px4': LaunchConfiguration('use_px4'),
                    'auto_fly': LaunchConfiguration('auto_fly'),
                    'world_name': LaunchConfiguration('world_name'),
                    'target_altitude': LaunchConfiguration('target_altitude'),
                    'target_speed': LaunchConfiguration('target_speed'),
                    'foxglove': LaunchConfiguration('foxglove'),
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        # Arguments
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

        # O3DE display
        xvfb,

        # O3DE launcher
        o3de_info,
        o3de_launcher,

        # Shared sensors + fusion + foxglove (delayed)
        sensors_launch,
    ])
