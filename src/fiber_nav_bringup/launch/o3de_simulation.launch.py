# Copyright 2026 Pavel Guzenfeld — All rights reserved.
# PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
# Version: 0.0.1

"""
O3DE simulation backend for fiber navigation.

Launches O3DE-specific components:
- Xvfb virtual framebuffer (headless display for O3DE)
- O3DE GameLauncher with Vulkan rendering
- (No ros_gz_bridge needed — O3DE ROS 2 Gem publishes directly)

Then includes sensors.launch.py for shared sensor/fusion nodes.

NOTE: This is a placeholder. Full O3DE integration will be completed
during Phases 2-4 of the O3DE migration plan.
"""

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

    # These Gazebo-specific args are accepted but unused by O3DE backend
    world_arg = DeclareLaunchArgument(
        'world', default_value='canyon_harmonic',
        description='World file (unused by O3DE — level is set in O3DE project)')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='canyon_world',
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

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-50.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.5')

    pkg_bringup = FindPackageShare('fiber_nav_bringup')

    # Start Xvfb (O3DE requires XCB display even in console/headless mode)
    xvfb = ExecuteProcess(
        cmd=['Xvfb', ':99', '-screen', '0', '1920x1080x24'],
        output='log',
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    # TODO(Phase 2-4): Launch O3DE GameLauncher
    # The GameLauncher binary path and arguments will be configured here
    # once the O3DE project is integrated with the fiber_nav workspace.
    #
    # Expected command:
    #   ${PROJECT_PATH}/build/linux/bin/profile/HeadlessTest.GameLauncher
    #     --rhi=Vulkan --NullRenderer=false --console-mode
    #
    # O3DE ROS 2 Gem will publish directly to ROS 2 topics:
    #   /model/quadtailsitter/odometry (nav_msgs/Odometry)
    #   /camera (sensor_msgs/Image)
    #   /camera_down (sensor_msgs/Image)
    #   /clock (rosgraph_msgs/Clock)
    o3de_info = LogInfo(
        msg='O3DE backend selected — GameLauncher placeholder '
            '(full integration pending Phases 2-4)')

    # Include shared sensor nodes
    sensors_launch = IncludeLaunchDescription(
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
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,

        # O3DE display
        xvfb,

        # O3DE placeholder
        o3de_info,

        # Shared sensors + fusion + foxglove
        sensors_launch,
    ])
