"""
Colosseum (Cosys-AirSim) simulation backend for fiber navigation.

Launches:
- colosseum_bridge_node (AirSim RPC -> ROS 2 topics)
- Shared sensor/fusion nodes via sensors.launch.py

The AirSim/Blocks process itself is managed by the Docker entrypoint
(colosseum-entrypoint.sh), not by this launch file.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Always headless for Colosseum (rendering in UE4 process)')

    use_px4_arg = DeclareLaunchArgument(
        'use_px4', default_value='false',
        description='PX4 controls motors')

    world_arg = DeclareLaunchArgument(
        'world', default_value='canyon_harmonic',
        description='World file (unused by Colosseum — level is in UE4 project)')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='canyon_world',
        description='World name (passed to sensors for compat)')

    auto_fly_arg = DeclareLaunchArgument(
        'auto_fly', default_value='false',
        description='Automatically apply thrust to fly')

    target_altitude_arg = DeclareLaunchArgument(
        'target_altitude', default_value='50.0',
        description='Target altitude (m)')

    target_speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='15.0',
        description='Target forward speed (m/s)')

    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch Foxglove bridge')

    mission_config_arg = DeclareLaunchArgument(
        'mission_config', default_value='',
        description='Optional mission YAML config overlay')

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='32.5')

    airsim_host_arg = DeclareLaunchArgument(
        'airsim_host', default_value='127.0.0.1',
        description='AirSim RPC host')

    airsim_port_arg = DeclareLaunchArgument(
        'airsim_port', default_value='41451',
        description='AirSim RPC port')

    pkg_bringup = FindPackageShare('fiber_nav_bringup')

    info = LogInfo(msg='Colosseum backend: launching bridge node')

    bridge_node = Node(
        package='fiber_nav_colosseum',
        executable='colosseum_bridge_node',
        name='colosseum_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('airsim_host'),
            'port': LaunchConfiguration('airsim_port'),
            'camera_name': 'front_center',
            'image_rate_hz': 30.0,
            'odom_rate_hz': 100.0,
        }],
    )

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
        airsim_host_arg,
        airsim_port_arg,
        info,
        bridge_node,
        sensors_launch,
    ])
