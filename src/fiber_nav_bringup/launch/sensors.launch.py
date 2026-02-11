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
Shared sensor and fusion nodes for fiber navigation simulation.

Included by both gazebo_simulation.launch.py and o3de_simulation.launch.py.
These nodes are backend-agnostic (pure ROS 2, no Gazebo dependency).

Launches:
- Spool sensor simulator
- Vision direction simulator
- Mock attitude publisher (when not using PX4)
- Fiber vision fusion node
- Stabilized flight controller (when auto_fly and not use_px4)
- Foxglove bridge for web visualization (when foxglove:=true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments (declared here so this file can be launched standalone for testing)
    use_px4_arg = DeclareLaunchArgument(
        'use_px4', default_value='false',
        description='PX4 controls motors (disables stabilized_flight_controller and mock attitude)')

    auto_fly_arg = DeclareLaunchArgument(
        'auto_fly', default_value='false',
        description='Automatically apply thrust to fly')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='canyon_world',
        description='Gazebo internal world name (for stabilized flight controller)')

    target_altitude_arg = DeclareLaunchArgument(
        'target_altitude', default_value='50.0',
        description='Target altitude (m) for stabilized flight controller')

    target_speed_arg = DeclareLaunchArgument(
        'target_speed', default_value='15.0',
        description='Target forward speed (m/s) for stabilized flight controller')

    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch Foxglove bridge for web visualization')

    # Params file for sensor/fusion nodes
    params_file = PathJoinSubstitution([
        FindPackageShare('fiber_nav_bringup'), 'config', 'sensor_params.yaml'
    ])

    # Odom topic (always quadtailsitter)
    odom_topic = '/model/quadtailsitter/odometry'

    # Spool sensor simulator (delayed)
    spool_sim = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='fiber_nav_sensors',
                executable='spool_sim_driver',
                name='spool_sim_driver',
                output='screen',
                parameters=[params_file, {'odom_topic': odom_topic}]
            )
        ]
    )

    # Vision direction simulator (delayed)
    vision_sim = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='fiber_nav_sensors',
                executable='vision_direction_sim',
                name='vision_direction_sim',
                output='screen',
                parameters=[params_file, {'odom_topic': odom_topic}]
            )
        ]
    )

    # Mock attitude publisher (when not using PX4)
    mock_attitude = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='fiber_nav_sensors',
                executable='mock_attitude_publisher',
                name='mock_attitude_publisher',
                output='screen',
                parameters=[{
                    'publish_rate': 50.0,
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': 0.0,
                }],
                condition=UnlessCondition(LaunchConfiguration('use_px4'))
            )
        ]
    )

    # Fusion node (delayed)
    fusion = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='fiber_nav_fusion',
                executable='fiber_vision_fusion',
                name='fiber_vision_fusion',
                output='screen',
                parameters=[params_file]
            )
        ]
    )

    # Stabilized flight controller (only when auto_fly is true AND not using PX4)
    # Start early to minimize free-fall before hover wrench is applied
    stabilized_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='fiber_nav_sensors',
                executable='stabilized_flight_controller',
                name='stabilized_flight_controller',
                output='screen',
                parameters=[{
                    'world_name': LaunchConfiguration('world_name'),
                    'model_name': 'quadtailsitter',
                    'target_altitude': LaunchConfiguration('target_altitude'),
                    'target_speed': LaunchConfiguration('target_speed'),
                    'auto_enable': True,
                }],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('auto_fly'),
                    "' == 'true' and '",
                    LaunchConfiguration('use_px4'),
                    "' != 'true'"
                ]))
            )
        ]
    )

    # Foxglove bridge for web visualization (default enabled)
    foxglove_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[{'port': 8765}],
                condition=IfCondition(LaunchConfiguration('foxglove'))
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        use_px4_arg,
        auto_fly_arg,
        world_name_arg,
        target_altitude_arg,
        target_speed_arg,
        foxglove_arg,

        # Sensors
        spool_sim,
        vision_sim,
        mock_attitude,

        # Fusion
        fusion,

        # Stabilized flight controller (standalone mode only)
        stabilized_controller,

        # Foxglove bridge (web visualization)
        foxglove_bridge,
    ])
