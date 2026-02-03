"""
Launch complete fiber navigation simulation:
- Gazebo with canyon world
- PX4 SITL
- ROS 2 nodes (sensors, fusion)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='plane',
        description='PX4 vehicle model'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='canyon',
        description='Gazebo world name'
    )

    # Paths
    pkg_gazebo = FindPackageShare('fiber_nav_gazebo')
    world_file = PathJoinSubstitution([
        pkg_gazebo, 'worlds', LaunchConfiguration('world'), '.world'
    ])

    # PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd $PX4_HOME && make px4_sitl gazebo-classic'
        ],
        output='screen',
        shell=True
    )

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless', default='false'))
    )

    # Spool sensor simulator
    spool_sim = Node(
        package='fiber_nav_sensors',
        executable='spool_sim_driver',
        name='spool_sim_driver',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model'),
            'noise_stddev': 0.1,
            'slack_factor': 1.05,
        }]
    )

    # Vision direction simulator
    vision_sim = Node(
        package='fiber_nav_sensors',
        executable='vision_direction_sim',
        name='vision_direction_sim',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model'),
            'drift_rate': 0.001,
            'min_velocity': 0.5,
        }]
    )

    # Fusion node
    fusion = Node(
        package='fiber_nav_fusion',
        executable='fiber_vision_fusion',
        name='fiber_vision_fusion',
        output='screen',
        parameters=[{
            'slack_factor': 1.05,
            'publish_rate': 50.0,
            'max_data_age': 0.1,
        }]
    )

    return LaunchDescription([
        headless_arg,
        model_arg,
        world_arg,
        # px4_sitl,  # Enable when PX4 is configured
        # gazebo,    # Enable when world is ready
        spool_sim,
        vision_sim,
        fusion,
    ])
