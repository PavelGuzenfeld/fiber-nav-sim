"""
Launch only the sensor simulation nodes for testing.
Requires Gazebo to be running separately with ground truth topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='plane',
        description='Model name in Gazebo'
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

    return LaunchDescription([
        model_arg,
        spool_sim,
        vision_sim,
    ])
