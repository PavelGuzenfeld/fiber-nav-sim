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
