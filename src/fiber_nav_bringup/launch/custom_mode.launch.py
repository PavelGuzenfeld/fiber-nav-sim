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
Launch custom PX4 flight mode with MicroXRCE-DDS agent.

Prerequisites:
- Gazebo must be running with the quadtailsitter model spawned
- PX4 SITL must be running and connected to Gazebo

Launches:
- MicroXRCE-DDS agent (UDP port 8888)
- Custom mode node (hold_mode_node, canyon_mission_node, or vtol_navigation_node)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='hold',
        description='Custom mode to launch: hold, canyon, or vtol'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Path to YAML parameter file (used with vtol mode)'
    )

    # MicroXRCE-DDS agent for PX4 <-> ROS 2 bridge
    dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    # Hold mode node (delayed to let DDS agent start)
    hold_mode = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='fiber_nav_mode',
                executable='hold_mode_node',
                name='hold_mode_node',
                output='screen',
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('mode'), "' == 'hold'"
                ]))
            )
        ]
    )

    # Canyon mission node (delayed to let DDS agent start)
    canyon_mission = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='fiber_nav_mode',
                executable='canyon_mission_node',
                name='canyon_mission_node',
                output='screen',
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('mode'), "' == 'canyon'"
                ]))
            )
        ]
    )

    # VTOL navigation node (delayed to let DDS agent start)
    vtol_navigation = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='fiber_nav_mode',
                executable='vtol_navigation_node',
                name='vtol_navigation_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
                additional_env={
                    'RCUTILS_LOGGING_BUFFERED_STREAM': '0',
                },
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('mode'), "' == 'vtol'"
                ]))
            )
        ]
    )

    return LaunchDescription([
        mode_arg,
        params_file_arg,
        dds_agent,
        hold_mode,
        canyon_mission,
        vtol_navigation,
    ])
