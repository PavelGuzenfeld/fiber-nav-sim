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
Launch PX4 SITL for fiber navigation simulation.

This launch file starts:
- PX4 SITL autopilot
- MicroXRCE-DDS agent for PX4 <-> ROS 2 communication

Requires PX4-Autopilot to be built and PX4_HOME environment variable set.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    px4_home_arg = DeclareLaunchArgument(
        'px4_home',
        default_value=os.environ.get('PX4_HOME', '/root/PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='gz_quadtailsitter_vision',
        description='PX4 vehicle model with vision (GPS-denied quad-tailsitter)'
    )

    airframe_arg = DeclareLaunchArgument(
        'airframe',
        default_value='4251',
        description='PX4 airframe ID for gz_quadtailsitter_vision'
    )

    # Set PX4 environment variables
    set_px4_home = SetEnvironmentVariable(
        name='PX4_HOME',
        value=LaunchConfiguration('px4_home')
    )

    set_px4_gz_model = SetEnvironmentVariable(
        name='PX4_GZ_MODEL',
        value=LaunchConfiguration('model')
    )

    set_px4_sim_model = SetEnvironmentVariable(
        name='PX4_SIM_MODEL',
        value=LaunchConfiguration('model')
    )

    # Start PX4 SITL with native Gazebo integration
    # Uses custom airframe with vision velocity fusion enabled
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd $PX4_HOME/build/px4_sitl_default/rootfs && '
            'rm -f dataman parameters*.bson && '
            'PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=plane '
            '../bin/px4'
        ],
        output='screen',
        shell=False
    )

    # MicroXRCE-DDS Agent for PX4 <-> ROS 2 communication
    # Connects to PX4's built-in XRCE-DDS client
    micro_xrce_dds_agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'udp4', '-p', '8888'
        ],
        output='screen'
    )

    # Apply PX4 parameters after startup
    # Note: This requires QGC or MAVLink shell access
    # Parameters are applied via PX4 command line or QGC
    apply_params_info = ExecuteProcess(
        cmd=[
            'echo',
            'PX4 parameters should be applied manually or via QGC.',
            'See config/px4_params.txt for required settings.'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        px4_home_arg,
        model_arg,
        airframe_arg,

        # Environment
        set_px4_home,
        set_px4_gz_model,
        set_px4_sim_model,

        # PX4 SITL
        px4_sitl,

        # DDS Agent (delayed start to let PX4 initialize)
        TimerAction(
            period=5.0,
            actions=[micro_xrce_dds_agent]
        ),

        # Info
        apply_params_info,
    ])
