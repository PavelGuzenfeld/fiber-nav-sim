"""
Launch PX4 SITL for fiber navigation simulation.

This launch file starts:
- PX4 SITL autopilot
- MicroXRCE-DDS agent for PX4 <-> ROS 2 communication

Requires PX4-Autopilot to be built and PX4_HOME environment variable set.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Arguments
    px4_home_arg = DeclareLaunchArgument(
        'px4_home',
        default_value=os.environ.get('PX4_HOME', '/root/PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='gz_x500',
        description='PX4 vehicle model'
    )

    # Package paths
    pkg_bringup = FindPackageShare('fiber_nav_bringup')
    px4_params_file = PathJoinSubstitution([
        pkg_bringup, 'config', 'px4_params.txt'
    ])

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

    # Start PX4 SITL (requires built PX4)
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd $PX4_HOME && make px4_sitl gz_x500'
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
