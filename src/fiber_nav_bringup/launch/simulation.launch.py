"""
Launch complete fiber navigation simulation with Gazebo Harmonic.

Launches:
- Gazebo Harmonic with canyon world (auto-running)
- ros_gz_bridge for topic bridging
- Sensor simulation nodes
- Fiber vision fusion node
- Plane controller for applying forces
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI'
    )

    use_px4_arg = DeclareLaunchArgument(
        'use_px4',
        default_value='false',
        description='Use PX4 SITL (false = use mock attitude)'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='canyon_harmonic',
        description='World file name (without .sdf extension)'
    )

    auto_fly_arg = DeclareLaunchArgument(
        'auto_fly',
        default_value='false',
        description='Automatically apply thrust to fly the plane'
    )

    thrust_arg = DeclareLaunchArgument(
        'thrust',
        default_value='20.0',
        description='Forward thrust force (N) when auto_fly is true'
    )

    lift_arg = DeclareLaunchArgument(
        'lift',
        default_value='20.0',
        description='Upward lift force (N) when auto_fly is true'
    )

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-50.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='50.0')

    # Package paths
    pkg_gazebo = FindPackageShare('fiber_nav_gazebo')
    pkg_bringup = FindPackageShare('fiber_nav_bringup')

    # World file path
    world_file = PathJoinSubstitution([
        pkg_gazebo, 'worlds',
        PythonExpression(["'", LaunchConfiguration('world'), ".sdf'"])
    ])

    # Gazebo Harmonic simulation (GUI) - with -r flag to auto-run
    gz_sim_gui = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-v4', '-r',  # -r to auto-run (unpause)
            world_file,
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    # Gazebo Harmonic simulation (headless) - with -r flag
    gz_sim_headless = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-v4', '-s', '-r',  # -s headless, -r auto-run
            world_file,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    # Spawn plane model (delayed to let Gazebo start)
    spawn_plane = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/canyon_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '10000',
                    '--req',
                    PythonExpression([
                        "'sdf_filename: \"",
                        PathJoinSubstitution([pkg_gazebo, 'models', 'plane', 'model.sdf']),
                        "\" pose: { position: { x: ",
                        LaunchConfiguration('spawn_x'),
                        ", y: ",
                        LaunchConfiguration('spawn_y'),
                        ", z: ",
                        LaunchConfiguration('spawn_z'),
                        " } } name: \"plane\"'"
                    ])
                ],
                output='screen'
            )
        ]
    )

    # ros_gz_bridge (delayed to let Gazebo start)
    # Bridges: odometry, clock, and PX4 sensors (IMU, baro, mag)
    ros_gz_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=[
                    # Core topics
                    '/model/plane/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    # PX4 sensor topics (for debugging/logging)
                    '/world/canyon_world/model/plane/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/world/canyon_world/model/plane/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                    '/world/canyon_world/model/plane/link/base_link/sensor/magnetometer_sensor/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
                ]
            )
        ]
    )

    # Spool sensor simulator (delayed)
    spool_sim = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='fiber_nav_sensors',
                executable='spool_sim_driver',
                name='spool_sim_driver',
                output='screen',
                parameters=[{
                    'odom_topic': '/model/plane/odometry',
                    'noise_stddev': 0.1,
                    'slack_factor': 1.05,
                }]
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
                parameters=[{
                    'odom_topic': '/model/plane/odometry',
                    'drift_rate': 0.001,
                    'min_velocity': 0.5,
                }]
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
                parameters=[{
                    'slack_factor': 1.05,
                    'publish_rate': 50.0,
                    'max_data_age': 0.1,
                }]
            )
        ]
    )

    # Plane controller node (delayed, only when auto_fly is true)
    plane_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='fiber_nav_sensors',
                executable='plane_controller',
                name='plane_controller',
                output='screen',
                parameters=[{
                    'thrust': LaunchConfiguration('thrust'),
                    'lift': LaunchConfiguration('lift'),
                    'world_name': 'canyon_world',
                    'model_name': 'plane',
                }],
                condition=IfCondition(LaunchConfiguration('auto_fly'))
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        headless_arg,
        use_px4_arg,
        world_arg,
        auto_fly_arg,
        thrust_arg,
        lift_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,

        # Gazebo
        gz_sim_gui,
        gz_sim_headless,
        spawn_plane,

        # Bridge
        ros_gz_bridge,

        # Sensors
        spool_sim,
        vision_sim,
        mock_attitude,

        # Fusion
        fusion,

        # Controller
        plane_controller,
    ])
