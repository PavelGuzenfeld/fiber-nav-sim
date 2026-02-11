# Copyright 2026 Pavel Guzenfeld — All rights reserved.
# PRIVATE AND CONFIDENTIAL. Unauthorized copying prohibited.
# Version: 0.0.1

"""
Gazebo Harmonic simulation backend for fiber navigation.

Launches Gazebo-specific components:
- Gazebo Harmonic simulator (GUI or headless)
- Model spawning (quadtailsitter)
- ros_gz_bridge for topic bridging (odometry, cameras, clock, sensors)

Then includes sensors.launch.py for shared sensor/fusion nodes.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI')

    use_px4_arg = DeclareLaunchArgument(
        'use_px4', default_value='false',
        description='PX4 controls motors (disables stabilized_flight_controller and mock attitude)')

    world_arg = DeclareLaunchArgument(
        'world', default_value='canyon_harmonic',
        description='World file name (without .sdf extension)')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='canyon_world',
        description='Gazebo internal world name (must match <world name="..."> in SDF)')

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

    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='32.5')

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
        cmd=['gz', 'sim', '-v4', '-r', world_file],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    # Gazebo Harmonic simulation (headless)
    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-v4', '-s', '-r', '--headless-rendering', world_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    # Model file selection: PX4 variant has motor plugins + revolute joints
    model_file = PythonExpression([
        "'model_px4.sdf' if '",
        LaunchConfiguration('use_px4'),
        "' == 'true' else 'model.sdf'"
    ])

    # Spawn orientation: always flat for now (tailsitter nose-up disabled for testing)
    spawn_orientation = ''

    # Dynamic spawn service path based on world name
    spawn_service = PythonExpression([
        "'/world/' + '",
        LaunchConfiguration('world_name'),
        "' + '/create'"
    ])

    # Spawn quadtailsitter model (always)
    spawn_model = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', spawn_service,
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '10000',
                    '--req',
                    PythonExpression([
                        '\'sdf_filename: "',
                        PathJoinSubstitution(
                            [pkg_gazebo, 'models', 'quadtailsitter', model_file]
                        ),
                        '" pose: { position: { x: ',
                        LaunchConfiguration('spawn_x'),
                        ', y: ',
                        LaunchConfiguration('spawn_y'),
                        ', z: ',
                        LaunchConfiguration('spawn_z'),
                        ' }',
                        spawn_orientation,
                        ' } name: "quadtailsitter"\''
                    ])
                ],
                output='screen',
            )
        ]
    )

    # ---- Bridge topics for quadtailsitter model ----
    _wn = LaunchConfiguration('world_name')
    _link = "/model/quadtailsitter/link/base_link"

    imu_topic = PythonExpression([
        "'/world/", _wn, _link,
        "/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'"
    ])
    baro_topic = PythonExpression([
        "'/world/", _wn, _link,
        "/sensor/air_pressure_sensor/air_pressure"
        "@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure'"
    ])
    mag_topic = PythonExpression([
        "'/world/", _wn, _link,
        "/sensor/magnetometer_sensor/magnetometer"
        "@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer'"
    ])
    forward_cam = PythonExpression([
        "'/world/", _wn, _link,
        "/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'"
    ])
    down_cam = PythonExpression([
        "'/world/", _wn, _link,
        "/sensor/camera_down/image@sensor_msgs/msg/Image[gz.msgs.Image'"
    ])
    follow_cam = PythonExpression([
        "'/world/", _wn, _link,
        "/sensor/follow_cam/image@sensor_msgs/msg/Image[gz.msgs.Image'"
    ])
    forward_cam_headless = '/camera@sensor_msgs/msg/Image[gz.msgs.Image'
    down_cam_headless = '/camera_down@sensor_msgs/msg/Image[gz.msgs.Image'
    follow_cam_headless = (
        '/follow_camera@sensor_msgs/msg/Image[gz.msgs.Image'
    )

    # ros_gz_bridge
    ros_gz_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=[
                    '/model/quadtailsitter/odometry'
                    '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    imu_topic,
                    baro_topic,
                    mag_topic,
                    forward_cam,
                    down_cam,
                    follow_cam,
                    forward_cam_headless,
                    down_cam_headless,
                    follow_cam_headless,
                ],
            )
        ]
    )

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

        # Gazebo
        gz_sim_gui,
        gz_sim_headless,
        spawn_model,

        # Bridge
        ros_gz_bridge,

        # Shared sensors + fusion + foxglove
        sensors_launch,
    ])
