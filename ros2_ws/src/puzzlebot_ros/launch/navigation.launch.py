#!/usr/bin/env python3


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg         = get_package_share_directory('puzzlebot_ros')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    cfg_dir    = os.path.join(pkg, 'config')
    calib_yaml = os.path.join(cfg_dir, 'camera_calibration.yaml')
    extr_yaml  = os.path.join(cfg_dir, 'camera_extrinsics.yaml')
    aruco_yaml = os.path.join(cfg_dir, 'aruco_map.yaml')

    arg_camera = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Habilitar cámara y ArUco (false = solo lidar+odom)')
    camera_en = LaunchConfiguration('camera')

    arg_map = DeclareLaunchArgument(
        'map_path', default_value=os.path.expanduser('~/maps/current'),
        description='Ruta base del mapa sin extensión (se agrega .pgm / .yaml)')

    arg_mission = DeclareLaunchArgument(
        'mission', default_value='false',
        description='Habilitar mission_node y qr_reader_node (true para E80)')
    mission_en = LaunchConfiguration('mission')

    unset_fastdds = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', '')

    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/hackerboard', '-b', '115200'],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    tf_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_footprint_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
    tf_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_lidar',
        arguments=['0', '0', '0.175', '0', '0', '0', 'base_link', 'lidar_link'],
    )
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_lidar_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser'],
    )
    tf_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=['0.09', '0', '0.07', '0', '0', '0', 'base_link', 'camera_link'],
        condition=IfCondition(camera_en),
    )
    tf_cam_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_optical',
        arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963',
                   'camera_link', 'camera_optical_frame'],
        condition=IfCondition(camera_en),
    )

    cmd_vel_relay = Node(
        package='puzzlebot_ros', executable='cmd_vel_to_wheels',
        name='cmd_vel_relay', output='screen',
    )

    odometry = Node(
        package='puzzlebot_ros', executable='odometry_node',
        name='odometry_node',
        parameters=[{
            'wheel_radius':    0.05,
            'wheel_separation': 0.174,
            'enc_deadband':    0.10,
        }],
        output='screen',
    )

    lifter = Node(
        package='puzzlebot_ros', executable='lift_controller',
        name='lift_controller', output='screen',
    )

    nav_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name='sllidar_node',
                parameters=[{
                    'channel_type':     'serial',
                    'serial_port':      '/dev/lidar',
                    'serial_baudrate':  115200,
                    'frame_id':         'laser',
                    'inverted':         False,
                    'angle_compensate': True,
                    'scan_mode':        'Sensitivity',
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package='puzzlebot_ros', executable='scan_restamper',
                name='scan_restamper',
                parameters=[{
                    'input_topic':      '/scan',
                    'target_frame':     'lidar_link',
                    'invert_angles':    False,
                    'angle_offset_rad': 3.14159265359,
                }],
                output='screen',
            ),
            Node(
                package='puzzlebot_ros', executable='map_server_node',
                name='map_server_node',
                parameters=[{'map_path': LaunchConfiguration('map_path')}],
                output='screen',
            ),
            Node(
                package='puzzlebot_ros', executable='particle_localization',
                name='particle_localization',
                output='screen',
            ),
            Node(
                package='puzzlebot_ros', executable='astar_navigator',
                name='astar_navigator',
                output='screen',
            ),
        ],
    )

    mission_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='puzzlebot_ros', executable='qr_reader_node',
                name='qr_reader_node',
                parameters=[{
                    'detect_hz':     5.0,
                    'publish_image': True,
                    'min_qr_area_px': 200,
                }],
                output='screen',
                condition=IfCondition(mission_en),
            ),
            Node(
                package='puzzlebot_ros', executable='mission_node',
                name='mission_node',
                parameters=[{
                    'pickup_x':        1.50,
                    'pickup_y':        2.00,
                    'approach_dist_m': 0.30,
                    'approach_speed':  0.04,
                    'lift_up_secs':    3.00,
                    'lift_down_secs':  3.00,
                    'backup_secs':     1.50,
                    'qr_timeout_s':    30.0,
                    'nav_timeout_s':   60.0,
                }],
                output='screen',
                condition=IfCondition(mission_en),
            ),
        ],
    )

    vision_nodes = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='puzzlebot_ros', executable='camera_node',
                name='camera_node', output='screen',
                condition=IfCondition(camera_en),
            ),
            Node(
                package='puzzlebot_ros', executable='aruco_node',
                name='aruco_node',
                parameters=[{
                    'image_topic':             '/video_source/raw',
                    'camera_info_file':        calib_yaml,
                    'extrinsics_file':         extr_yaml,
                    'marker_map_file':         aruco_yaml,
                    'marker_length':           0.10,
                    'min_marker_area_px':      40.0,
                    'max_detection_distance':  2.0,
                    'max_incidence_angle_deg': 65.0,
                    'max_processing_hz':       10.0,
                    'max_position_jump':       0.20,
                    'last_pose_timeout':       3.0,
                    'camera_yaw_correction_deg': 0.0,
                    'far_marker_position_std': 0.25,
                    'far_marker_yaw_std':      0.30,
                }],
                output='screen',
                condition=IfCondition(camera_en),
            ),
            Node(
                package='puzzlebot_ros', executable='aruco_map_odom',
                name='aruco_map_odom',
                parameters=[{
                    'correction_alpha':        0.25,
                    'yaw_alpha':               0.01,
                    'max_correction_step_m':   0.8,
                    'max_correction_step_yaw': 0.50,
                    'publish_rate_hz':         20.0,
                    'correct_yaw':             False,
                    'map_min_x': 0.0,
                    'map_max_x': 3.76,
                    'map_min_y': 0.0,
                    'map_max_y': 4.86,
                }],
                output='screen',
                condition=IfCondition(camera_en),
            ),
        ],
    )

    return LaunchDescription([
        unset_fastdds,
        arg_camera,
        arg_map,
        arg_mission,
        micro_ros,
        tf_base, tf_lidar, tf_laser, tf_camera, tf_cam_optical,
        cmd_vel_relay,
        odometry,
        lifter,
        nav_nodes,
        mission_nodes,
        vision_nodes,
    ])
