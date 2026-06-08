#!/usr/bin/env python3


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, LogInfo, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg      = get_package_share_directory('puzzlebot_ros')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    cfg_dir = os.path.join(pkg, 'config')
    calib_yaml   = os.path.join(cfg_dir, 'camera_calibration.yaml')
    extr_yaml    = os.path.join(cfg_dir, 'camera_extrinsics.yaml')
    aruco_yaml   = os.path.join(cfg_dir, 'aruco_map.yaml')
    slam_yaml    = os.path.join(cfg_dir, 'slam_params.yaml')

    arg_camera = DeclareLaunchArgument('camera', default_value='true',
                    description='Habilitar cámara y ArUco (false = solo lidar+odom)')
    camera_en  = LaunchConfiguration('camera')

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
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint_link',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0',
                   'base_footprint', 'base_link'],
        output='screen',
    )


    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_lidar',
        arguments=['0.0', '0.0', '0.175', '0', '0', '0',
                   'base_link', 'lidar_link'],
        output='screen',
    )

    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_lidar_laser',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0',
                   'lidar_link', 'laser'],
        output='screen',
    )

    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=['0.09', '0.0', '0.07', '0', '0', '0',
                   'base_link', 'camera_link'],
        output='screen',
        condition=IfCondition(camera_en),
    )

    tf_cam_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_optical',
        arguments=['0.0', '0.0', '0.0',
                   '-1.5707963', '0.0', '-1.5707963',
                   'camera_link', 'camera_optical_frame'],
        output='screen',
        condition=IfCondition(camera_en),
    )

    cmd_vel_relay = Node(
        package='puzzlebot_ros',
        executable='cmd_vel_to_wheels',
        name='cmd_vel_relay',
        output='screen',
    )

    odometry = Node(
        package='puzzlebot_ros',
        executable='odometry_node',
        name='odometry_node',
        parameters=[{
            'wheel_radius':     0.05,
      
            'wheel_separation': 0.174,
            'enc_deadband':     0.10,
        }],
        output='screen',
    )

    lidar_nodes = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_sllidar, 'launch', 'sllidar_a2m8_launch.py')
                ),
                launch_arguments={'serial_port': '/dev/lidar'}.items(),
            ),
        
            Node(
                package='puzzlebot_ros',
                executable='scan_restamper',
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
                package='puzzlebot_ros',
                executable='amigo_slam_node',
                name='amigo_slam_node',
                parameters=[slam_yaml],
                output='screen',
            ),
        ],
    )

    vision_nodes = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='puzzlebot_ros',
                executable='camera_node',
                name='camera_node',
                output='screen',
                condition=IfCondition(camera_en),
            ),
            Node(
                package='puzzlebot_ros',
                executable='aruco_node',
                name='aruco_node',
                parameters=[{
                    'image_topic':            '/video_source/raw',
                    'camera_info_file':       calib_yaml,
                    'extrinsics_file':        extr_yaml,
                    'marker_map_file':        aruco_yaml,
                    'marker_length':          0.10,
             
                    'min_marker_area_px':     40.0,

                    'max_detection_distance': 2.0,
             
                    'max_incidence_angle_deg': 65.0,
           
                    'max_processing_hz':      10.0,
    
                    'max_position_jump':      0.20,
                    'last_pose_timeout':      3.0,
     
                    'camera_yaw_correction_deg': 0.0,
                    'far_marker_position_std': 0.25,
                    'far_marker_yaw_std':      0.30,
                }],
                output='screen',
                condition=IfCondition(camera_en),
            ),
            Node(
                package='puzzlebot_ros',
                executable='aruco_map_odom',
                name='aruco_map_odom',
                parameters=[{
                
                    'correction_alpha': 0.25,
             
                    'yaw_alpha': 0.01,
                    'max_correction_step_m':   0.8,
                    'max_correction_step_yaw': 0.50,
                    'publish_rate_hz': 20.0,
                    'correct_yaw': False,
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

    lifter = Node(
        package='puzzlebot_ros',
        executable='lift_controller',
        name='lift_controller',
        output='screen',
    )

    return LaunchDescription([
        unset_fastdds,
        arg_camera,
        # Infraestructura base
        micro_ros,
        tf_base,
        tf_lidar,
        tf_laser,
        tf_camera,
        tf_cam_optical,
        cmd_vel_relay,
        odometry,
        lifter,
        # t=3s: lidar + slam
        lidar_nodes,
        # t=8s: cámara + aruco
        vision_nodes,
    ])
