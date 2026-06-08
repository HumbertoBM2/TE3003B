#!/usr/bin/env python3


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/hackerboard', '-b', '115200'],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=['0.08', '0.0', '0.12', '0', '0', '0', 'base_link', 'camera'],
        output='screen',
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0.0', '0.0', '0.10', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
    )


    core_nodes = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_sllidar, 'launch', 'sllidar_a2m8_launch.py')
                ),
                launch_arguments={'serial_port': '/dev/lidar'}.items()
            ),
            Node(
                package='puzzlebot_ros',
                executable='ekf_slam',
                name='ekf_slam',
                output='screen',
            ),
            Node(
                package='puzzlebot_ros',
                executable='navigator',
                name='navigator',
                output='screen',
            ),
            Node(
                package='puzzlebot_ros',
                executable='lift_controller',
                name='lift_controller',
                output='screen',
            ),
        ]
    )

  
    vision_nodes = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='puzzlebot_ros',
                executable='camera_node',
                name='camera_node',
                output='screen',
            ),
            Node(
                package='camera_info_publisher',
                executable='camera_info_publisher',
                name='camera_info_publisher',
                parameters=[
                    {'camera_calibration_file': 'file:///home/puzzlebot/.ros/jetson_cam.yaml'},
                    {'frame_id': 'camera'},
                ],
                remappings=[('camera_info', '/video_source/camera_info')],
                output='screen',
            ),
            Node(
                package='puzzlebot_ros',
                executable='aruco_detector',
                name='aruco_detector',
                parameters=[
                    {'marker_size': 0.10},
                    {'camera_frame': 'camera'},
                ],
                output='screen',
            ),
        ]
    )

    instructions = LogInfo(msg=(
        '\n'
        '══════════════════════════════════════════════════════════\n'
        '  FORKLIFT LAUNCH — arranque escalonado (13s total)\n'
        '  t=0s  micro-ROS + TFs\n'
        '  t=3s  lidar + EKF-SLAM + navigator\n'
        '  t=8s  cámara + ArUco\n'
        '\n'
        '  Verificar conexión ESP32:\n'
        '    ros2 node list | grep puzzlebot\n'
        '\n'
        '  Enviar goal de navegación:\n'
        '    ros2 topic pub --once /nav/goal geometry_msgs/msg/PoseStamped \\\n'
        '      "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}}}"\n'
        '══════════════════════════════════════════════════════════\n'
    ))

    return LaunchDescription([
        instructions,
        micro_ros,
        static_tf_camera,
        static_tf_laser,
        core_nodes,
        vision_nodes,
    ])
