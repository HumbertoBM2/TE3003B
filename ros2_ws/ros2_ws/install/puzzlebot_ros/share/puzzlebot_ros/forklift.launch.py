#!/usr/bin/env python3
"""
Launch principal — sistema montacargas autónomo completo.

Orden de arranque (crítico para DDS estable):
  1. micro-ROS agent     — PRIMERO, antes de cualquier otro nodo
  2. TFs estáticos       — inmediatos, sin carga
  3. [delay 3s]
  4. RPLIDAR A2M8        — /scan
  5. EKF-SLAM            — pose desde encoders + ArUco
  6. Navegador           — publica /cmd_vel autónomo
  7. [delay 5s adicional para que DDS esté asentado]
  8. Cámara + CameraInfo + ArUco  — más pesados, al final

Requiere mapa guardado en ~/maps/current.yaml para navegación.

══════════════════════════════════════════════════════════════════════
Antes de correr:
  sudo systemctl restart nvargus-daemon

Cómo correr:
  source ~/ros2_ws/install/setup.bash
  ros2 launch puzzlebot_ros forklift.launch.py

Para visualizar en PC master:
  export ROS_DOMAIN_ID=0
  rviz2

Para enviar un goal de navegación:
  ros2 topic pub --once /nav/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}}}"

Para verificar estado:
  ros2 topic echo /slam/status
  ros2 topic echo /nav/status
══════════════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    # ── 1. micro-ROS agent — PRIMERO para estabilidad DDS ────────────────────
    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/hackerboard', '-b', '115200'],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    # ── 2. TFs estáticos — sin carga, arrancan junto con el agent ────────────
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

    # ── 3-6. Lidar + EKF + Navigator — delay 3s para que micro-ROS establezca
    #         la sesión DDS antes de que haya más tráfico en el bus ───────────
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

    # ── 7-9. Cámara + ArUco — delay 8s (los más pesados, al final) ───────────
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
