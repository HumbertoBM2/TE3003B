#!/usr/bin/env python3
"""
Launch de SLAM mapping manual — sistema completo con cámara y ArUco.

Arranque escalonado:
  t=0s   micro-ROS agent + TFs estáticos
  t=3s   RPLIDAR + EKF-SLAM + lidar_mapper
  t=8s   cámara + CameraInfo + aruco_detector (OpenCV DICT_4X4_50)

══════════════════════════════════════════════════════════════════════
PASO 1 — Terminal 1:
  sudo systemctl restart nvargus-daemon
  source ~/ros2_ws/install/setup.bash
  ros2 launch puzzlebot_ros mapping.launch.py

PASO 2 — Terminal 2 (teleop — con remap para pasar por el relay de corrección):
  source ~/.bashrc
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=cmd_vel_keyboard
  → Presiona z varias veces para bajar la velocidad antes de mover
  → Velocidad recomendada: 0.2 m/s máximo

PASO 3 — Guardar mapa cuando termines:
  ros2 service call /map/save std_srvs/srv/Trigger {}
  # O simplemente Ctrl+C → el mapa se guarda automáticamente

PASO 4 — RViz en PC master:
  export ROS_DOMAIN_ID=0
  rviz2  (Fixed Frame: map → agregar Map /map, LaserScan /scan,
          MarkerArray /slam/map, Odometry /slam/odom)

NOTA: Presiona z varias veces antes de mover para bajar velocidad (~0.2 m/s).
      Si el ESP32 se resetea: espera ~5s y vuelve a intentar.
══════════════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    # ── t=0s: relay corrección motor (cmd_vel_keyboard → cmd_vel con bias) ──────
    cmd_vel_relay = Node(
        package='puzzlebot_ros',
        executable='cmd_vel_to_wheels',
        name='cmd_vel_relay',
        parameters=[{
            'left_scale': 0.52,
            'kp': 1.5,
            'ki': 0.30,
            'wheel_radius': 0.05,
            'wheel_base': 0.18,
        }],
        output='screen',
    )

    # ── t=0s: micro-ROS agent ─────────────────────────────────────────────────
    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/hackerboard', '-b', '115200'],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    # ── t=0s: TFs estáticos ───────────────────────────────────────────────────
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

    # ── t=3s: lidar + scan matcher + SLAM + mapper ───────────────────────────
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
                executable='scan_matcher',
                name='scan_matcher',
                output='screen',
            ),
            Node(
                package='puzzlebot_ros',
                executable='ekf_slam',
                name='ekf_slam',
                output='screen',
            ),
            Node(
                package='puzzlebot_ros',
                executable='lidar_mapper',
                name='lidar_mapper',
                output='screen',
            ),
        ]
    )

    # ── t=8s: cámara + ArUco (detector propio OpenCV DICT_4X4_50) ────────────
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
                    {'marker_size': 0.096},
                    {'camera_frame': 'camera'},
                ],
                output='screen',
            ),
        ]
    )

    instructions = LogInfo(msg=(
        '\n'
        '══════════════════════════════════════════════════════════\n'
        '  MAPPING LAUNCH — arranque escalonado\n'
        '  t=0s  micro-ROS + TFs estáticos\n'
        '  t=3s  lidar + EKF-SLAM + lidar_mapper\n'
        '  t=8s  cámara + aruco_detector (DICT_4X4_50)\n'
        '\n'
        '  TELEOP (terminal 2) — SIEMPRE con el remap:\n'
        '    source ~/.bashrc\n'
        '    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\\n'
        '      --ros-args -r cmd_vel:=cmd_vel_keyboard\n'
        '\n'
        '  Presiona z varias veces para bajar velocidad antes de mover\n'
        '  GUARDAR MAPA: Ctrl+C o ros2 service call /map/save std_srvs/srv/Trigger {}\n'
        '══════════════════════════════════════════════════════════\n'
    ))

    return LaunchDescription([
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', ''),
        instructions,
        cmd_vel_relay,
        micro_ros,
        static_tf_camera,
        static_tf_laser,
        core_nodes,
        vision_nodes,
    ])
