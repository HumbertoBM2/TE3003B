#!/usr/bin/env python3
"""
Launch MÍNIMO para mapeo — sin cámara ni ArUco.

Solo lo necesario para construir el mapa de occupancy grid:
  1. micro-ROS agent  — encoders del ESP32 + cmd_vel
  2. TF laser         — base_link → laser
  3. RPLIDAR A2M8     — /scan
  4. EKF-SLAM         — pose desde encoders (/slam/odom)
  5. Lidar Mapper     — occupancy grid /map + servicios save/clear

Sin cámara ni aruco → carga de CPU ~50% menos → DDS estable.

══════════════════════════════════════════════════════════════════════
PASO 1 — Terminal 1:
  source ~/ros2_ws/install/setup.bash
  ros2 launch puzzlebot_ros mapping_lite.launch.py

PASO 2 — Terminal 2 (teleop):
  source ~/.bashrc
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  → Baja la velocidad con z/z/z hasta speed ~0.2 antes de mover

PASO 3 — Guardar mapa cuando termines:
  ros2 service call /map/save std_srvs/srv/Trigger {}
  # Guarda en ~/maps/map_YYYYMMDD_HHMMSS.pgm y ~/maps/current.pgm

PASO 4 — RViz en PC master:
  export ROS_DOMAIN_ID=0
  rviz2  (Fixed Frame: map → agregar Map /map y LaserScan /scan)

Para limpiar el mapa sin reiniciar:
  ros2 service call /map/clear std_srvs/srv/Trigger {}

Para borrar un mapa guardado:
  rm ~/maps/NOMBRE.pgm ~/maps/NOMBRE.yaml
══════════════════════════════════════════════════════════════════════
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    pkg_sllidar = get_package_share_directory('sllidar_ros2')

    # ── 1. micro-ROS agent — PRIMERO para que DDS se establezca antes ────────
    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/hackerboard', '-b', '115200'],
        output='screen',
    )

    # ── 2. TF estático base_link → laser ─────────────────────────────────────
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0.0', '0.0', '0.10', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
    )

    # ── 3. RPLIDAR A2M8 ──────────────────────────────────────────────────────
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_a2m8_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/lidar'}.items()
    )

    # ── 4 & 5. EKF-SLAM y lidar_mapper — con delay para que micro-ROS
    #           establezca la sesión DDS antes de que haya más tráfico ─────────
    ekf_and_mapper = TimerAction(
        period=3.0,
        actions=[
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

    instructions = LogInfo(msg=(
        '\n'
        '══════════════════════════════════════════════════════════\n'
        '  MAPPING LITE listo (sin cámara).\n'
        '\n'
        '  TELEOP (terminal 2):\n'
        '    source ~/.bashrc\n'
        '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
        '    → Baja la velocidad con z/z/z hasta speed ~0.2\n'
        '\n'
        '  GUARDAR MAPA:\n'
        '    ros2 service call /map/save std_srvs/srv/Trigger {}\n'
        '\n'
        '  LIMPIAR MAPA:\n'
        '    ros2 service call /map/clear std_srvs/srv/Trigger {}\n'
        '══════════════════════════════════════════════════════════\n'
    ))

    return LaunchDescription([
        instructions,
        micro_ros,
        static_tf_laser,
        lidar,
        ekf_and_mapper,
    ])
