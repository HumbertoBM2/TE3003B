#!/usr/bin/env python3


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
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
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0.0', '0.0', '0.10', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar, 'launch', 'sllidar_a2m8_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/lidar'}.items()
    )


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
