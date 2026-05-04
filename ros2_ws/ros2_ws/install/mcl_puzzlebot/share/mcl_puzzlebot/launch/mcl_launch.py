#!/usr/bin/env python3
"""
MCL Puzzlebot Launch File
=========================
Starts:
  1. Gazebo Classic with the MCL room world.
  2. robot_state_publisher — broadcasts /robot_description and static TFs.
  3. spawn_entity.py       — drops the Puzzlebot URDF into Gazebo.
  4. mcl_node              — Monte Carlo Localization particle filter.
  5. rviz2                 — visualisation.

Usage:
  ros2 launch mcl_puzzlebot mcl_launch.py

Drive the robot in a separate terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mcl     = get_package_share_directory('mcl_puzzlebot')
    pkg_gazebo  = get_package_share_directory('gazebo_ros')

    world_file  = os.path.join(pkg_mcl, 'worlds', 'mcl_room.world')
    urdf_file   = os.path.join(pkg_mcl, 'urdf',   'puzzlebot_gazebo.urdf')
    rviz_file   = os.path.join(pkg_mcl, 'rviz',   'mcl_rviz.rviz')

    with open(urdf_file, 'r') as fh:
        robot_description = fh.read()

    return LaunchDescription([

        # ── 1. Gazebo (gzserver + gzclient) ──────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # ── 2. Robot State Publisher ──────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # ── 3. Spawn robot into Gazebo ────────────────────────────────────
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_puzzlebot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'puzzlebot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.05',
                '-Y', '0.0',
            ],
        ),

        # ── 4. MCL node ───────────────────────────────────────────────────
        Node(
            package='mcl_puzzlebot',
            executable='mcl_node',
            name='mcl_node',
            output='screen',
        ),

        # ── 5. RViz2 ──────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            on_exit=None,      # keep the rest alive if rviz is closed
        ),
    ])
