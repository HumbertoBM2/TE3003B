"""
puzzlebot_launch.py
===================
Launch file for the Puzzlebot RVIZ simulation.

Nodes started
-------------
1. static_transform_publisher  – publishes the static TF  map -> odom  (offset: x=1, y=1).
2. robot_state_publisher       – reads the URDF and publishes:
     - Static TF  base_footprint -> base_link       (from fixed joint)
     - Static TF  base_link      -> caster_link      (from fixed joint)
     - Dynamic TF base_link      -> wheel_r_link     (driven by /joint_states)
     - Dynamic TF base_link      -> wheel_l_link     (driven by /joint_states)
3. joint_state_publisher       – publishes the dynamic TF  odom -> base_footprint
                                  and /joint_states for the two wheels.
4. rviz2                       – opens RVIZ with the pre-configured .rviz file.
                                  Closing RVIZ shuts down the whole launch.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('puzzlebot_sim')

    # ------------------------------------------------------------------
    # URDF
    # ------------------------------------------------------------------
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ------------------------------------------------------------------
    # 1. Static transform: map -> odom
    #    A fixed offset is given so that the two frames are visually
    #    separated in RVIZ (matching the diagram in the challenge brief).
    #    In a real robot this offset would represent the accumulated
    #    localisation correction; here it is a constant approximation.
    # ------------------------------------------------------------------
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '--x', '1.0', '--y', '1.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    # ------------------------------------------------------------------
    # 2. Robot state publisher – translates URDF joints to TF broadcasts
    # ------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ------------------------------------------------------------------
    # 3. Custom joint state publisher node
    # ------------------------------------------------------------------
    joint_state_publisher = Node(
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        name='puzzlebot_publisher',
        output='screen',
    )

    # ------------------------------------------------------------------
    # 4. RVIZ
    # ------------------------------------------------------------------
    rviz_config = os.path.join(pkg_share, 'rviz', 'puzzlebot_rviz.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Shut down the entire launch when RVIZ is closed.
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    # ------------------------------------------------------------------
    # Launch description
    # ------------------------------------------------------------------
    return LaunchDescription([
        map_to_odom_tf,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
        shutdown_on_rviz_exit,
    ])
