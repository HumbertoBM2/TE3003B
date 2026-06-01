from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    package_share = get_package_share_directory("voice_hmm")
    config_path = os.path.join(package_share, "config", "voice_hmm.yaml")

    return LaunchDescription([
        Node(
            package="voice_hmm",
            executable="voice_recognition_node",
            name="voice_recognition_node",
            output="screen",
            parameters=[config_path]
        )
    ])
