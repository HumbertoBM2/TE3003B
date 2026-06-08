import os
from glob import glob
from setuptools import setup

package_name = 'puzzlebot_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puzzlebot',
    maintainer_email='humbertoronja@gmail.com',
    description='Puzzlebot montacargas autonomo — ROS2 Humble, Jetson Nano',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── Localizacion y SLAM ───────────────────────────────────────────
            'odometry_node   = puzzlebot_ros.odometry_node:main',
            'aruco_node      = puzzlebot_ros.aruco_node:main',
            'aruco_map_odom  = puzzlebot_ros.aruco_map_odom:main',
            'scan_restamper  = puzzlebot_ros.scan_restamper:main',
            'amigo_slam_node = puzzlebot_ros.amigo_slam_node:main',
            # ── Navegacion autonoma ───────────────────────────────────────────
            'map_server_node           = puzzlebot_ros.map_server_node:main',
            'astar_navigator           = puzzlebot_ros.astar_navigator:main',
            'particle_localization     = puzzlebot_ros.particle_localization_node:main',
            # ── Camara ───────────────────────────────────────────────────────
            'camera_node     = puzzlebot_ros.camera_node:main',
            # ── Control y hardware ───────────────────────────────────────────
            'cmd_vel_to_wheels = puzzlebot_ros.cmd_vel_to_wheels:main',
            'lift_controller = puzzlebot_ros.lift_controller:main',
            'qr_reader_node  = puzzlebot_ros.qr_reader_node:main',
            'mission_node    = puzzlebot_ros.mission_node:main',
            # ── Utilidades heredadas (no se usan en mapping.launch.py) ───────
            'ekf_slam        = puzzlebot_ros.ekf_slam:main',
            'aruco_detector  = puzzlebot_ros.aruco_detector:main',
            'kalman          = puzzlebot_ros.kalman:main',
            'kalman_aruco    = puzzlebot_ros.kalman_aruco:main',
            'goto_point      = puzzlebot_ros.goto_point:main',
            'navigator       = puzzlebot_ros.navigator:main',
            'lidar_mapper    = puzzlebot_ros.lidar_mapper:main',
            'scan_matcher    = puzzlebot_ros.scan_matcher:main',
            'yolo_detector   = puzzlebot_ros.yolo_detector:main',
            'puzzlebot_teleop = puzzlebot_ros.puzzlebot_teleop:main',
            'pwm_control     = puzzlebot_ros.pwm_control:main',
            'velocity_control = puzzlebot_ros.velocity_control:main',
            'distance_control = puzzlebot_ros.distance_control:main',
        ],
    },
)
