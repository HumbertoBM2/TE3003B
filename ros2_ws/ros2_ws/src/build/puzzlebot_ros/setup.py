import os
from glob import glob
from setuptools import setup

package_name = 'puzzlebot_ros'


#gazebo_files = []
#gazebo_files.append(('share/ament_index/resource_index/packages',['resource/' + package_name]))
#gazebo_files.append(('share/' + package_name, ['package.xml']))
#gazebo_files.append((os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')))
#for dirpath, dirnames, filenames in os.walk('gazebo'):
#    for filename in filenames:
#        gazebo_files.append(('share/' + package_name+'/'+dirpath,[dirpath +'/'+filename]))
#for file in gazebo_files:
#    print(file)



setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduard Codres',
    maintainer_email='eduard.codres@manchester.ac.uk',
    description='Puzzlebot control examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
                'pwm_control = puzzlebot_ros.pwm_control:main',
                'velocity_control = puzzlebot_ros.velocity_control:main',
                'distance_control = puzzlebot_ros.distance_control:main',
                'test_imu = puzzlebot_ros.test_imu:main',
                'kalman = puzzlebot_ros.kalman:main',
                'kalman_aruco = puzzlebot_ros.kalman_aruco:main',
                'goto_point = puzzlebot_ros.goto_point:main',
                # ── Montacargas autónomo (proyecto final) ──────────────────
                'ekf_slam = puzzlebot_ros.ekf_slam:main',
                'navigator = puzzlebot_ros.navigator:main',
                'camera_compress = puzzlebot_ros.camera_compress:main',
                'camera_node = puzzlebot_ros.camera_node:main',
                'cmd_vel_to_wheels = puzzlebot_ros.cmd_vel_to_wheels:main',
                'aruco_detector = puzzlebot_ros.aruco_detector:main',
                'lidar_mapper = puzzlebot_ros.lidar_mapper:main',
                'scan_matcher = puzzlebot_ros.scan_matcher:main',
                'lift_controller = puzzlebot_ros.lift_controller:main',
                'puzzlebot_teleop = puzzlebot_ros.puzzlebot_teleop:main',
        ],
    },
)
