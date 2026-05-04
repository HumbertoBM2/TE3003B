from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mcl_puzzlebot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        # Gazebo worlds
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.world')),
        # URDF
        (os.path.join('share', package_name, 'urdf'),    glob('urdf/*.urdf')),
        # Map images + YAML
        (os.path.join('share', package_name, 'maps'),    glob('maps/*')),
        # RViz config
        (os.path.join('share', package_name, 'rviz'),    glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humberto',
    maintainer_email='humberto@todo.todo',
    description='Monte Carlo Localization for Puzzlebot in Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mcl_node = mcl_puzzlebot.mcl_node:main',
        ],
    },
)
