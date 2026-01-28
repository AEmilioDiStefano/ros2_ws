from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_legion_teleop_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install config into the package share directory
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Install web UI assets if you use them
        (os.path.join('share', package_name, 'webui'), glob('robot_legion_teleop_python/webui/*')),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('robot_legion_teleop_python/launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A. Emilio DiStefano',
    maintainer_email='emilio@vitruvian.systems',
    description='Robot Legion teleop + motor driver + executor (proprietary)',
    license='LicenseRef-Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_legion_key = robot_legion_teleop_python.teleop_legion_key:main',
            'motor_driver_node = robot_legion_teleop_python.motor_driver_node:main',
            'heartbeat_node = robot_legion_teleop_python.heartbeat_node:main',
            'unit_executor_action_server = robot_legion_teleop_python.unit_executor_action_server:main',
        ],
    },
)
