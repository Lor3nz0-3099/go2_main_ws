from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'go2_supervisor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzo@example.com',
    description='Supervisor and mode manager for Go2 teleoperation, navigation, exploration, and special actions',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_manager = go2_supervisor.mode_manager:main',
            'cmd_vel_mux = go2_supervisor.cmd_vel_mux:main',
            'action_manager = go2_supervisor.action_manager:main',
            'body_pose_mux = go2_supervisor.body_pose_mux:main',
            'nav_request_manager = go2_supervisor.nav_request_manager:main',
        ],
    },
)
