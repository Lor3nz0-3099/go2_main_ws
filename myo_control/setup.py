from setuptools import setup
from glob import glob

package_name = 'myo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenzo',
    maintainer_email='lorenzo@example.com',
    description='ROS2 package for reading Myo armband data and controlling Go2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'myo_reader_node = myo_control.myo_reader_node:main',
            'myo_to_cmdvel_node = myo_control.myo_to_cmdvel_node:main',
            'myo_emg_monitor_node = myo_control.myo_emg_monitor_node:main',
            'myo_emg_events_node = myo_control.myo_emg_events_node:main',
        ],
    },
)