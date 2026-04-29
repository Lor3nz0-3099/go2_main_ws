import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_frontier_explorer')
    params_file = os.path.join(pkg_share, 'config', 'frontier_explorer.yaml')

    frontier_explorer = Node(
        package='go2_frontier_explorer',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        frontier_explorer
    ])