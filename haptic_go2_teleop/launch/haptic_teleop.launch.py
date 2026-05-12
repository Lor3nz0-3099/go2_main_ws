from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    omni_common_dir = get_package_share_directory('omni_common')
    omni_launch = os.path.join(omni_common_dir, 'launch', 'omni_state.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omni_launch)
        ),

        Node(
            package='haptic_go2_teleop',
            executable='haptic_go2_teleop_position_node',
            name='haptic_go2_teleop_position_node',
            output='screen',
            parameters=[
                {'cmd_vel_topic': '/teleop_haptic/cmd_vel'},
                {'body_pose_topic': '/teleop_haptic/body_pose'},
                {'invert_body_roll': True},
                {'invert_body_pitch': True},
            ]
        )
    ])