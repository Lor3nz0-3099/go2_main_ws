from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_supervisor',
            executable='mode_manager',
            name='mode_manager',
            output='screen'
        ),
        Node(
            package='go2_supervisor',
            executable='cmd_vel_mux',
            name='cmd_vel_mux',
            output='screen'
        ),
        Node(
            package='go2_supervisor',
            executable='body_pose_mux',
            name='body_pose_mux',
            output='screen'
        ),
        Node(
            package='go2_supervisor',
            executable='action_manager',
            name='action_manager',
            output='screen'
        ),
        Node(
            package='go2_supervisor',
            executable='nav_request_manager',
            name='nav_request_manager',
            output='screen'
        ),
    ])