from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_real_go2 = LaunchConfiguration('use_real_go2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_real_go2',
            default_value='false',
            description='Enable bridge from /body_pose to real Go2 services'
        ),

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
            executable='body_pose_go2_bridge',
            name='body_pose_go2_bridge',
            output='screen',
            condition=IfCondition(use_real_go2),
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