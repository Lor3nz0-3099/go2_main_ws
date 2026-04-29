from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    map_mode = LaunchConfiguration('map_mode')
    use_myo = LaunchConfiguration('use_myo')
    use_haptic = LaunchConfiguration('use_haptic')
    use_explorer = LaunchConfiguration('use_explorer')
    use_supervisor = LaunchConfiguration('use_supervisor')

    go2_nav_share = FindPackageShare('go2_nav_slam_sim')
    go2_supervisor_share = FindPackageShare('go2_supervisor')
    myo_share = FindPackageShare('myo_control')
    haptic_share = FindPackageShare('haptic_go2_teleop')
    explorer_share = FindPackageShare('go2_frontier_explorer')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            go2_nav_share,
            '/launch/go2_nav_localization.launch.py'
        ]),
        condition=IfCondition(
            PythonExpression(["'", map_mode, "' == 'localization'"])
        )
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            go2_nav_share,
            '/launch/go2_nav_slam_sim.launch.py'
        ]),
        condition=IfCondition(
            PythonExpression(["'", map_mode, "' == 'slam'"])
        )
    )

    supervisor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            go2_supervisor_share,
            '/launch/supervisor.launch.py'
        ]),
        condition=IfCondition(use_supervisor)
    )

    myo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            myo_share,
            '/launch/myo_teleop.launch.py'
        ]),
        condition=IfCondition(use_myo)
    )

    haptic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            haptic_share,
            '/launch/haptic_teleop.launch.py'
        ]),
        condition=IfCondition(use_haptic)
    )

    explorer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            explorer_share,
            '/launch/frontier_explorer.launch.py'
        ]),
        condition=IfCondition(use_explorer)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_mode',
            default_value='slam',
            description='Map mode: slam oppure localization'
        ),

        DeclareLaunchArgument(
            'use_supervisor',
            default_value='true',
            description='Launch supervisor, mux and managers'
        ),

        DeclareLaunchArgument(
            'use_myo',
            default_value='true',
            description='Launch Myo teleoperation'
        ),

        DeclareLaunchArgument(
            'use_haptic',
            default_value='false',
            description='Launch haptic teleoperation'
        ),

        DeclareLaunchArgument(
            'use_explorer',
            default_value='false',
            description='Launch frontier explorer'
        ),

        localization_launch,
        slam_launch,
        supervisor_launch,
        myo_launch,
        haptic_launch,
        explorer_launch,
    ])