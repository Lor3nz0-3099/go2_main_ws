import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_nav_slam_sim')
    params_file = os.path.join(pkg_share, 'config', 'go2_nav_slam_sim.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'go2_nav_slam_sim.rviz')

    ground_truth_bridge = Node(
        package='go2_nav_slam_sim',
        executable='ground_truth_to_tf',
        name='ground_truth_to_tf',
        output='screen',
        parameters=[{
            'input_topic': '/odom/ground_truth',
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }]
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.10,
            'min_height': -0.15,
            'max_height': 0.30,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'queue_size': 8
        }],
        remappings=[
            ('cloud_in', '/velodyne_points'),
            ('scan', '/scan_raw')
        ]
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('scan', '/scan_raw')
        ]
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('cmd_vel', '/nav/cmd_vel_raw'),
            ('/cmd_vel', '/nav/cmd_vel_raw'),
        ]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('cmd_vel', '/nav/cmd_vel_raw'),
            ('/cmd_vel', '/nav/cmd_vel_raw'),
        ]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[
            ('cmd_vel', '/nav/cmd_vel_raw'),
            ('/cmd_vel', '/nav/cmd_vel_raw'),
            ('smoothed_cmd_vel', '/nav/cmd_vel'),
            ('/smoothed_cmd_vel', '/nav/cmd_vel'),
            ('cmd_vel_smoothed', '/nav/cmd_vel'),
            ('/cmd_vel_smoothed', '/nav/cmd_vel'),
        ]
    )

    configure_slam = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/map_saver', 'configure'], output='screen'),
        ]
    )

    activate_slam = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/map_saver', 'activate'], output='screen'),
        ]
    )

    configure_nav = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/planner_server', 'configure'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'configure'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/bt_navigator', 'configure'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/behavior_server', 'configure'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/smoother_server', 'configure'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/velocity_smoother', 'configure'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/waypoint_follower', 'configure'], output='screen'),
        ]
    )

    activate_nav = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/planner_server', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/bt_navigator', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/behavior_server', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/smoother_server', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/velocity_smoother', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/waypoint_follower', 'activate'], output='screen'),
        ]
    )

    reactivate_critical_nav = TimerAction(
        period=13.0,
        actions=[
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'activate'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/bt_navigator', 'activate'], output='screen'),
        ]
    )

    rviz = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
            )
        ]
    )

    return LaunchDescription([
        ground_truth_bridge,
        pointcloud_to_scan,
        slam_toolbox,
        map_saver,
        planner_server,
        controller_server,
        behavior_server,
        smoother_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        configure_slam,
        activate_slam,
        configure_nav,
        activate_nav,
        reactivate_critical_nav,
        rviz,
    ])