from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='myo_control',
            executable='myo_reader_node',
            name='myo_reader_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'}
            ]
        ),

        Node(
            package='myo_control',
            executable='myo_emg_events_node',
            name='myo_emg_events_node',
            output='screen',
            parameters=[
                {'activation_on_threshold': 16.0},
                {'activation_off_threshold': 9.0},
                {'short_min_duration': 0.08},
                {'short_max_duration': 0.60},
                {'long_activation_duration': 1.20},
                {'double_activation_window': 0.90},
                {'event_cooldown_sec': 0.60},
            ]
        ),

        Node(
            package='myo_control',
            executable='myo_to_cmdvel_node',
            name='myo_to_cmdvel_node',
            output='screen',
            parameters=[
                {'cmd_topic': '/teleop_myo/cmd_vel'},
                {'body_topic': '/teleop_myo/body_pose'},

                {'max_linear_x': 0.20},
                {'max_linear_y': 0.15},
                {'max_angular_z': 0.45},

                {'pitch_deadband': 0.18},
                {'roll_deadband': 0.18},
                {'yaw_deadband': 0.22},

                {'pitch_scale': 0.7},
                {'roll_scale': 0.7},
                {'yaw_scale': 0.9},

                {'max_body_height': 0.08},
                {'max_body_roll': 0.30},
                {'max_body_pitch': 0.30},

                {'body_height_scale': 0.15},
                {'body_roll_scale': 1.2},
                {'body_pitch_scale': 1.2},

                {'filter_alpha': 0.10},
                {'imu_timeout_sec': 0.5},
            ]
        ),
    ])