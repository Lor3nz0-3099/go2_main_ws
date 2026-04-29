#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.current_mode = 'IDLE'

        self.last_myo_cmd = Twist()
        self.last_haptic_cmd = Twist()
        self.last_nav_cmd = Twist()
        self.last_explore_cmd = Twist()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(String, '/cmd_source', self.mode_callback, 10)
        self.create_subscription(Twist, '/teleop_myo/cmd_vel', self.myo_callback, 10)
        self.create_subscription(Twist, '/teleop_haptic/cmd_vel', self.haptic_callback, 10)
        self.create_subscription(Twist, '/nav/cmd_vel', self.nav_callback, 10)
        self.create_subscription(Twist, '/explore/cmd_vel', self.explore_callback, 10)

        self.timer = self.create_timer(0.05, self.publish_selected_cmd)

        self.get_logger().info('CmdVel Mux started')

    def mode_callback(self, msg: String):
        self.current_mode = msg.data.strip().upper()

    def myo_callback(self, msg: Twist):
        self.last_myo_cmd = msg

    def haptic_callback(self, msg: Twist):
        self.last_haptic_cmd = msg

    def nav_callback(self, msg: Twist):
        self.last_nav_cmd = msg

    def explore_callback(self, msg: Twist):
        self.last_explore_cmd = msg

    def publish_selected_cmd(self):
        cmd = Twist()

        if self.current_mode == 'MYO':
            cmd = self.last_myo_cmd
        elif self.current_mode == 'HAPTIC':
            cmd = self.last_haptic_cmd
        elif self.current_mode == 'NAV':
            cmd = self.last_nav_cmd
        elif self.current_mode == 'EXPLORE':
            cmd = self.last_explore_cmd
        else:
            cmd = Twist()

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()