#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class LogColors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


class ActionManager(Node):
    def __init__(self):
        super().__init__('action_manager')

        self.current_action = None
        self.action_timer = None

        self.status_pub = self.create_publisher(String, '/special_action/status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(String, '/special_action/request', self.action_callback, 10)

        self.get_logger().info(
            f'{LogColors.CYAN}[ACTION_MANAGER] mock backend started{LogColors.RESET}'
        )

    def action_callback(self, msg: String):
        action = msg.data.strip().lower()

        if action not in ['sit', 'stand', 'wave']:
            self.get_logger().warn(
                f'{LogColors.YELLOW}[ACTION] unknown action: {action}{LogColors.RESET}'
            )
            return

        if self.current_action is not None:
            self.get_logger().warn(
                f'{LogColors.YELLOW}[ACTION] already running: {self.current_action}{LogColors.RESET}'
            )
            return

        self.current_action = action
        self.publish_stop()

        self.publish_status('RUNNING')

        self.get_logger().info(
            f'{LogColors.MAGENTA}{LogColors.BOLD}[ACTION]{LogColors.RESET} '
            f'{LogColors.MAGENTA}{action.upper()} started mock execution{LogColors.RESET}'
        )

        duration = self.get_action_duration(action)
        self.action_timer = self.create_timer(duration, self.finish_action)

    def get_action_duration(self, action: str) -> float:
        if action == 'sit':
            return 2.0
        if action == 'stand':
            return 1.5
        if action == 'wave':
            return 3.0
        return 2.0

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def finish_action(self):
        if self.action_timer is not None:
            self.action_timer.cancel()
            self.action_timer = None

        finished_action = self.current_action
        self.current_action = None

        self.publish_stop()
        self.publish_status('DONE')

        self.get_logger().info(
            f'{LogColors.GREEN}{LogColors.BOLD}[ACTION]{LogColors.RESET} '
            f'{LogColors.GREEN}{finished_action.upper()} DONE{LogColors.RESET}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ActionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()