#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray


class LogColors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


class NavRequestManager(Node):
    def __init__(self):
        super().__init__('nav_request_manager')

        self.nav_request_pub = self.create_publisher(Bool, '/nav/request', 10)
        self.last_nav_active = None

        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )

        self._log_info('NavRequest Manager started', LogColors.CYAN)
        self.publish_nav_request(False, force_log=True)

    def _color(self, text: str, color: str) -> str:
        return f'{color}{text}{LogColors.RESET}'

    def _log_info(self, text: str, color: str = LogColors.RESET):
        self.get_logger().info(self._color(text, color))

    def _log_nav_change(self, active: bool):
        state = 'ACTIVE' if active else 'INACTIVE'
        color = LogColors.GREEN if active else LogColors.YELLOW
        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[NAV_REQUEST]{LogColors.RESET} {state}', color)
        )

    def publish_nav_request(self, value: bool, force_log: bool = False):
        msg = Bool()
        msg.data = value
        self.nav_request_pub.publish(msg)

        if force_log or self.last_nav_active is None or self.last_nav_active != value:
            self._log_nav_change(value)
            self.last_nav_active = value

    def status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            self.publish_nav_request(False)
            return

        latest_status = msg.status_list[-1].status

        # action_msgs/GoalStatus values:
        # 1 ACCEPTED
        # 2 EXECUTING
        # 3 CANCELING
        # 4 SUCCEEDED
        # 5 CANCELED
        # 6 ABORTED

        if latest_status in [1, 2, 3]:
            self.publish_nav_request(True)
        else:
            self.publish_nav_request(False)


def main(args=None):
    rclpy.init(args=args)
    node = NavRequestManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()