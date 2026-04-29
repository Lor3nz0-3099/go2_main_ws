#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class LogColors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        self.current_mode = 'IDLE'
        self.previous_mode = 'IDLE'
        self.special_action_running = False
        self.emergency_active = False

        self.teleop_haptic_active = False
        self.teleop_haptic_body_active = False

        self.teleop_myo_active = False
        self.teleop_myo_body_active = False

        self.nav_requested = False
        self.explore_requested = False

        self.last_mode_logged = None
        self.last_cmd_source_logged = None
        self.last_body_source_logged = None

        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.cmd_source_pub = self.create_publisher(String, '/cmd_source', 10)
        self.body_source_pub = self.create_publisher(String, '/body_source', 10)

        self.teleop_haptic_enable_pub = self.create_publisher(Bool, '/teleop_haptic/enabled', 10)
        self.teleop_myo_enable_pub = self.create_publisher(Bool, '/teleop_myo/enabled', 10)
        self.nav_enable_pub = self.create_publisher(Bool, '/nav/enabled', 10)
        self.explore_enable_pub = self.create_publisher(Bool, '/explore/enabled', 10)
        self.special_action_enable_pub = self.create_publisher(Bool, '/special_action/enabled', 10)

        self.create_subscription(String, '/mode_request', self.mode_request_callback, 10)
        self.create_subscription(String, '/special_action/request', self.special_action_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        self.create_subscription(Bool, '/teleop_haptic/active', self.teleop_haptic_callback, 10)
        self.create_subscription(Bool, '/teleop_haptic/body_active', self.teleop_haptic_body_callback, 10)

        self.create_subscription(Bool, '/teleop_myo/active', self.teleop_myo_callback, 10)
        self.create_subscription(Bool, '/teleop_myo/body_active', self.teleop_myo_body_callback, 10)

        self.create_subscription(Bool, '/nav/request', self.nav_request_callback, 10)
        self.create_subscription(Bool, '/explore/request', self.explore_request_callback, 10)
        self.create_subscription(String, '/special_action/status', self.special_action_status_callback, 10)

        self.timer = self.create_timer(0.05, self.update_state)

        self._log_info('Mode Manager started', LogColors.CYAN)

    def _color(self, text: str, color: str) -> str:
        return f'{color}{text}{LogColors.RESET}'

    def _log_info(self, text: str, color: str = LogColors.RESET):
        self.get_logger().info(self._color(text, color))

    def _log_warn(self, text: str, color: str = LogColors.YELLOW):
        self.get_logger().warn(self._color(text, color))

    def _log_mode_change(self, mode: str):
        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[MODE]{LogColors.RESET} {mode}', LogColors.BLUE)
        )

    def _log_cmd_source_change(self, source: str):
        color = {
            'NONE': LogColors.YELLOW,
            'MYO': LogColors.GREEN,
            'HAPTIC': LogColors.CYAN,
            'NAV': LogColors.MAGENTA,
            'EXPLORE': LogColors.BLUE,
        }.get(source, LogColors.RESET)

        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[CMD_SOURCE]{LogColors.RESET} {source}', color)
        )

    def _log_body_source_change(self, source: str):
        color = {
            'NONE': LogColors.YELLOW,
            'MYO': LogColors.GREEN,
            'HAPTIC': LogColors.CYAN,
        }.get(source, LogColors.RESET)

        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[BODY_SOURCE]{LogColors.RESET} {source}', color)
        )

    def mode_request_callback(self, msg: String):
        requested = msg.data.strip().upper()
        valid_modes = ['IDLE', 'TELEOP_MYO', 'TELEOP_HAPTIC', 'NAVIGATION', 'EXPLORATION']
        if requested in valid_modes:
            self.current_mode = requested
            self._log_info(f'Manual mode request: {requested}', LogColors.CYAN)

    def special_action_callback(self, msg: String):
        action = msg.data.strip().lower()
        if action in ['sit', 'stand', 'wave']:
            if not self.special_action_running and not self.emergency_active:
                self.previous_mode = self.current_mode
                self.special_action_running = True
                self.current_mode = 'SPECIAL_ACTION'
                self._log_info(f'Special action requested: {action}', LogColors.MAGENTA)

    def emergency_callback(self, msg: Bool):
        self.emergency_active = msg.data
        if self.emergency_active:
            self.current_mode = 'EMERGENCY_STOP'
            self._log_warn('Emergency stop active', LogColors.RED)

    def teleop_haptic_callback(self, msg: Bool):
        self.teleop_haptic_active = msg.data

    def teleop_haptic_body_callback(self, msg: Bool):
        self.teleop_haptic_body_active = msg.data

    def teleop_myo_callback(self, msg: Bool):
        self.teleop_myo_active = msg.data

    def teleop_myo_body_callback(self, msg: Bool):
        self.teleop_myo_body_active = msg.data

    def nav_request_callback(self, msg: Bool):
        self.nav_requested = msg.data

    def explore_request_callback(self, msg: Bool):
        self.explore_requested = msg.data

    def special_action_status_callback(self, msg: String):
        status = msg.data.strip().upper()
        if self.special_action_running and status == 'DONE':
            self.special_action_running = False
            self.current_mode = self.previous_mode
            self._log_info(
                f'Special action completed, returning to {self.current_mode}',
                LogColors.MAGENTA
            )

    def compute_mode(self) -> str:
        if self.emergency_active:
            return 'EMERGENCY_STOP'
        if self.special_action_running:
            return 'SPECIAL_ACTION'
        if self.teleop_haptic_active:
            return 'TELEOP_HAPTIC'
        if self.teleop_myo_active:
            return 'TELEOP_MYO'
        if self.nav_requested:
            return 'NAVIGATION'
        if self.explore_requested:
            return 'EXPLORATION'
        return 'IDLE'

   
    def compute_cmd_source(self) -> str:
        if self.emergency_active or self.special_action_running:
            return 'NONE'
        if self.teleop_haptic_active:
            return 'HAPTIC'
        if self.teleop_myo_active:
            return 'MYO'
        if self.nav_requested:
            return 'NAV'
        if self.explore_requested:
            return 'NAV'
        return 'NONE'

    def compute_body_source(self) -> str:
        if self.teleop_haptic_body_active:
            return 'HAPTIC'
        if self.teleop_myo_body_active:
            return 'MYO'
        return 'NONE'

    def update_state(self):
        self.current_mode = self.compute_mode()
        cmd_source = self.compute_cmd_source()
        body_source = self.compute_body_source()

        self.publish_mode_and_sources(cmd_source, body_source)
        self.publish_enables()

    def publish_mode_and_sources(self, cmd_source: str, body_source: str):
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

        cmd_msg = String()
        cmd_msg.data = cmd_source
        self.cmd_source_pub.publish(cmd_msg)

        body_msg = String()
        body_msg.data = body_source
        self.body_source_pub.publish(body_msg)

        if self.current_mode != self.last_mode_logged:
            self._log_mode_change(self.current_mode)
            self.last_mode_logged = self.current_mode

        if cmd_source != self.last_cmd_source_logged:
            self._log_cmd_source_change(cmd_source)
            self.last_cmd_source_logged = cmd_source

        if body_source != self.last_body_source_logged:
            self._log_body_source_change(body_source)
            self.last_body_source_logged = body_source

    def publish_enables(self):
        self.publish_bool(self.teleop_haptic_enable_pub, self.current_mode == 'TELEOP_HAPTIC')
        self.publish_bool(self.teleop_myo_enable_pub, self.current_mode == 'TELEOP_MYO')
        self.publish_bool(self.nav_enable_pub, self.current_mode == 'NAVIGATION')
        self.publish_bool(self.explore_enable_pub, self.current_mode == 'EXPLORATION')
        self.publish_bool(self.special_action_enable_pub, self.current_mode == 'SPECIAL_ACTION')

    def publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = value
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()