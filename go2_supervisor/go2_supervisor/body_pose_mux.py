#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose


class LogColors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


def pose_is_nontrivial(pose: Pose, pos_eps: float = 1e-4, ori_eps: float = 1e-4) -> bool:
    return (
        abs(pose.position.x) > pos_eps or
        abs(pose.position.y) > pos_eps or
        abs(pose.position.z) > pos_eps or
        abs(pose.orientation.x) > ori_eps or
        abs(pose.orientation.y) > ori_eps or
        abs(pose.orientation.z) > ori_eps or
        abs(pose.orientation.w - 1.0) > ori_eps
    )


class BodyPoseMux(Node):
    def __init__(self):
        super().__init__('body_pose_mux')

        self.body_source = 'NONE'

        self.last_myo_body_pose = Pose()
        self.last_haptic_body_pose = Pose()
        self.last_output_pose = Pose()

        self.last_myo_body_pose.orientation.w = 1.0
        self.last_haptic_body_pose.orientation.w = 1.0
        self.last_output_pose.orientation.w = 1.0

        self.last_logged_source = None

        self.body_pose_pub = self.create_publisher(Pose, '/body_pose', 10)

        self.create_subscription(String, '/body_source', self.body_source_callback, 10)
        self.create_subscription(Pose, '/teleop_myo/body_pose', self.myo_body_pose_callback, 10)
        self.create_subscription(Pose, '/teleop_haptic/body_pose', self.haptic_body_pose_callback, 10)

        self.timer = self.create_timer(0.05, self.publish_selected_body_pose)

        self._log_info('BodyPose Mux started', LogColors.CYAN)

    def _color(self, text: str, color: str) -> str:
        return f'{color}{text}{LogColors.RESET}'

    def _log_info(self, text: str, color: str = LogColors.RESET):
        self.get_logger().info(self._color(text, color))

    def _log_source_change(self, source: str):
        color = {
            'NONE': LogColors.YELLOW,
            'MYO': LogColors.MAGENTA,
            'HAPTIC': LogColors.BLUE,
        }.get(source, LogColors.RESET)

        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[BODY_MUX]{LogColors.RESET} source -> {source}', color)
        )

    def body_source_callback(self, msg: String):
        self.body_source = msg.data.strip().upper()

    def myo_body_pose_callback(self, msg: Pose):
        self.last_myo_body_pose = msg

    def haptic_body_pose_callback(self, msg: Pose):
        self.last_haptic_body_pose = msg

    def publish_selected_body_pose(self):
        if self.body_source != self.last_logged_source:
            self._log_source_change(self.body_source)
            self.last_logged_source = self.body_source

        if self.body_source == 'MYO':
            if pose_is_nontrivial(self.last_myo_body_pose):
                self.last_output_pose = self.last_myo_body_pose

        elif self.body_source == 'HAPTIC':
            if pose_is_nontrivial(self.last_haptic_body_pose):
                self.last_output_pose = self.last_haptic_body_pose

        # Se body_source == NONE:
        # mantieni e ripubblica l'ultima pose valida

        self.body_pose_pub.publish(self.last_output_pose)


def main(args=None):
    rclpy.init(args=args)
    node = BodyPoseMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()