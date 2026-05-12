#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Pose, Quaternion
from omni_msgs.msg import OmniState, OmniButtonEvent, OmniFeedback
from std_msgs.msg import Bool


class LogColors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class HapticGo2TeleopPositionNode(Node):
    def __init__(self) -> None:
        super().__init__('haptic_go2_teleop_position_node')

        # Topics
        self.declare_parameter('phantom_state_topic', '/phantom/state')
        self.declare_parameter('phantom_button_topic', '/phantom/button')
        self.declare_parameter('cmd_vel_topic', '/teleop_haptic/cmd_vel')
        self.declare_parameter('body_pose_topic', '/teleop_haptic/body_pose')
        self.declare_parameter('force_feedback_topic', '/phantom/force_feedback')

        # Position mapping axes
        self.declare_parameter('linear_axis', 'y')
        self.declare_parameter('angular_axis', 'x')
        self.declare_parameter('height_axis', 'z')

        # Sign conventions
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('invert_lateral', True)
        self.declare_parameter('invert_height', False)
        self.declare_parameter('invert_body_roll', False)
        self.declare_parameter('invert_body_pitch', False)

        # Reverse steering correction
        self.declare_parameter('invert_angular_when_reversing', True)

        # Output limits
        self.declare_parameter('max_linear', 0.32)
        self.declare_parameter('max_lateral', 0.15)
        self.declare_parameter('max_angular', 0.70)

        # Minimum useful commands once active
        self.declare_parameter('min_linear_active', 0.20)
        self.declare_parameter('min_lateral_active', 0.08)
        self.declare_parameter('min_angular_active', 0.25)

        # Position processing
        self.declare_parameter('deadzone_distance', 8.0)
        self.declare_parameter('max_distance', 35.0)

        # Roll processing for angular.z
        self.declare_parameter('deadzone_roll', 0.16)
        self.declare_parameter('max_roll', 0.55)

        # Height/body processing
        self.declare_parameter('deadzone_height', 5.0)
        self.declare_parameter('max_height_distance', 25.0)
        self.declare_parameter('max_body_height', 0.03)
        self.declare_parameter('body_filter_alpha', 0.30)
        self.declare_parameter('max_height_rate', 0.12)

        # Body orientation processing
        self.declare_parameter('deadzone_body_angle', 0.10)
        self.declare_parameter('max_body_angle_input', 0.45)
        self.declare_parameter('max_body_roll', 0.25)
        self.declare_parameter('max_body_pitch', 0.25)
        self.declare_parameter('body_angle_filter_alpha', 0.30)
        self.declare_parameter('max_body_roll_rate', 0.80)
        self.declare_parameter('max_body_pitch_rate', 0.80)

        # Timing
        self.declare_parameter('publish_rate', 40.0)
        self.declare_parameter('enable_timeout', True)
        self.declare_parameter('timeout_sec', 0.2)

        # Buttons
        self.declare_parameter('enable_deadman', True)
        self.declare_parameter('deadman_button', 'grey')
        self.declare_parameter('recalibrate_with_white_button', True)

        # Smoothing
        self.declare_parameter('filter_alpha', 0.35)

        # Rate limiting
        self.declare_parameter('max_linear_accel', 1.20)
        self.declare_parameter('max_lateral_accel', 1.00)
        self.declare_parameter('max_angular_accel', 2.50)

        # Haptic spring feedback
        self.declare_parameter('enable_force_feedback', True)
        self.declare_parameter('spring_k_x', 0.015)
        self.declare_parameter('spring_k_y', 0.015)
        self.declare_parameter('spring_k_z', 0.035)
        self.declare_parameter('damping_b_x', 0.0)
        self.declare_parameter('damping_b_y', 0.0)
        self.declare_parameter('damping_b_z', 0.0)
        self.declare_parameter('max_force_x', 1.5)
        self.declare_parameter('max_force_y', 1.5)
        self.declare_parameter('max_force_z', 2.2)

        # Debug
        self.declare_parameter('debug', True)

        self.phantom_state_topic = self.get_parameter('phantom_state_topic').value
        self.phantom_button_topic = self.get_parameter('phantom_button_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.body_pose_topic = self.get_parameter('body_pose_topic').value
        self.force_feedback_topic = self.get_parameter('force_feedback_topic').value

        self.linear_axis = str(self.get_parameter('linear_axis').value).strip().lower()
        self.angular_axis = str(self.get_parameter('angular_axis').value).strip().lower()
        self.height_axis = str(self.get_parameter('height_axis').value).strip().lower()

        self.invert_linear = bool(self.get_parameter('invert_linear').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)
        self.invert_lateral = bool(self.get_parameter('invert_lateral').value)
        self.invert_height = bool(self.get_parameter('invert_height').value)
        self.invert_body_roll = bool(self.get_parameter('invert_body_roll').value)
        self.invert_body_pitch = bool(self.get_parameter('invert_body_pitch').value)
        self.invert_angular_when_reversing = bool(
            self.get_parameter('invert_angular_when_reversing').value
        )

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_lateral = float(self.get_parameter('max_lateral').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.min_linear_active = float(self.get_parameter('min_linear_active').value)
        self.min_lateral_active = float(self.get_parameter('min_lateral_active').value)
        self.min_angular_active = float(self.get_parameter('min_angular_active').value)

        self.deadzone_distance = float(self.get_parameter('deadzone_distance').value)
        self.max_distance = float(self.get_parameter('max_distance').value)

        self.deadzone_roll = float(self.get_parameter('deadzone_roll').value)
        self.max_roll = float(self.get_parameter('max_roll').value)

        self.deadzone_height = float(self.get_parameter('deadzone_height').value)
        self.max_height_distance = float(self.get_parameter('max_height_distance').value)
        self.max_body_height = float(self.get_parameter('max_body_height').value)
        self.body_filter_alpha = float(self.get_parameter('body_filter_alpha').value)
        self.max_height_rate = float(self.get_parameter('max_height_rate').value)

        self.deadzone_body_angle = float(self.get_parameter('deadzone_body_angle').value)
        self.max_body_angle_input = float(self.get_parameter('max_body_angle_input').value)
        self.max_body_roll = float(self.get_parameter('max_body_roll').value)
        self.max_body_pitch = float(self.get_parameter('max_body_pitch').value)
        self.body_angle_filter_alpha = float(self.get_parameter('body_angle_filter_alpha').value)
        self.max_body_roll_rate = float(self.get_parameter('max_body_roll_rate').value)
        self.max_body_pitch_rate = float(self.get_parameter('max_body_pitch_rate').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.enable_timeout = bool(self.get_parameter('enable_timeout').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.enable_deadman = bool(self.get_parameter('enable_deadman').value)
        self.deadman_button = str(self.get_parameter('deadman_button').value).strip().lower()
        self.recalibrate_with_white_button = bool(
            self.get_parameter('recalibrate_with_white_button').value
        )

        self.filter_alpha = float(self.get_parameter('filter_alpha').value)

        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_lateral_accel = float(self.get_parameter('max_lateral_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)

        self.enable_force_feedback = bool(self.get_parameter('enable_force_feedback').value)
        self.spring_k_x = float(self.get_parameter('spring_k_x').value)
        self.spring_k_y = float(self.get_parameter('spring_k_y').value)
        self.spring_k_z = float(self.get_parameter('spring_k_z').value)
        self.damping_b_x = float(self.get_parameter('damping_b_x').value)
        self.damping_b_y = float(self.get_parameter('damping_b_y').value)
        self.damping_b_z = float(self.get_parameter('damping_b_z').value)
        self.max_force_x = float(self.get_parameter('max_force_x').value)
        self.max_force_y = float(self.get_parameter('max_force_y').value)
        self.max_force_z = float(self.get_parameter('max_force_z').value)

        self.debug = bool(self.get_parameter('debug').value)

        # Internal state
        self.last_state_time: Optional[float] = None
        self.last_publish_time: Optional[float] = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_z = 0.0

        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw = 0.0

        self.reference_initialized = False

        self.grey_button = 0
        self.white_button = 0
        self.prev_white_button = 0

        self.filtered_linear = 0.0
        self.filtered_lateral = 0.0
        self.filtered_angular = 0.0
        self.filtered_body_height = 0.0
        self.filtered_body_roll = 0.0
        self.filtered_body_pitch = 0.0

        self.last_debug_log_time = 0.0
        self.last_active_state = None

        # QoS
        self.sub_qos = qos_profile_sensor_data
        self.pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ROS interfaces
        self.state_sub = self.create_subscription(
            OmniState,
            self.phantom_state_topic,
            self.phantom_state_callback,
            self.sub_qos
        )

        self.button_sub = self.create_subscription(
            OmniButtonEvent,
            self.phantom_button_topic,
            self.phantom_button_callback,
            self.sub_qos
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, self.pub_qos)
        self.body_pose_pub = self.create_publisher(Pose, self.body_pose_topic, self.pub_qos)
        self.force_pub = self.create_publisher(OmniFeedback, self.force_feedback_topic, self.pub_qos)
        self.active_pub = self.create_publisher(Bool, '/teleop_haptic/active', 10)
        self.body_active_pub = self.create_publisher(Bool, '/teleop_haptic/body_active', 10)

        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_cmd_callback)

        self._log_info(
            f'started | cmd_vel_topic={self.cmd_vel_topic} | body_pose_topic={self.body_pose_topic}',
            LogColors.CYAN
        )
        self._log_info(
            f'Mapping: {self.linear_axis}->vx, {self.angular_axis}->vy, roll->wz, '
            f'{self.height_axis}->body_z, pitch->body_pitch, yaw->body_roll',
            LogColors.BLUE
        )
        self.publish_active(force_log=True)

    def _color(self, text: str, color: str) -> str:
        return f'{color}{text}{LogColors.RESET}'

    def _log_info(self, text: str, color: str = LogColors.RESET) -> None:
        self.get_logger().info(self._color(text, color))

    def _log_warn(self, text: str, color: str = LogColors.YELLOW) -> None:
        self.get_logger().warn(self._color(text, color))

    def _log_active_change(self, active: bool) -> None:
        state = 'ACTIVE' if active else 'INACTIVE'
        color = LogColors.GREEN if active else LogColors.YELLOW
        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[HAPTIC]{LogColors.RESET} teleop_haptic={state}', color)
        )

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    @staticmethod
    def quaternion_to_euler(x: float, y: float, z: float, w: float):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def get_axis_value(self, axis_name: str) -> float:
        if axis_name == 'x':
            return self.current_x
        if axis_name == 'y':
            return self.current_y
        if axis_name == 'z':
            return self.current_z
        self._log_warn(f"Invalid axis '{axis_name}', fallback to x.")
        return self.current_x

    def get_ref_axis_value(self, axis_name: str) -> float:
        if axis_name == 'x':
            return self.ref_x
        if axis_name == 'y':
            return self.ref_y
        if axis_name == 'z':
            return self.ref_z
        return self.ref_x

    def distance_to_command(self, delta: float, min_active: float, max_active: float) -> float:
        sign = 1.0 if delta >= 0.0 else -1.0
        mag = abs(delta)

        if mag < self.deadzone_distance:
            return 0.0

        usable_range = max(self.max_distance - self.deadzone_distance, 1e-6)
        u = (mag - self.deadzone_distance) / usable_range
        u = self.clamp(u, 0.0, 1.0)

        cmd_mag = min_active + u * (max_active - min_active)
        cmd_mag = self.clamp(cmd_mag, 0.0, max_active)
        return sign * cmd_mag

    def roll_to_command(self, delta_roll: float, min_active: float, max_active: float) -> float:
        sign = 1.0 if delta_roll >= 0.0 else -1.0
        mag = abs(delta_roll)

        if mag < self.deadzone_roll:
            return 0.0

        usable_range = max(self.max_roll - self.deadzone_roll, 1e-6)
        u = (mag - self.deadzone_roll) / usable_range
        u = self.clamp(u, 0.0, 1.0)

        cmd_mag = min_active + u * (max_active - min_active)
        cmd_mag = self.clamp(cmd_mag, 0.0, max_active)
        return sign * cmd_mag

    def height_to_body_z(self, delta: float) -> float:
        sign = 1.0 if delta >= 0.0 else -1.0
        mag = abs(delta)

        if mag < self.deadzone_height:
            return 0.0

        usable_range = max(self.max_height_distance - self.deadzone_height, 1e-6)
        u = (mag - self.deadzone_height) / usable_range
        u = self.clamp(u, 0.0, 1.0)

        return sign * (u * self.max_body_height)

    def body_angle_to_command(self, delta_angle: float, max_body_angle: float) -> float:
        sign = 1.0 if delta_angle >= 0.0 else -1.0
        mag = abs(delta_angle)

        if mag < self.deadzone_body_angle:
            return 0.0

        usable_range = max(self.max_body_angle_input - self.deadzone_body_angle, 1e-6)
        u = (mag - self.deadzone_body_angle) / usable_range
        u = self.clamp(u, 0.0, 1.0)

        return sign * (u * max_body_angle)

    def low_pass(self, new_value: float, prev_value: float, alpha: float) -> float:
        a = self.clamp(alpha, 0.0, 1.0)
        return a * new_value + (1.0 - a) * prev_value

    def rate_limit(self, target: float, current: float, max_rate: float, dt: float) -> float:
        if dt <= 0.0:
            return target
        max_delta = max_rate * dt
        delta = target - current
        delta = self.clamp(delta, -max_delta, max_delta)
        return current + delta

    def build_twist(self, linear_x: float, linear_y: float, angular_z: float) -> Twist:
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        return msg

    def build_body_pose(self, body_height: float, body_roll: float, body_pitch: float) -> Pose:
        msg = Pose()
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = body_height
        msg.orientation = euler_to_quaternion(body_roll, body_pitch, 0.0)
        return msg

    def build_force_feedback(self, fx: float, fy: float, fz: float) -> OmniFeedback:
        msg = OmniFeedback()
        msg.force.x = fx
        msg.force.y = fy
        msg.force.z = fz
        msg.position.x = self.ref_x
        msg.position.y = self.ref_y
        msg.position.z = self.ref_z
        return msg

    def is_input_recent(self) -> bool:
        if self.last_state_time is None:
            return False
        if not self.enable_timeout:
            return True
        return (time.monotonic() - self.last_state_time) <= self.timeout_sec

    def is_haptic_active(self) -> bool:
        if not self.reference_initialized:
            return False
        if not self.deadman_pressed():
            return False
        if not self.is_input_recent():
            return False

        motion_eps = 0.01
        return (
            abs(self.filtered_linear) > motion_eps or
            abs(self.filtered_lateral) > motion_eps or
            abs(self.filtered_angular) > motion_eps or
            abs(self.filtered_body_height) > motion_eps or
            abs(self.filtered_body_roll) > motion_eps or
            abs(self.filtered_body_pitch) > motion_eps
        )

    def publish_active(self, force_log: bool = False) -> None:
        active = self.is_haptic_active()

        msg = Bool()
        msg.data = active
        self.active_pub.publish(msg)

        if force_log or self.last_active_state is None or self.last_active_state != active:
            self._log_active_change(active)
            self.last_active_state = active

    def publish_body_active(self) -> None:
        active = self.is_haptic_active()

        msg = Bool()
        msg.data = active
        self.body_active_pub.publish(msg)

    def publish_zero(self) -> None:
        self.filtered_linear = 0.0
        self.filtered_lateral = 0.0
        self.filtered_angular = 0.0
        self.filtered_body_height = 0.0
        self.filtered_body_roll = 0.0
        self.filtered_body_pitch = 0.0

        self.cmd_pub.publish(Twist())
        self.body_pose_pub.publish(self.build_body_pose(0.0, 0.0, 0.0))
        self.force_pub.publish(self.build_force_feedback(0.0, 0.0, 0.0))
        self.publish_active()
        self.publish_body_active()

    def calibrate_reference(self) -> None:
        self.ref_x = self.current_x
        self.ref_y = self.current_y
        self.ref_z = self.current_z

        self.ref_roll = self.current_roll
        self.ref_pitch = self.current_pitch
        self.ref_yaw = self.current_yaw

        self.reference_initialized = True

        self._log_info(
            f'{LogColors.BOLD}[CALIBRATION]{LogColors.RESET} '
            f'pos=({self.ref_x:.3f}, {self.ref_y:.3f}, {self.ref_z:.3f}) '
            f'rpy=({self.ref_roll:.3f}, {self.ref_pitch:.3f}, {self.ref_yaw:.3f})',
            LogColors.CYAN
        )
        self.publish_active(force_log=True)
        self.publish_body_active()

    def deadman_pressed(self) -> bool:
        if not self.enable_deadman:
            return True
        if self.deadman_button == 'grey':
            return self.grey_button != 0
        if self.deadman_button == 'white':
            return self.white_button != 0
        return self.grey_button != 0

    def log_debug_throttled(
        self,
        linear_delta: float,
        lateral_delta: float,
        roll_delta: float,
        height_delta: float,
        pitch_delta: float,
        yaw_delta: float,
    ) -> None:
        if not self.debug:
            return

        now = time.monotonic()
        if now - self.last_debug_log_time < 0.5:
            return
        self.last_debug_log_time = now

        self._log_info(
            f'[DEBUG] '
            f'd_lin={linear_delta:.3f} '
            f'd_lat={lateral_delta:.3f} '
            f'd_roll={roll_delta:.3f} '
            f'd_h={height_delta:.3f} '
            f'd_pitch={pitch_delta:.3f} '
            f'd_yaw={yaw_delta:.3f} | '
            f'vx={self.filtered_linear:.3f} '
            f'vy={self.filtered_lateral:.3f} '
            f'wz={self.filtered_angular:.3f} '
            f'body_z={self.filtered_body_height:.3f} '
            f'body_roll={self.filtered_body_roll:.3f} '
            f'body_pitch={self.filtered_body_pitch:.3f}',
            LogColors.GREEN
        )

    def phantom_state_callback(self, msg: OmniState) -> None:
        try:
            self.current_x = float(msg.pose.position.x)
            self.current_y = float(msg.pose.position.y)
            self.current_z = float(msg.pose.position.z)

            self.current_vx = float(msg.velocity.x)
            self.current_vy = float(msg.velocity.y)
            self.current_vz = float(msg.velocity.z)

            qx = float(msg.pose.orientation.x)
            qy = float(msg.pose.orientation.y)
            qz = float(msg.pose.orientation.z)
            qw = float(msg.pose.orientation.w)
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

            self.current_roll = roll
            self.current_pitch = pitch
            self.current_yaw = yaw

            self.last_state_time = time.monotonic()

            if not self.reference_initialized:
                self.calibrate_reference()

        except Exception as exc:
            self._log_warn(f'Failed to parse OmniState: {exc}')

    def phantom_button_callback(self, msg: OmniButtonEvent) -> None:
        try:
            self.grey_button = int(msg.grey_button)
            new_white = int(msg.white_button)

            if self.recalibrate_with_white_button:
                white_rising_edge = (self.prev_white_button == 0 and new_white != 0)
                if white_rising_edge:
                    self.calibrate_reference()

            self.white_button = new_white
            self.prev_white_button = new_white

        except Exception as exc:
            self._log_warn(f'Failed to parse OmniButtonEvent: {exc}')

    def publish_force_feedback(self) -> None:
        if not self.enable_force_feedback or not self.reference_initialized:
            self.force_pub.publish(self.build_force_feedback(0.0, 0.0, 0.0))
            return

        dx = self.current_x - self.ref_x
        dy = self.current_y - self.ref_y
        dz = self.current_z - self.ref_z

        fx = -self.spring_k_x * dx - self.damping_b_x * self.current_vx
        fy = -self.spring_k_y * dy - self.damping_b_y * self.current_vy
        fz = -self.spring_k_z * dz - self.damping_b_z * self.current_vz

        fx = self.clamp(fx, -self.max_force_x, self.max_force_x)
        fy = self.clamp(fy, -self.max_force_y, self.max_force_y)
        fz = self.clamp(fz, -self.max_force_z, self.max_force_z)

        self.force_pub.publish(self.build_force_feedback(fx, fy, fz))

    def publish_cmd_callback(self) -> None:
        now = time.monotonic()
        if self.last_publish_time is None:
            dt = 1.0 / max(self.publish_rate, 1.0)
        else:
            dt = max(now - self.last_publish_time, 1e-3)
        self.last_publish_time = now

        if self.last_state_time is None:
            self.publish_zero()
            return

        if self.enable_timeout and (now - self.last_state_time) > self.timeout_sec:
            self.publish_zero()
            return

        if not self.deadman_pressed():
            self.publish_zero()
            return

        if not self.reference_initialized:
            self.publish_zero()
            return

        linear_delta = self.get_axis_value(self.linear_axis) - self.get_ref_axis_value(self.linear_axis)
        lateral_delta = self.get_axis_value(self.angular_axis) - self.get_ref_axis_value(self.angular_axis)
        height_delta = self.get_axis_value(self.height_axis) - self.get_ref_axis_value(self.height_axis)
        roll_delta = self.current_roll - self.ref_roll
        pitch_delta = self.current_pitch - self.ref_pitch
        yaw_delta = self.current_yaw - self.ref_yaw

        raw_linear_cmd = self.distance_to_command(
            linear_delta, self.min_linear_active, self.max_linear
        )
        raw_lateral_cmd = self.distance_to_command(
            lateral_delta, self.min_lateral_active, self.max_lateral
        )
        raw_angular_cmd = self.roll_to_command(
            roll_delta, self.min_angular_active, self.max_angular
        )
        raw_height_cmd = self.height_to_body_z(height_delta)

        raw_body_pitch = self.body_angle_to_command(
            pitch_delta, self.max_body_pitch
        )
        raw_body_roll = self.body_angle_to_command(
            yaw_delta, self.max_body_roll
        )

        cmd_linear = -raw_linear_cmd if self.invert_linear else raw_linear_cmd
        cmd_angular = -raw_angular_cmd if self.invert_angular else raw_angular_cmd
        cmd_lateral = -raw_lateral_cmd if self.invert_lateral else raw_lateral_cmd
        cmd_height = -raw_height_cmd if self.invert_height else raw_height_cmd
        cmd_body_roll = -raw_body_roll if self.invert_body_roll else raw_body_roll
        cmd_body_pitch = -raw_body_pitch if self.invert_body_pitch else raw_body_pitch

        if self.invert_angular_when_reversing and cmd_linear < 0.0:
            cmd_angular = -cmd_angular

        filtered_linear = self.low_pass(cmd_linear, self.filtered_linear, self.filter_alpha)
        filtered_lateral = self.low_pass(cmd_lateral, self.filtered_lateral, self.filter_alpha)
        filtered_angular = self.low_pass(cmd_angular, self.filtered_angular, self.filter_alpha)
        filtered_height = self.low_pass(cmd_height, self.filtered_body_height, self.body_filter_alpha)
        filtered_body_roll = self.low_pass(
            cmd_body_roll, self.filtered_body_roll, self.body_angle_filter_alpha
        )
        filtered_body_pitch = self.low_pass(
            cmd_body_pitch, self.filtered_body_pitch, self.body_angle_filter_alpha
        )

        self.filtered_linear = self.rate_limit(
            filtered_linear, self.filtered_linear, self.max_linear_accel, dt
        )
        self.filtered_lateral = self.rate_limit(
            filtered_lateral, self.filtered_lateral, self.max_lateral_accel, dt
        )
        self.filtered_angular = self.rate_limit(
            filtered_angular, self.filtered_angular, self.max_angular_accel, dt
        )
        self.filtered_body_height = self.rate_limit(
            filtered_height, self.filtered_body_height, self.max_height_rate, dt
        )
        self.filtered_body_roll = self.rate_limit(
            filtered_body_roll, self.filtered_body_roll, self.max_body_roll_rate, dt
        )
        self.filtered_body_pitch = self.rate_limit(
            filtered_body_pitch, self.filtered_body_pitch, self.max_body_pitch_rate, dt
        )

        self.cmd_pub.publish(
            self.build_twist(
                self.filtered_linear,
                self.filtered_lateral,
                self.filtered_angular
            )
        )

        self.body_pose_pub.publish(
            self.build_body_pose(
                self.filtered_body_height,
                self.filtered_body_roll,
                self.filtered_body_pitch
            )
        )

        self.publish_force_feedback()
        self.publish_active()
        self.publish_body_active()

        self.log_debug_throttled(
            linear_delta,
            lateral_delta,
            roll_delta,
            height_delta,
            pitch_delta,
            yaw_delta,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HapticGo2TeleopPositionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._log_info('Keyboard interrupt received. Stopping robot...', LogColors.YELLOW)
    finally:
        try:
            node.publish_zero()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()