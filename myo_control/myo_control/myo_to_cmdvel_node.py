#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


class LogColors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


def quaternion_to_euler(w, x, y, z):
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


def euler_to_quaternion(roll, pitch, yaw):
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


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def apply_deadband(value, threshold):
    if abs(value) < threshold:
        return 0.0
    return value


def snap_to_zero(value, eps=1e-4):
    if abs(value) < eps:
        return 0.0
    return value


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MyoToCmdVelNode(Node):
    def __init__(self):
        super().__init__('myo_to_cmdvel_node')

        # Topics
        self.declare_parameter('cmd_topic', '/teleop_myo/cmd_vel')
        self.declare_parameter('body_topic', '/teleop_myo/body_pose')

        # Locomotion limits
        self.declare_parameter('max_linear_x', 0.20)
        self.declare_parameter('max_linear_y', 0.15)
        self.declare_parameter('max_angular_z', 0.45)

        # Locomotion deadbands
        self.declare_parameter('pitch_deadband', 0.18)
        self.declare_parameter('roll_deadband', 0.18)
        self.declare_parameter('yaw_deadband', 0.22)

        # Locomotion scales
        self.declare_parameter('pitch_scale', 0.7)
        self.declare_parameter('roll_scale', 0.7)
        self.declare_parameter('yaw_scale', 0.9)

        # Body limits
        self.declare_parameter('max_body_height', 0.08)
        self.declare_parameter('max_body_roll', 0.30)
        self.declare_parameter('max_body_pitch', 0.30)

        # Body scales
        self.declare_parameter('body_height_scale', 0.15)
        self.declare_parameter('body_roll_scale', 1.2)
        self.declare_parameter('body_pitch_scale', 1.2)

        # Common
        self.declare_parameter('filter_alpha', 0.10)
        self.declare_parameter('imu_timeout_sec', 0.5)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.body_topic = self.get_parameter('body_topic').value

        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_linear_y = float(self.get_parameter('max_linear_y').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)

        self.pitch_deadband = float(self.get_parameter('pitch_deadband').value)
        self.roll_deadband = float(self.get_parameter('roll_deadband').value)
        self.yaw_deadband = float(self.get_parameter('yaw_deadband').value)

        self.pitch_scale = float(self.get_parameter('pitch_scale').value)
        self.roll_scale = float(self.get_parameter('roll_scale').value)
        self.yaw_scale = float(self.get_parameter('yaw_scale').value)

        self.max_body_height = float(self.get_parameter('max_body_height').value)
        self.max_body_roll = float(self.get_parameter('max_body_roll').value)
        self.max_body_pitch = float(self.get_parameter('max_body_pitch').value)

        self.body_height_scale = float(self.get_parameter('body_height_scale').value)
        self.body_roll_scale = float(self.get_parameter('body_roll_scale').value)
        self.body_pitch_scale = float(self.get_parameter('body_pitch_scale').value)

        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.body_pub = self.create_publisher(Pose, self.body_topic, 10)
        self.mode_pub = self.create_publisher(String, '/myo/mode', 10)
        self.haptic_pub = self.create_publisher(String, '/myo/haptic_cmd', 10)

        self.active_pub = self.create_publisher(Bool, '/teleop_myo/active', 10)
        self.body_active_pub = self.create_publisher(Bool, '/teleop_myo/body_active', 10)

        self.imu_sub = self.create_subscription(Imu, '/myo/imu', self.imu_callback, 10)
        self.reset_sub = self.create_subscription(Bool, '/myo/reset_reference', self.reset_callback, 10)
        self.event_sub = self.create_subscription(String, '/myo/event', self.event_callback, 10)

        self.current_roll = None
        self.current_pitch = None
        self.current_yaw = None

        self.reference_roll = None
        self.reference_pitch = None
        self.reference_yaw = None

        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0

        self.current_body_height = 0.0
        self.current_body_roll = 0.0
        self.current_body_pitch = 0.0

        self.body_base_height = 0.0
        self.body_base_roll = 0.0
        self.body_base_pitch = 0.0

        self.filtered_body_height_offset = 0.0
        self.filtered_body_roll_offset = 0.0
        self.filtered_body_pitch_offset = 0.0

        self.last_imu_time = self.get_clock().now()
        self.last_debug_log_ns = 0
        self.last_active_state = None
        self.last_body_active_state = None

        self.mode = 'locomotion'
        self.stop_latched = False

        self.create_timer(0.05, self.safety_timer_callback)
        self.create_timer(0.10, self.body_pose_keepalive_callback)

        self._log_info(
            f'started | cmd_topic={self.cmd_topic} | body_topic={self.body_topic}',
            LogColors.CYAN
        )
        self._log_mode_change(None, self.mode, reason='startup')
        self._log_info('Neutralize forearm, then reset reference once.', LogColors.YELLOW)

        self.publish_mode()
        self.publish_active(force_log=True)
        self.publish_body_active(force_log=True)

    def _color(self, text, color):
        return f'{color}{text}{LogColors.RESET}'

    def _log_info(self, text, color=LogColors.RESET):
        self.get_logger().info(self._color(text, color))

    def _log_warn(self, text, color=LogColors.YELLOW):
        self.get_logger().warn(self._color(text, color))

    def _log_error(self, text, color=LogColors.RED):
        self.get_logger().error(self._color(text, color))

    def _log_mode_change(self, old_mode, new_mode, reason=''):
        old_s = old_mode if old_mode is not None else 'NONE'
        msg = f'{LogColors.BOLD}[MODE]{LogColors.RESET} {old_s} -> {new_mode}'
        if reason:
            msg += f' | reason={reason}'
        self.get_logger().info(self._color(msg, LogColors.BLUE))

    def _log_active_change(self, active):
        state = 'ACTIVE' if active else 'INACTIVE'
        color = LogColors.GREEN if active else LogColors.YELLOW
        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[MYO]{LogColors.RESET} teleop_myo={state}', color)
        )

    def _log_body_active_change(self, active):
        state = 'BODY_ACTIVE' if active else 'BODY_INACTIVE'
        color = LogColors.MAGENTA if active else LogColors.YELLOW
        self.get_logger().info(
            self._color(f'{LogColors.BOLD}[MYO]{LogColors.RESET} {state}', color)
        )

    def is_myo_active(self):
        if self.mode != 'locomotion':
            return False

        if self.stop_latched:
            return False

        motion_eps = 0.01

        return (
            abs(self.filtered_linear_x) > motion_eps or
            abs(self.filtered_linear_y) > motion_eps or
            abs(self.filtered_angular_z) > motion_eps
        )

    def is_myo_body_active(self):
        return self.mode == 'body'

    def publish_active(self, force_log=False):
        active = self.is_myo_active()

        msg = Bool()
        msg.data = active
        self.active_pub.publish(msg)

        if force_log or self.last_active_state is None or self.last_active_state != active:
            self._log_active_change(active)
            self.last_active_state = active

    def publish_body_active(self, force_log=False):
        active = self.is_myo_body_active()

        msg = Bool()
        msg.data = active
        self.body_active_pub.publish(msg)

        if force_log or self.last_body_active_state is None or self.last_body_active_state != active:
            self._log_body_active_change(active)
            self.last_body_active_state = active

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def publish_haptic(self, pattern: str):
        msg = String()
        msg.data = pattern
        self.haptic_pub.publish(msg)

    def publish_body_pose(self):
        body_msg = Pose()
        body_msg.position.x = 0.0
        body_msg.position.y = 0.0
        body_msg.position.z = self.current_body_height
        body_msg.orientation = euler_to_quaternion(
            self.current_body_roll,
            self.current_body_pitch,
            0.0
        )
        self.body_pub.publish(body_msg)

    def body_pose_keepalive_callback(self):
        if self.mode == 'body':
            self.publish_body_pose()

    def publish_stop(self):
        if not rclpy.ok():
            return

        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0
        self.cmd_pub.publish(Twist())

    def reset_reference(self):
        if self.current_roll is None or self.current_pitch is None or self.current_yaw is None:
            self._log_warn('Cannot reset reference: IMU not ready')
            return False

        self.reference_roll = self.current_roll
        self.reference_pitch = self.current_pitch
        self.reference_yaw = self.current_yaw

        self.stop_latched = False
        self.filtered_body_height_offset = 0.0
        self.filtered_body_roll_offset = 0.0
        self.filtered_body_pitch_offset = 0.0

        self.publish_stop()
        self._log_info(
            f'{LogColors.BOLD}[RESET]{LogColors.RESET} reference updated | mode={self.mode}',
            LogColors.CYAN
        )
        return True

    def reset_callback(self, msg: Bool):
        if msg.data:
            ok = self.reset_reference()
            self.publish_active()
            self.publish_body_active()
            if ok:
                self.publish_haptic('short')

    def event_callback(self, msg: String):
        event = msg.data.strip().lower()

        if event == 'reset':
            ok = self.reset_reference()
            self.publish_active()
            self.publish_body_active()
            if ok:
                self.publish_haptic('short')
                self._log_info(f'{LogColors.BOLD}[EVENT]{LogColors.RESET} reset', LogColors.CYAN)
            return

        if event == 'toggle_mode':
            old_mode = self.mode
            self.mode = 'body' if self.mode == 'locomotion' else 'locomotion'

            if old_mode == 'locomotion' and self.mode == 'body':
                self.body_base_height = self.current_body_height
                self.body_base_roll = self.current_body_roll
                self.body_base_pitch = self.current_body_pitch

                self.filtered_body_height_offset = 0.0
                self.filtered_body_roll_offset = 0.0
                self.filtered_body_pitch_offset = 0.0

            self.publish_stop()
            self.reset_reference()
            self.publish_mode()
            self.publish_active()
            self.publish_body_active()
            self.publish_haptic('double_short')

            self._log_mode_change(old_mode, self.mode, reason='emg_toggle')
            self._log_info(
                f'body_base: z={self.body_base_height:.3f}, roll={self.body_base_roll:.3f}, pitch={self.body_base_pitch:.3f}',
                LogColors.MAGENTA
            )
            return

        if event == 'stop':
            self.stop_latched = True
            self.publish_stop()
            self.publish_active()
            self.publish_body_active()
            self.publish_haptic('long')
            self._log_warn(
                f'{LogColors.BOLD}[STOP]{LogColors.RESET} stop latched | locomotion disabled until reset',
                LogColors.RED
            )
            return

        self._log_warn(f'Unknown /myo/event received: {event}')

    def imu_callback(self, msg: Imu):
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )

        self.current_roll = roll
        self.current_pitch = pitch
        self.current_yaw = yaw
        self.last_imu_time = self.get_clock().now()

        if self.reference_roll is None or self.reference_pitch is None or self.reference_yaw is None:
            self.reference_roll = roll
            self.reference_pitch = pitch
            self.reference_yaw = yaw
            self._log_info('Initial reference set automatically', LogColors.CYAN)
            self.publish_active()
            self.publish_body_active()
            return

        if self.stop_latched:
            self.publish_stop()
            self.publish_active()
            self.publish_body_active()
            self._debug_log(f'MODE={self.mode} | STOP LATCHED')
            return

        delta_roll = wrap_angle(self.current_roll - self.reference_roll)
        delta_pitch = wrap_angle(self.current_pitch - self.reference_pitch)
        delta_yaw = wrap_angle(self.current_yaw - self.reference_yaw)

        delta_roll = apply_deadband(delta_roll, self.roll_deadband)
        delta_pitch = apply_deadband(delta_pitch, self.pitch_deadband)
        delta_yaw = apply_deadband(delta_yaw, self.yaw_deadband)

        a = self.filter_alpha

        if self.mode == 'locomotion':
            raw_linear_x = -self.pitch_scale * delta_pitch
            raw_linear_y = -self.roll_scale * delta_roll
            raw_angular_z = self.yaw_scale * delta_yaw

            reverse_threshold = 0.05
            if raw_linear_x < -reverse_threshold:
                raw_angular_z = -raw_angular_z

            raw_linear_x = clamp(raw_linear_x, -self.max_linear_x, self.max_linear_x)
            raw_linear_y = clamp(raw_linear_y, -self.max_linear_y, self.max_linear_y)
            raw_angular_z = clamp(raw_angular_z, -self.max_angular_z, self.max_angular_z)

            self.filtered_linear_x = a * raw_linear_x + (1.0 - a) * self.filtered_linear_x
            self.filtered_linear_y = a * raw_linear_y + (1.0 - a) * self.filtered_linear_y
            self.filtered_angular_z = a * raw_angular_z + (1.0 - a) * self.filtered_angular_z

            self.filtered_linear_x = snap_to_zero(self.filtered_linear_x)
            self.filtered_linear_y = snap_to_zero(self.filtered_linear_y)
            self.filtered_angular_z = snap_to_zero(self.filtered_angular_z)

            cmd = Twist()
            cmd.linear.x = self.filtered_linear_x
            cmd.linear.y = self.filtered_linear_y
            cmd.angular.z = self.filtered_angular_z
            self.cmd_pub.publish(cmd)

            self._debug_log(
                f'LOCOMOTION | d_pitch={delta_pitch:.3f} d_roll={delta_roll:.3f} d_yaw={delta_yaw:.3f} | '
                f'vx={cmd.linear.x:.3f} vy={cmd.linear.y:.3f} wz={cmd.angular.z:.3f}'
            )

        elif self.mode == 'body':
            raw_body_height_offset = self.body_height_scale * delta_yaw
            raw_body_roll_offset = self.body_roll_scale * delta_roll
            raw_body_pitch_offset = self.body_pitch_scale * delta_pitch

            raw_body_height_offset = clamp(
                raw_body_height_offset,
                -self.max_body_height,
                self.max_body_height
            )
            raw_body_roll_offset = clamp(
                raw_body_roll_offset,
                -self.max_body_roll,
                self.max_body_roll
            )
            raw_body_pitch_offset = clamp(
                raw_body_pitch_offset,
                -self.max_body_pitch,
                self.max_body_pitch
            )

            self.filtered_body_height_offset = (
                a * raw_body_height_offset + (1.0 - a) * self.filtered_body_height_offset
            )
            self.filtered_body_roll_offset = (
                a * raw_body_roll_offset + (1.0 - a) * self.filtered_body_roll_offset
            )
            self.filtered_body_pitch_offset = (
                a * raw_body_pitch_offset + (1.0 - a) * self.filtered_body_pitch_offset
            )

            self.filtered_body_height_offset = snap_to_zero(self.filtered_body_height_offset)
            self.filtered_body_roll_offset = snap_to_zero(self.filtered_body_roll_offset)
            self.filtered_body_pitch_offset = snap_to_zero(self.filtered_body_pitch_offset)

            self.current_body_height = clamp(
                self.body_base_height + self.filtered_body_height_offset,
                -self.max_body_height,
                self.max_body_height
            )
            self.current_body_roll = clamp(
                self.body_base_roll + self.filtered_body_roll_offset,
                -self.max_body_roll,
                self.max_body_roll
            )
            self.current_body_pitch = clamp(
                self.body_base_pitch + self.filtered_body_pitch_offset,
                -self.max_body_pitch,
                self.max_body_pitch
            )

            self.publish_body_pose()
            self.cmd_pub.publish(Twist())

            self._debug_log(
                f'BODY | d_pitch={delta_pitch:.3f} d_roll={delta_roll:.3f} d_yaw={delta_yaw:.3f} | '
                f'body_z={self.current_body_height:.3f} body_roll={self.current_body_roll:.3f} '
                f'body_pitch={self.current_body_pitch:.3f}'
            )

        self.publish_active()
        self.publish_body_active()

    def safety_timer_callback(self):
        imu_age = (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e9
        if imu_age > self.imu_timeout_sec:
            self.publish_stop()
            self.publish_active()
            self.publish_body_active()

    def _debug_log(self, text: str):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_debug_log_ns > int(1.0e9):
            self.get_logger().info(self._color(f'[DEBUG] {text}', LogColors.GREEN))
            self.last_debug_log_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = MyoToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_stop()
                node.publish_active(force_log=True)
                node.publish_body_active(force_log=True)
        except Exception:
            pass

        node.destroy_node()

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()