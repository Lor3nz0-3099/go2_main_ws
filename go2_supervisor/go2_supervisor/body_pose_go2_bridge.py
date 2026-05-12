#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from go2_interfaces.srv import Euler, BodyHeight, Pose as Go2Pose


def quaternion_to_euler(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

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


class BodyPoseGo2Bridge(Node):
    def __init__(self):
        super().__init__('body_pose_go2_bridge')

        self.declare_parameter('body_pose_topic', '/body_pose')

        # Se true, il bridge chiama /pose true all'avvio e prima dei comandi.
        # Serve per roll/pitch/yaw via /euler.
        self.declare_parameter('enable_pose_on_start', True)

        # Limiti Euler prudenti.
        self.declare_parameter('max_roll', 0.30)
        self.declare_parameter('max_pitch', 0.30)
        self.declare_parameter('max_yaw', 0.30)

        # Range reale visto nel go2_driver:
        # if (height < -0.18 || height > 0.03) reject
        self.declare_parameter('min_height', -0.18)
        self.declare_parameter('max_height', 0.03)

        # Il tuo /body_pose position.z è relativo.
        # Se il Geomagic dà z positivo quando vuoi abbassare il cane,
        # metti invert_body_height=True.
        self.declare_parameter('invert_body_height', True)

        # Log diagnostico throttled.
        self.declare_parameter('debug', True)

        self.body_pose_topic = self.get_parameter('body_pose_topic').value
        self.enable_pose_on_start = bool(self.get_parameter('enable_pose_on_start').value)

        self.max_roll = float(self.get_parameter('max_roll').value)
        self.max_pitch = float(self.get_parameter('max_pitch').value)
        self.max_yaw = float(self.get_parameter('max_yaw').value)

        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.invert_body_height = bool(self.get_parameter('invert_body_height').value)

        self.debug = bool(self.get_parameter('debug').value)

        self.euler_client = self.create_client(Euler, '/euler')
        self.body_height_client = self.create_client(BodyHeight, '/body_height')
        self.pose_client = self.create_client(Go2Pose, '/pose')

        self.create_subscription(
            Pose,
            self.body_pose_topic,
            self.body_pose_callback,
            10
        )

        self.pose_enabled = False
        self.last_height = None

        self.get_logger().info(
            f'body_pose_go2_bridge started | topic={self.body_pose_topic} | '
            f'height_range=[{self.min_height:.3f}, {self.max_height:.3f}] | '
            f'invert_body_height={self.invert_body_height}'
        )

        if self.enable_pose_on_start:
            self.enable_pose_mode()

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def enable_pose_mode(self):
        if not self.pose_client.service_is_ready():
            self.get_logger().warn('/pose service not ready yet')
            return

        req = Go2Pose.Request()
        req.flag = True
        self.pose_client.call_async(req)
        self.pose_enabled = True
        self.get_logger().info('Requested /pose true')

    def body_pose_callback(self, msg: Pose):
        if self.enable_pose_on_start and not self.pose_enabled:
            self.enable_pose_mode()

        roll, pitch, yaw = quaternion_to_euler(msg.orientation)

        roll = self.clamp(roll, -self.max_roll, self.max_roll)
        pitch = self.clamp(pitch, -self.max_pitch, self.max_pitch)
        yaw = self.clamp(yaw, -self.max_yaw, self.max_yaw)

        relative_height = -msg.position.z if self.invert_body_height else msg.position.z
        height = self.clamp(relative_height, self.min_height, self.max_height)

        if self.debug:
            self.get_logger().info(
                f'RX /body_pose z={msg.position.z:.3f} -> /body_height={height:.3f} | '
                f'roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}',
                throttle_duration_sec=0.5
            )

        self.send_euler(roll, pitch, yaw)

        # Importante: inviare body_height sempre, non solo quando cambia.
        # Il Go2 tende a richiedere comandi mantenuti nel tempo.
        self.send_body_height(height)
        self.last_height = height

    def send_euler(self, roll, pitch, yaw):
        if not self.euler_client.service_is_ready():
            self.get_logger().warn('/euler service not ready', throttle_duration_sec=2.0)
            return

        req = Euler.Request()
        req.roll = float(roll)
        req.pitch = float(pitch)
        req.yaw = float(yaw)
        self.euler_client.call_async(req)

    def send_body_height(self, height):
        if not self.body_height_client.service_is_ready():
            self.get_logger().warn('/body_height service not ready', throttle_duration_sec=2.0)
            return

        req = BodyHeight.Request()
        req.height = float(height)
        self.body_height_client.call_async(req)

    def send_neutral(self):
        self.send_euler(0.0, 0.0, 0.0)
        self.send_body_height(0.0)

    def shutdown_pose_mode(self):
        if not self.pose_client.service_is_ready():
            return

        req = Go2Pose.Request()
        req.flag = False
        self.pose_client.call_async(req)

    def destroy_node(self):
        try:
            self.send_neutral()
            self.shutdown_pose_mode()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BodyPoseGo2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()