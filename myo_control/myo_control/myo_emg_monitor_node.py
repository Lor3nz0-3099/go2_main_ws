#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


class MyoEmgMonitorNode(Node):
    def __init__(self):
        super().__init__('myo_emg_monitor_node')
        self.sub = self.create_subscription(
            Int16MultiArray,
            '/myo/emg',
            self.callback,
            10
        )
        self.counter = 0

    def callback(self, msg: Int16MultiArray):
        if not msg.data:
            return

        self.counter += 1
        if self.counter % 20 != 0:
            return

        abs_vals = [abs(int(v)) for v in msg.data]
        mean_abs = sum(abs_vals) / len(abs_vals)

        self.get_logger().info(
            f'abs_emg={abs_vals} | mean_abs={mean_abs:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MyoEmgMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()