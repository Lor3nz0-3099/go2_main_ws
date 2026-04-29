#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray, String


class MyoEmgEventsNode(Node):
    def __init__(self):
        super().__init__('myo_emg_events_node')

        # EMG preprocessing
        self.declare_parameter('emg_window_size', 12)
        self.declare_parameter('activation_on_threshold', 16.0)
        self.declare_parameter('activation_off_threshold', 9.0)

        # Timing thresholds (seconds)
        self.declare_parameter('short_min_duration', 0.08)
        self.declare_parameter('short_max_duration', 0.60)
        self.declare_parameter('long_activation_duration', 1.20)
        self.declare_parameter('double_activation_window', 0.90)

        # Event cooldown to prevent multiple events from a single activation (seconds)
        self.declare_parameter('event_cooldown_sec', 0.60)
        self.event_cooldown_sec = float(self.get_parameter('event_cooldown_sec').value)
        self.last_event_time = None

        self.emg_window_size = int(self.get_parameter('emg_window_size').value)
        self.activation_on_threshold = float(self.get_parameter('activation_on_threshold').value)
        self.activation_off_threshold = float(self.get_parameter('activation_off_threshold').value)

        self.short_min_duration = float(self.get_parameter('short_min_duration').value)
        self.short_max_duration = float(self.get_parameter('short_max_duration').value)
        self.long_activation_duration = float(self.get_parameter('long_activation_duration').value)
        self.double_activation_window = float(self.get_parameter('double_activation_window').value)

        self.emg_sub = self.create_subscription(
            Int16MultiArray,
            '/myo/emg',
            self.emg_callback,
            10
        )

        self.event_pub = self.create_publisher(String, '/myo/event', 10)

        self.emg_metric_window = deque(maxlen=self.emg_window_size)
        self.current_emg_metric = 0.0

        self.active = False
        self.activation_start_time = None
        self.long_event_sent = False

        # For double-short detection
        self.pending_short = False
        self.pending_short_time = None

        self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('myo_emg_events_node started')
        self.get_logger().info(
            f'EMG thresholds ON={self.activation_on_threshold:.1f}, OFF={self.activation_off_threshold:.1f}'
        )

    def in_cooldown(self):
        if self.last_event_time is None:
            return False
        now = self.get_clock().now()
        dt = (now - self.last_event_time).nanoseconds / 1e9
        return dt < self.event_cooldown_sec
    
    def emg_callback(self, msg: Int16MultiArray):
        
        if self.in_cooldown():
            return
        
        if not msg.data:
            return

        mean_abs_emg = sum(abs(int(v)) for v in msg.data) / len(msg.data)

        self.emg_metric_window.append(mean_abs_emg)
        self.current_emg_metric = sum(self.emg_metric_window) / len(self.emg_metric_window)

        now = self.get_clock().now()

        # Rising edge: activation starts
        if not self.active and self.current_emg_metric >= self.activation_on_threshold:
            self.active = True
            self.activation_start_time = now
            self.long_event_sent = False
            self.get_logger().info(f'EMG ACTIVE | metric={self.current_emg_metric:.2f}')
            return

        # While active: detect long activation
        if self.active and not self.long_event_sent:
            active_duration = (now - self.activation_start_time).nanoseconds / 1e9
            if active_duration >= self.long_activation_duration:
                self.publish_event('stop')
                self.long_event_sent = True
                self.pending_short = False
                self.pending_short_time = None
                self.get_logger().info(
                    f'LONG ACTIVATION -> STOP | duration={active_duration:.2f}s metric={self.current_emg_metric:.2f}'
                )
                return

        # Falling edge: activation ends
        if self.active and self.current_emg_metric <= self.activation_off_threshold:
            active_duration = (now - self.activation_start_time).nanoseconds / 1e9

            self.active = False
            self.activation_start_time = None

            # Ignore release of a long activation: stop already emitted
            if self.long_event_sent:
                self.long_event_sent = False
                self.get_logger().info('EMG INACTIVE after long activation')
                return

            # Short activation candidate
            if self.short_min_duration <= active_duration <= self.short_max_duration:
                if self.pending_short:
                    delta = (now - self.pending_short_time).nanoseconds / 1e9
                    if delta <= self.double_activation_window:
                        self.publish_event('toggle_mode')
                        self.pending_short = False
                        self.pending_short_time = None
                        self.get_logger().info(
                            f'DOUBLE SHORT ACTIVATION -> TOGGLE_MODE | dt={delta:.2f}s'
                        )
                    else:
                        # Previous pending short expired; start a new one
                        self.pending_short = True
                        self.pending_short_time = now
                        self.get_logger().info(
                            f'SHORT ACTIVATION pending reset | duration={active_duration:.2f}s'
                        )
                else:
                    self.pending_short = True
                    self.pending_short_time = now
                    self.get_logger().info(
                        f'SHORT ACTIVATION pending reset | duration={active_duration:.2f}s'
                    )
            else:
                self.get_logger().info(
                    f'Activation ignored | duration={active_duration:.2f}s metric={self.current_emg_metric:.2f}'
                )

    def timer_callback(self):
        
        if self.in_cooldown():
            return
        
        # If one short activation is pending and no second one arrives in time -> RESET
        if not self.pending_short or self.pending_short_time is None:
            return

        now = self.get_clock().now()
        wait_time = (now - self.pending_short_time).nanoseconds / 1e9

        if wait_time > self.double_activation_window:
            self.publish_event('reset')
            self.pending_short = False
            self.pending_short_time = None
            self.get_logger().info('SINGLE SHORT ACTIVATION -> RESET')

    def publish_event(self, event_name: str):
        if self.in_cooldown():
            self.get_logger().info(f'Event {event_name} ignored due to cooldown')
            return

        msg = String()
        msg.data = event_name
        self.event_pub.publish(msg)
        self.last_event_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = MyoEmgEventsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()