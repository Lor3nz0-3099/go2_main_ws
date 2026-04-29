#!/usr/bin/env python3

import queue
import threading
import time

from pymyolinux.core.myo import MyoDongle

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Int16MultiArray, String


class MyoReaderNode(Node):
    def __init__(self):
        super().__init__('myo_reader_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value

        self.imu_pub = self.create_publisher(Imu, '/myo/imu', 10)
        self.emg_pub = self.create_publisher(Int16MultiArray, '/myo/emg', 10)

        self.haptic_sub = self.create_subscription(
            String,
            '/myo/haptic_cmd',
            self.haptic_callback,
            10
        )

        self.packet_count = 0
        self.last_packet_time = time.time()

        self.device_lock = threading.Lock()
        self.haptic_queue = queue.Queue()

        self.get_logger().info(f'Connecting to Myo dongle on {self.serial_port}...')

        self.device = MyoDongle(self.serial_port)

        # Inizializzazione protetta del device
        with self.device_lock:
            self.device.clear_state()
            myo_devices = self.device.discover_myo_devices()

        if len(myo_devices) == 0:
            self.get_logger().error('No Myo devices found.')
            raise RuntimeError('No Myo devices found.')

        self.get_logger().info(f'Found {len(myo_devices)} Myo device(s), connecting to first one...')

        with self.device_lock:
            self.device.connect(myo_devices[0])

            # opzionale: evita sleep automatico
            try:
                self.device.set_sleep_mode(False)
                self.get_logger().info('Myo sleep disabled')
            except Exception as e:
                self.get_logger().warn(f'Could not disable Myo sleep: {e}')

            self.device.enable_imu_readings()
            self.device.enable_emg_readings()
            self.device.add_joint_emg_imu_handler(self.joint_event_handler)

        self.get_logger().info('Myo connected. Publishing /myo/imu and /myo/emg')

        # Timer debug
        self.create_timer(2.0, self.debug_timer_callback)

        # Thread lettura pacchetti
        self.reader_thread = threading.Thread(target=self.read_packets_loop, daemon=True)
        self.reader_thread.start()

    def haptic_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.haptic_queue.put(cmd)
        self.get_logger().info(f'HAPTIC QUEUED -> {cmd}')

    def process_haptic_queue(self):
        while not self.haptic_queue.empty():
            cmd = self.haptic_queue.get_nowait()

            try:
                with self.device_lock:
                    if cmd == 'short':
                        self.device.vibrate_short()
                        self.get_logger().info('HAPTIC -> short')

                    elif cmd == 'double_short':
                        self.device.vibrate_short()
                        time.sleep(0.12)
                        self.device.vibrate_short()
                        self.get_logger().info('HAPTIC -> double_short')

                    elif cmd == 'long':
                        self.device.vibrate_long()
                        self.get_logger().info('HAPTIC -> long')

                    else:
                        self.get_logger().warn(f'Unknown haptic command: {cmd}')

            except Exception as e:
                self.get_logger().error(f'Haptic error: {e}')

    def read_packets_loop(self):
        self.get_logger().info('Starting Myo packet read loop...')
        try:
            while rclpy.ok():
                # Se c'è aptica da fare, falla subito
                if not self.haptic_queue.empty():
                    self.process_haptic_queue()
                    time.sleep(0.001)
                    continue

                # Altrimenti leggi pacchetti con timeout molto corto
                with self.device_lock:
                    self.device.scan_for_data_packets_conditional(0.005)

                time.sleep(0.001)

        except Exception as e:
            self.get_logger().error(f'Myo read loop error: {e}')

        except Exception as e:
            self.get_logger().error(f'Myo read loop error: {e}')

    def debug_timer_callback(self):
        elapsed = time.time() - self.last_packet_time
        self.get_logger().info(
            f'Packets received: {self.packet_count}, last packet {elapsed:.2f}s ago'
        )

    def joint_event_handler(
        self,
        emg_list,
        orient_w, orient_x, orient_y, orient_z,
        accel_1, accel_2, accel_3,
        gyro_1, gyro_2, gyro_3,
        sample_num
    ):
        self.packet_count += 1
        self.last_packet_time = time.time()

        MYOHW_ACCELEROMETER_SCALE = 2048.0
        MYOHW_GYROSCOPE_SCALE = 16.0
        MYOHW_ORIENTATION_SCALE = 16384.0

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'myo_link'

        imu_msg.orientation.w = orient_w / MYOHW_ORIENTATION_SCALE
        imu_msg.orientation.x = orient_x / MYOHW_ORIENTATION_SCALE
        imu_msg.orientation.y = orient_y / MYOHW_ORIENTATION_SCALE
        imu_msg.orientation.z = orient_z / MYOHW_ORIENTATION_SCALE

        imu_msg.linear_acceleration.x = accel_1 / MYOHW_ACCELEROMETER_SCALE
        imu_msg.linear_acceleration.y = accel_2 / MYOHW_ACCELEROMETER_SCALE
        imu_msg.linear_acceleration.z = accel_3 / MYOHW_ACCELEROMETER_SCALE

        imu_msg.angular_velocity.x = gyro_1 / MYOHW_GYROSCOPE_SCALE
        imu_msg.angular_velocity.y = gyro_2 / MYOHW_GYROSCOPE_SCALE
        imu_msg.angular_velocity.z = gyro_3 / MYOHW_GYROSCOPE_SCALE

        self.imu_pub.publish(imu_msg)

        emg_msg = Int16MultiArray()
        emg_msg.data = [int(x) for x in emg_list]
        self.emg_pub.publish(emg_msg)


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = MyoReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f'Error: {e}')
        else:
            print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()