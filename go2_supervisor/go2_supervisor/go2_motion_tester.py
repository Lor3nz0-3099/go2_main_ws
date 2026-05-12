#!/usr/bin/env python3

import argparse
import json
import time

import rclpy
from rclpy.node import Node

from unitree_api.msg import Request, Response
from go2_interfaces.srv import BodyHeight


class Go2MotionTester(Node):
    def __init__(self):
        super().__init__('go2_motion_tester')

        self.motion_pub = self.create_publisher(
            Request,
            '/api/motion_switcher/request',
            10
        )

        self.sport_pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )

        self.motion_responses = []
        self.sport_responses = []

        self.create_subscription(
            Response,
            '/api/motion_switcher/response',
            self.motion_response_cb,
            10
        )

        self.create_subscription(
            Response,
            '/api/sport/response',
            self.sport_response_cb,
            10
        )

        self.body_height_client = self.create_client(
            BodyHeight,
            '/body_height'
        )

    def motion_response_cb(self, msg):
        self.motion_responses.append(msg)
        self.get_logger().info(
            f'MOTION RESPONSE api_id={msg.header.identity.api_id} '
            f'code={msg.header.status.code} data={msg.data}'
        )

    def sport_response_cb(self, msg):
        self.sport_responses.append(msg)
        self.get_logger().info(
            f'SPORT RESPONSE api_id={msg.header.identity.api_id} '
            f'code={msg.header.status.code} data={msg.data}'
        )

    @staticmethod
    def make_request(api_id, parameter='{}', request_id=0):
        req = Request()
        req.header.identity.id = int(request_id)
        req.header.identity.api_id = int(api_id)
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = parameter
        req.binary = []
        return req

    def spin_for(self, seconds):
        end_time = time.time() + seconds
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def query_motion_service(self):
        self.motion_responses.clear()

        self.get_logger().info(
            'Query motion service: /api/motion_switcher/request api_id=1001'
        )

        req = self.make_request(api_id=1001, parameter='{}')
        self.motion_pub.publish(req)

        self.spin_for(2.0)

        if not self.motion_responses:
            self.get_logger().warn('No /api/motion_switcher/response received')
            return None

        last = self.motion_responses[-1]

        self.get_logger().info(
            f'CURRENT MOTION SERVICE: '
            f'code={last.header.status.code} data={last.data}'
        )

        return last

    def switch_motion_service(self, name):
        self.motion_responses.clear()

        parameter = json.dumps({"name": name})

        self.get_logger().warn(
            f'Trying motion service switch: '
            f'/api/motion_switcher/request api_id=1002 parameter={parameter}'
        )

        req = self.make_request(api_id=1002, parameter=parameter)
        self.motion_pub.publish(req)

        self.spin_for(3.0)

        if not self.motion_responses:
            self.get_logger().warn(
                'No /api/motion_switcher/response received after switch'
            )
        else:
            last = self.motion_responses[-1]
            self.get_logger().info(
                f'SWITCH RESPONSE: code={last.header.status.code} data={last.data}'
            )

        self.get_logger().info('Querying motion service after switch attempt...')
        return self.query_motion_service()

    def raw_motion_request(self, api_id, parameter):
        self.motion_responses.clear()

        self.get_logger().warn(
            f'Publishing RAW motion_switcher request '
            f'api_id={api_id} parameter={parameter}'
        )

        req = self.make_request(api_id=api_id, parameter=parameter)
        self.motion_pub.publish(req)

        self.spin_for(3.0)

        if not self.motion_responses:
            self.get_logger().warn('No /api/motion_switcher/response received')

    def raw_sport_request(self, api_id, parameter):
        self.sport_responses.clear()

        self.get_logger().warn(
            f'Publishing RAW sport request api_id={api_id} parameter={parameter}'
        )

        req = self.make_request(api_id=api_id, parameter=parameter)
        self.sport_pub.publish(req)

        self.spin_for(3.0)

        if not self.sport_responses:
            self.get_logger().warn('No /api/sport/response received')

    def test_body_height(self, height):
        self.sport_responses.clear()

        if not self.body_height_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/body_height service not available')
            return

        self.get_logger().info(f'Calling /body_height height={height}')

        req = BodyHeight.Request()
        req.height = float(height)

        future = self.body_height_client.call_async(req)

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        try:
            res = future.result()
            self.get_logger().info(
                f'/body_height ROS response: '
                f'success={res.success} message="{res.message}"'
            )
        except Exception as exc:
            self.get_logger().error(f'/body_height call failed: {exc}')
            return

        self.get_logger().info('Waiting for /api/sport/response...')
        self.spin_for(2.0)

        matching = [
            msg for msg in self.sport_responses
            if msg.header.identity.api_id == 1013
        ]

        if not matching:
            self.get_logger().warn('No sport response for api_id=1013 received')
            return

        last = matching[-1]
        code = last.header.status.code

        if code == 0:
            self.get_logger().info('BodyHeight accepted by controller: code=0')
        else:
            self.get_logger().error(
                f'BodyHeight rejected by controller: code={code}'
            )


def main():
    parser = argparse.ArgumentParser(
        description='Safe tester for Go2 motion switcher and BodyHeight'
    )

    subparsers = parser.add_subparsers(dest='command', required=True)

    subparsers.add_parser(
        'status',
        help='Query current motion service via /api/motion_switcher/request api_id=1001'
    )

    switch_parser = subparsers.add_parser(
        'switch',
        help='Try switching motion service by name'
    )
    switch_parser.add_argument(
        '--name',
        type=str,
        required=True,
        choices=['normal', 'ai', 'advanced', 'mcf'],
        help='Motion service name to request'
    )

    body_parser = subparsers.add_parser(
        'bodyheight',
        help='Call /body_height and verify /api/sport/response api_id=1013'
    )
    body_parser.add_argument(
        '--height',
        type=float,
        default=-0.02,
        help='Relative body height. Use small values only. Default: -0.02'
    )

    raw_motion_parser = subparsers.add_parser(
        'raw-motion',
        help='Expert only: publish a raw /api/motion_switcher/request'
    )
    raw_motion_parser.add_argument('--api-id', type=int, required=True)
    raw_motion_parser.add_argument('--parameter', type=str, default='{}')

    raw_sport_parser = subparsers.add_parser(
        'raw-sport',
        help='Expert only: publish a raw /api/sport/request'
    )
    raw_sport_parser.add_argument('--api-id', type=int, required=True)
    raw_sport_parser.add_argument('--parameter', type=str, default='{}')

    args = parser.parse_args()

    rclpy.init()
    node = Go2MotionTester()

    try:
        # Allow DDS discovery to settle.
        node.spin_for(0.5)

        if args.command == 'status':
            node.query_motion_service()

        elif args.command == 'switch':
            node.switch_motion_service(args.name)

        elif args.command == 'bodyheight':
            node.test_body_height(args.height)

        elif args.command == 'raw-motion':
            json.loads(args.parameter)
            node.raw_motion_request(args.api_id, args.parameter)

        elif args.command == 'raw-sport':
            json.loads(args.parameter)
            node.raw_sport_request(args.api_id, args.parameter)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()