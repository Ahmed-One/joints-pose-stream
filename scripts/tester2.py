#!/usr/bin/env python3

import argparse
from math import pi
import random
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_COUNT = 6
TOPIC = '/joint_states'
DEFAULT_TRAJECTORY_DURATION_SEC = 10.0
DEFAULT_INTERMEDIATE_POINTS = 3
TARGET_POSITION_RAD = pi
NOMINAL_PERIOD_SEC = 0.010
PERIOD_JITTER_FRACTION = 0.30
PAUSE_PROBABILITY = 0.02
MAX_PAUSE_SEC = 0.100


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Publish a stepped six-joint trajectory from random positions to pi.'
        )
    )
    parser.add_argument(
        '--joint-names',
        nargs=JOINT_COUNT,
        metavar=('JOINT1', 'JOINT2', 'JOINT3', 'JOINT4', 'JOINT5', 'JOINT6'),
        help='Joint names in the same order expected by joints_stream.',
    )
    parser.add_argument(
        '--seed',
        type=int,
        help='Optional random seed for repeatable starting positions.',
    )
    parser.add_argument(
        '--duration-sec',
        type=float,
        default=DEFAULT_TRAJECTORY_DURATION_SEC,
        help=(
            'Duration of the trajectory in seconds '
            f'(default: {DEFAULT_TRAJECTORY_DURATION_SEC}).'
        ),
    )
    parser.add_argument(
        '--points',
        type=int,
        default=DEFAULT_INTERMEDIATE_POINTS,
        help=(
            'Number of intermediate points between each random start and pi '
            f'(default: {DEFAULT_INTERMEDIATE_POINTS}).'
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration_sec <= 0:
        raise SystemExit('--duration-sec must be greater than zero')
    if args.points < 0:
        raise SystemExit('--points must be zero or greater')

    joint_names = args.joint_names or [
        f'joint_{index}' for index in range(1, JOINT_COUNT + 1)
    ]
    rng = random.Random(args.seed)
    starting_positions = [
        rng.uniform(0.0, TARGET_POSITION_RAD) for _ in range(JOINT_COUNT)
    ]
    hold_duration_sec = args.duration_sec / (args.points + 1)
    held_step_count = args.points + 1

    rclpy.init()
    node: Node = Node('joint_trajectory_tester2')
    publisher = node.create_publisher(JointState, TOPIC, 10)

    try:
        for step in range(held_step_count):
            positions_fraction = step / held_step_count
            positions = [
                start + (TARGET_POSITION_RAD - start) * positions_fraction
                for start in starting_positions
            ]
            hold_end_time = time.monotonic() + hold_duration_sec

            while rclpy.ok() and time.monotonic() < hold_end_time:
                message = JointState()
                message.header.stamp = node.get_clock().now().to_msg()
                message.name = joint_names
                message.position = positions
                publisher.publish(message)

                delay = rng.uniform(
                    NOMINAL_PERIOD_SEC * (1 - PERIOD_JITTER_FRACTION),
                    NOMINAL_PERIOD_SEC * (1 + PERIOD_JITTER_FRACTION),
                )
                if rng.random() < PAUSE_PROBABILITY:
                    delay += rng.uniform(0.001, MAX_PAUSE_SEC)
                remaining = hold_end_time - time.monotonic()
                time.sleep(min(delay, max(0.0, remaining)))

        if rclpy.ok():
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = joint_names
            message.position = [TARGET_POSITION_RAD] * JOINT_COUNT
            publisher.publish(message)
            time.sleep(NOMINAL_PERIOD_SEC)
    except KeyboardInterrupt:
        node.get_logger().info('Stepped trajectory publishing interrupted.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
