#!/usr/bin/env python3

import argparse
import random
import time
from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_COUNT = 6
TOPIC = "/joint_states"
DEFAULT_TRAJECTORY_DURATION_SEC = 10.0
NOMINAL_PERIOD_SEC = 0.010
PERIOD_JITTER_FRACTION = 0.30
PAUSE_PROBABILITY = 0.02
MAX_PAUSE_SEC = 0.100
NOISE_AMPLITUDE_RAD = 0.01


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish one noisy linear six-joint trajectory to /joint_states."
    )
    parser.add_argument(
        "--joint-names",
        nargs=JOINT_COUNT,
        metavar=("JOINT1", "JOINT2", "JOINT3", "JOINT4", "JOINT5", "JOINT6"),
        help="Joint names in the same order expected by joints_stream.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="Optional random seed for repeatable noise and timing.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=DEFAULT_TRAJECTORY_DURATION_SEC,
        help=(
            "Duration of the linear trajectory in seconds "
            f"(default: {DEFAULT_TRAJECTORY_DURATION_SEC})."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    joint_names = args.joint_names or [
        f"joint_{index}" for index in range(1, JOINT_COUNT + 1)
    ]
    rng = random.Random(args.seed)
    if args.duration_sec <= 0:
        raise SystemExit("--duration-sec must be greater than zero")
    steps = max(1, round(args.duration_sec / NOMINAL_PERIOD_SEC))

    rclpy.init()
    node: Node = Node("joint_trajectory_tester")
    publisher = node.create_publisher(JointState, TOPIC, 10)

    try:
        for step in range(steps + 1):
            if not rclpy.ok():
                break

            fraction = step / steps
            ideal_position = 1.5 * pi * fraction
            if step == 0 or step == steps:
                positions = [ideal_position] * JOINT_COUNT
            else:
                positions = [
                    ideal_position + rng.uniform(
                        -NOISE_AMPLITUDE_RAD, NOISE_AMPLITUDE_RAD
                    )
                    for _ in range(JOINT_COUNT)
                ]

            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = joint_names
            message.position = positions
            publisher.publish(message)

            if step < steps:
                delay = rng.uniform(
                    NOMINAL_PERIOD_SEC * (1 - PERIOD_JITTER_FRACTION),
                    NOMINAL_PERIOD_SEC * (1 + PERIOD_JITTER_FRACTION),
                )
                if rng.random() < PAUSE_PROBABILITY:
                    delay += rng.uniform(0.001, MAX_PAUSE_SEC)
                time.sleep(delay)

        time.sleep(NOMINAL_PERIOD_SEC)
    except KeyboardInterrupt:
        node.get_logger().info("Trajectory publishing interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
