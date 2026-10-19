#!/usr/bin/env python3

import argparse
from collections import deque
import threading
import time
from typing import Deque

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_COUNT = 6
SOURCE_TOPIC = '/joint_states'
COMMAND_TOPIC = '/drive_command'
HISTORY_LENGTH = 25000
DISPLAY_WINDOW_SEC = 20.0
REFRESH_INTERVAL_MS = 100
VELOCITY_LIMIT_RAD_S = 2.0
ACCELERATION_LIMIT_RAD_S2 = 10.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Plot pose, velocity, and acceleration for one joint.'
    )
    parser.add_argument(
        '--joint-index',
        type=int,
        default=1,
        help='1-based joint index to plot (default: 1).',
    )
    return parser.parse_args()


class SingleJointVisualizer(Node):

    def __init__(self, joint_index: int) -> None:
        super().__init__('joint_stream_visualizer2')
        self._joint_index = joint_index
        self._lock = threading.Lock()
        self._start_time = time.monotonic()
        self._positions: dict[str, Deque[tuple[float, float]]] = {
            SOURCE_TOPIC: deque(maxlen=HISTORY_LENGTH),
            COMMAND_TOPIC: deque(maxlen=HISTORY_LENGTH),
        }
        self._velocities: Deque[tuple[float, float]] = deque(maxlen=HISTORY_LENGTH)
        self._accelerations: Deque[tuple[float, float]] = deque(maxlen=HISTORY_LENGTH)
        self._last_drive_sample: tuple[str, float, float, float | None] | None = None

        self.create_subscription(
            JointState,
            SOURCE_TOPIC,
            lambda message: self._on_joint_state(SOURCE_TOPIC, message),
            10,
        )
        self.create_subscription(
            JointState,
            COMMAND_TOPIC,
            lambda message: self._on_joint_state(COMMAND_TOPIC, message),
            10,
        )

    def elapsed_time(self) -> float:
        return time.monotonic() - self._start_time

    def _on_joint_state(self, topic: str, message: JointState) -> None:
        if len(message.position) <= self._joint_index:
            return

        receive_time = time.monotonic()
        elapsed = receive_time - self._start_time
        position = message.position[self._joint_index]

        with self._lock:
            self._positions[topic].append((elapsed, position))
            if topic != COMMAND_TOPIC:
                return

            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            if stamp_ns:
                stamp_source = 'header'
                sample_time = stamp_ns / 1_000_000_000
            else:
                stamp_source = 'receive'
                sample_time = elapsed

            previous = self._last_drive_sample
            if previous is not None and previous[0] == stamp_source:
                delta_time = sample_time - previous[1]
                if delta_time > 0:
                    velocity = (position - previous[2]) / delta_time
                    self._velocities.append((elapsed, velocity))
                    if previous[3] is not None:
                        acceleration = (velocity - previous[3]) / delta_time
                        self._accelerations.append((elapsed, acceleration))
                else:
                    velocity = None
            else:
                velocity = None

            self._last_drive_sample = (
                stamp_source,
                sample_time,
                position,
                velocity,
            )

    def snapshot(
        self,
    ) -> tuple[
        list[tuple[float, float]],
        list[tuple[float, float]],
        list[tuple[float, float]],
        list[tuple[float, float]],
    ]:
        with self._lock:
            return (
                list(self._positions[SOURCE_TOPIC]),
                list(self._positions[COMMAND_TOPIC]),
                list(self._velocities),
                list(self._accelerations),
            )


def main() -> None:
    args = parse_args()
    if not 1 <= args.joint_index <= JOINT_COUNT:
        raise SystemExit(f'--joint-index must be between 1 and {JOINT_COUNT}')

    rclpy.init()
    node = SingleJointVisualizer(args.joint_index - 1)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    pose_axis, velocity_axis, acceleration_axis = axes

    source_pose_line, = pose_axis.plot([], [], label='Tester /joint_states')
    drive_pose_line, = pose_axis.plot([], [], label='Drive /drive_command')
    pose_axis.set_title(f'Joint {args.joint_index} pose')
    pose_axis.set_ylabel('Position (rad)')
    pose_axis.grid(True)
    pose_axis.legend(loc='upper left')

    velocity_line, = velocity_axis.plot([], [], label='Drive velocity')
    velocity_axis.axhline(
        VELOCITY_LIMIT_RAD_S,
        color='red',
        linestyle='--',
        label='Velocity limit (+/-2 rad/s)',
    )
    velocity_axis.axhline(-VELOCITY_LIMIT_RAD_S, color='red', linestyle='--')
    velocity_axis.set_title('Drive velocity')
    velocity_axis.set_ylabel('Velocity (rad/s)')
    velocity_axis.grid(True)
    velocity_axis.legend(loc='upper left')

    acceleration_line, = acceleration_axis.plot([], [], label='Drive acceleration')
    acceleration_axis.axhline(
        ACCELERATION_LIMIT_RAD_S2,
        color='red',
        linestyle='--',
        label='Acceleration limit (+/-10 rad/s²)',
    )
    acceleration_axis.axhline(
        -ACCELERATION_LIMIT_RAD_S2, color='red', linestyle='--'
    )
    acceleration_axis.set_title('Drive acceleration')
    acceleration_axis.set_ylabel('Acceleration (rad/s²)')
    acceleration_axis.set_xlabel('Elapsed time (s)')
    acceleration_axis.grid(True)
    acceleration_axis.legend(loc='upper left')
    figure.suptitle(f'Joint {args.joint_index} stream and motion limits')
    figure.tight_layout()

    def update(_frame: int) -> list[object]:
        source_positions, drive_positions, velocities, accelerations = node.snapshot()
        source_pose_line.set_data(
            [sample[0] for sample in source_positions],
            [sample[1] for sample in source_positions],
        )
        drive_pose_line.set_data(
            [sample[0] for sample in drive_positions],
            [sample[1] for sample in drive_positions],
        )
        velocity_line.set_data(
            [sample[0] for sample in velocities],
            [sample[1] for sample in velocities],
        )
        acceleration_line.set_data(
            [sample[0] for sample in accelerations],
            [sample[1] for sample in accelerations],
        )

        elapsed = node.elapsed_time()
        for axis in axes:
            axis.set_xlim(
                max(0.0, elapsed - DISPLAY_WINDOW_SEC),
                max(DISPLAY_WINDOW_SEC, elapsed),
            )
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        return [
            source_pose_line,
            drive_pose_line,
            velocity_line,
            acceleration_line,
        ]

    animation = FuncAnimation(
        figure,
        update,
        interval=REFRESH_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        del animation
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
