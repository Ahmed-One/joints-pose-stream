#!/usr/bin/env python3

from collections import deque
import threading
import time
from typing import Deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_COUNT = 6
SOURCE_TOPIC = "/joint_states"
COMMAND_TOPIC = "/drive_command"
SOURCE_HISTORY_LENGTH = 3000
COMMAND_HISTORY_LENGTH = 25000
DISPLAY_WINDOW_SEC = 20.0
REFRESH_INTERVAL_MS = 100


class JointStreamVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("joint_stream_visualizer")
        self._lock = threading.Lock()
        self._start_time = time.monotonic()
        self._positions: dict[str, list[Deque[tuple[float, float]]]] = {
            SOURCE_TOPIC: [
                deque(maxlen=SOURCE_HISTORY_LENGTH) for _ in range(JOINT_COUNT)
            ],
            COMMAND_TOPIC: [
                deque(maxlen=COMMAND_HISTORY_LENGTH) for _ in range(JOINT_COUNT)
            ],
        }
        self._periods: dict[str, Deque[tuple[float, float]]] = {
            SOURCE_TOPIC: deque(maxlen=SOURCE_HISTORY_LENGTH),
            COMMAND_TOPIC: deque(maxlen=COMMAND_HISTORY_LENGTH),
        }
        self._last_message_time: dict[str, tuple[str, int] | None] = {
            SOURCE_TOPIC: None,
            COMMAND_TOPIC: None,
        }
        self._joint_names = [f"Joint {index + 1}" for index in range(JOINT_COUNT)]

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
        receive_time = time.monotonic()
        elapsed = receive_time - self._start_time
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        if stamp_ns == 0:
            stamp_ns = time.monotonic_ns()
            stamp_source = "receive"
        else:
            stamp_source = "header"

        with self._lock:
            for index, position in enumerate(message.position[:JOINT_COUNT]):
                self._positions[topic][index].append((elapsed, position))

            if topic == SOURCE_TOPIC:
                for index, name in enumerate(message.name[:JOINT_COUNT]):
                    if name:
                        self._joint_names[index] = name

            previous_time = self._last_message_time[topic]
            if (
                previous_time is not None
                and previous_time[0] == stamp_source
                and stamp_ns >= previous_time[1]
            ):
                period_ms = (stamp_ns - previous_time[1]) / 1_000_000
                self._periods[topic].append((elapsed, period_ms))
            self._last_message_time[topic] = (stamp_source, stamp_ns)

    def snapshot(self) -> tuple[
        dict[str, list[list[tuple[float, float]]]],
        dict[str, list[tuple[float, float]]],
        list[str],
    ]:
        with self._lock:
            positions = {
                topic: [list(samples) for samples in joint_samples]
                for topic, joint_samples in self._positions.items()
            }
            periods = {
                topic: list(samples) for topic, samples in self._periods.items()
            }
            names = self._joint_names.copy()
        return positions, periods, names


def main() -> None:
    rclpy.init()
    node = JointStreamVisualizer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    joint_figure, joint_axes = plt.subplots(4, 2, figsize=(14, 12), sharex=True)
    joint_axes_flat = joint_axes.flatten()
    joint_lines: list[tuple[object, object]] = []
    for index, axis in enumerate(joint_axes_flat[:JOINT_COUNT]):
        source_line, = axis.plot([], [], label="URDF / Tester")
        command_line, = axis.plot([], [], label="Drive command")
        joint_lines.append((source_line, command_line))
        axis.set_title(f"Joint {index + 1}")
        axis.set_ylabel("Position (rad)")
        axis.grid(True)
        axis.legend(loc="upper left")
    for axis in joint_axes_flat[JOINT_COUNT:]:
        axis.set_visible(False)
    joint_axes_flat[4].set_xlabel("Elapsed time (s)")
    joint_axes_flat[5].set_xlabel("Elapsed time (s)")
    joint_figure.suptitle("Source Joint Positions vs. Drive Commands")
    joint_figure.tight_layout()

    period_figure, period_axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    source_period_line, = period_axes[0].plot([], [], label="Source /joint_states")
    command_period_line, = period_axes[1].plot([], [], label="Drive /drive_command")
    period_axes[0].set_title("Source message period")
    period_axes[1].set_title("Drive command message period")
    for axis in period_axes:
        axis.set_ylabel("Period (ms)")
        axis.grid(True)
        axis.legend(loc="upper left")
    period_axes[1].set_xlabel("Elapsed time (s)")
    period_figure.suptitle("Inter-message Periods (from JointState header stamps)")
    period_figure.tight_layout()

    def update_joint_positions(_frame: int) -> list[object]:
        positions, periods, names = node.snapshot()
        for index, (source_line, command_line) in enumerate(joint_lines):
            source_samples = positions[SOURCE_TOPIC][index]
            command_samples = positions[COMMAND_TOPIC][index]
            source_line.set_data(
                [sample[0] for sample in source_samples],
                [sample[1] for sample in source_samples],
            )
            command_line.set_data(
                [sample[0] for sample in command_samples],
                [sample[1] for sample in command_samples],
            )
            joint_axes_flat[index].set_title(names[index])
            joint_axes_flat[index].set_ylim(-0.1, 1.5 * 3.141592653589793 + 0.1)

        elapsed = node.elapsed_time()
        for axis in joint_axes_flat[:JOINT_COUNT]:
            axis.set_xlim(
                max(0.0, elapsed - DISPLAY_WINDOW_SEC),
                max(DISPLAY_WINDOW_SEC, elapsed),
            )
        return [line for pair in joint_lines for line in pair]

    def update_periods(_frame: int) -> list[object]:
        _, periods, _ = node.snapshot()
        source_period_line.set_data(
            [sample[0] for sample in periods[SOURCE_TOPIC]],
            [sample[1] for sample in periods[SOURCE_TOPIC]],
        )
        command_period_line.set_data(
            [sample[0] for sample in periods[COMMAND_TOPIC]],
            [sample[1] for sample in periods[COMMAND_TOPIC]],
        )
        for axis in period_axes:
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
            elapsed = node.elapsed_time()
            axis.set_xlim(
                max(0.0, elapsed - DISPLAY_WINDOW_SEC),
                max(DISPLAY_WINDOW_SEC, elapsed),
            )
        return [source_period_line, command_period_line]

    animation = FuncAnimation(
        joint_figure,
        update_joint_positions,
        interval=REFRESH_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )
    period_animation = FuncAnimation(
        period_figure,
        update_periods,
        interval=REFRESH_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        del animation, period_animation
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
