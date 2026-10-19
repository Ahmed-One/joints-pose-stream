# joints_stream

`joints_stream` is a ROS 2 C++ package that receives six-joint positions on
`/joint_states`, generates a bounded motion trajectory, and publishes drive
commands on `/drive_command`.

## Build

Install ROS 2 with `rclcpp`, `sensor_msgs`, `rclpy`, and Python Matplotlib, plus
`colcon`. From the workspace root:

```bash
cd ~/ros-ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select joints_stream
source install/setup.bash
```

## Run a demo

Open three terminals. In each, run `cd ~/ros-ws`, then source ROS and the
workspace overlay:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

Start the package node in terminal 1:

```bash
ros2 run joints_stream joints_stream_node
```

Start a visualizer in terminal 2, then a tester in terminal 3:

```bash
ros2 run joints_stream visualizer.py
ros2 run joints_stream tester.py --duration-sec 60
```

Alternatively, use `tester2.py` for a stepped trajectory and `visualizer2.py`
for pose, velocity, and acceleration plots of one joint:

```bash
ros2 run joints_stream visualizer2.py --joint-index 1
ros2 run joints_stream tester2.py --duration-sec 10 --points 3
```

Both testers publish six joints to `/joint_states`; the names default to
`joint_1` through `joint_6`. `tester2.py` holds each waypoint for the allotted
interval and adds timing jitter and occasional pauses. Stop the package node
with Ctrl+C to print the recorded log events.

Joint count, loop periods, velocity and acceleration limits, and buffer sizing
are configurable in [`include/config.hpp`](./include/config.hpp).

## How it works

The writer places joint poses in a single-producer/single-consumer ring buffer.
The producer advances the **head** and the consumer advances the **tail**;
atomic per-slot availability flags coordinate access without a mutex. The
trajectory generator uses a trapezoidal velocity profile, respecting configured
velocity and acceleration limits. The reader consumes poses and the simulated
drive publishes them as `/drive_command`. The logger records buffer full/empty
events and prints them when the node shuts down.
