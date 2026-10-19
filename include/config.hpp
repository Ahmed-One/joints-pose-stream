/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#ifndef JOINTS_STREAM_CONFIG_HPP
#define JOINTS_STREAM_CONFIG_HPP

#define NUM_JOINTS 6
#define PERIOD_READER 1 // ms
#define JOINT_ERROR_MARGIN 0.01 // multiplier

#define READER_PERIOD_MS 1
#define WRITER_PERIOD_MS 10
#define MAX_CLIENT_DELAY_FACTOR 0.3

#define MAX_DELTA_RAD_S 2 // rad/s
#define MAX_DELTA_RAD_S2 10 // rad/s2
#define MAX_DELTA_RAD_MS (MAX_DELTA_RAD_S / 1000.0) // rad/ms
#define MAX_DELTA_RAD_MS2 (MAX_DELTA_RAD_S2 / 1000000.0) // rad/ms2

#define ROS_QOS 10
#define ROS_JOINT_POSE_TOPIC "/joint_states"
#define ROS_SUBSCRIBER_NODE_NAME "joint_reader"

// client topic

// Calculate Apropriate buffer size
#define MAX_JOINT_TRAVEL_RAD       (2.0 * M_PI)
#define TRAJECTORY_BUFFER_SIZE_MARGIN 0.5
#define TRAJECTORY_BUFFER_SIZE_FACTOR 3

#define TRAJECTORY_ACCEL_TIME_S \
    (MAX_DELTA_RAD_S / MAX_DELTA_RAD_S2)

#define TRAJECTORY_ACCEL_DISTANCE_RAD \
    (0.5 * MAX_DELTA_RAD_S2 * \
     TRAJECTORY_ACCEL_TIME_S * TRAJECTORY_ACCEL_TIME_S)

#define TRAJECTORY_DURATION_S \
    (2.0 * TRAJECTORY_ACCEL_TIME_S + \
     (MAX_JOINT_TRAVEL_RAD - \
      2.0 * TRAJECTORY_ACCEL_DISTANCE_RAD) / \
     MAX_DELTA_RAD_S)

#define TRAJECTORY_BUFFER_SIZE \
    (TRAJECTORY_DURATION_S * (1000.0 / READER_PERIOD_MS)) + 2

#define BUFF_SIZE \
    (int)(TRAJECTORY_BUFFER_SIZE_FACTOR * (TRAJECTORY_BUFFER_SIZE * (1 + TRAJECTORY_BUFFER_SIZE_MARGIN)))

/*
Calculate the required trajectory buffer size.

1. Calculate the time required to reach maximum angular velocity:
t_accel = max_velocity / max_acceleration

2. Calculate the angular distance covered during acceleration:
d_accel = 0.5 * max_acceleration * t_accel^2

3. Calculate the total trajectory duration, assuming the joint starts and ends at rest,
accelerates to maximum velocity, travels at constant velocity, and then decelerates:
t_total = 2 * t_accel + (total_travel - 2 * d_accel) / max_velocity

4. Calculate the number of trajectory samples required:
samples = t_total * (1000 / reader_period_ms) + 2
where (1000 / reader_period_ms) is the sampling frequency in Hz.
The additional 2 samples provide endpoint/buffer allowance.

5. Apply a configurable safety factor and margin to accommodate additional samples and provide extra buffer capacity:
buffer_size = ceil(samples * factor * (1 + margin))
The final result is rounded up and converted to size_t to ensure sufficient buffer capacity and avoid
floating-point-to-integer conversion errors at the point of use.

Assumptions:

Angular travel is 2*pi radians.
Maximum angular velocity and acceleration are respected.
The trajectory follows a trapezoidal velocity profile.
The calculation estimates the minimum-duration trajectory;
slower trajectories may require a larger buffer.
*/

#endif //JOINTS_STREAM_CONFIG_HPP
