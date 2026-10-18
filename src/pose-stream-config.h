/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#ifndef POSE_STREAM_CONFIG_H
#define POSE_STREAM_CONFIG_H

// buffer sizes
#define NUM_JOINTS 6
#define BUFF_SIZE 128 // TODO: generate using a constexpr function
#define PERIOD_READER 1 // ms
#define JOINT_ERROR_MARGIN 0.01 // multiplier

#define ROS_QOS 10
#define ROS_JOINT_POSE_TOPIC "/joint_states"
#define ROS_SUBSCRIBER_NODE_NAME "joint_reader"
// limits for various states
// reader HOLD error margin

// drive topic?Function?Feedback?Script?


// client topic

#endif //POSE_STREAM_CONFIG_H