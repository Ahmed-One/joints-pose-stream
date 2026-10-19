#ifndef JOINTS_STREAM_WRITER_HPP
#define JOINTS_STREAM_WRITER_HPP

#include <chrono>
#include <functional>

#include <sensor_msgs/msg/joint_state.hpp>

#include "config.hpp"
#include "joints_pose.hpp"
#include "ring_buffer.hpp"
#include "trajectory.hpp"

class Writer{
private:
    RingBuffer<BUFF_SIZE>& buffer_;
    JointsPose last_setpoint_{};  // pose variable to save last setpoint
    JointsPose current_setpoint_{};  // pose variable to save current setpoint
    bool new_setpoint_available_ = false;  // flag to indicate if a new setpoint is available

public:
    Writer(RingBuffer<BUFF_SIZE>& buffer): buffer_(buffer) {}

    void write() noexcept {
        if (new_setpoint_available_) {
            new_setpoint_available_ = false;
            // Generate setpoint from last setpoint to current setpoint
            generate_trajectory(last_setpoint_, current_setpoint_,
                [this](const JointsPose& pose) {
                return buffer_.try_write_pose(pose);});
        }
        // else, do nothing and let reader continue as is
    }

    // ubscriber callback member function to read from ROS topic
    void msg_read_callback(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        // read the ros topic and update latest setpoint
        for (int i = 0; i < NUM_JOINTS; ++i) {
            last_setpoint_ = current_setpoint_;
            current_setpoint_.joint[i] = msg->position[i];
        }
        current_setpoint_.timestamp = std::chrono::steady_clock::now();
        new_setpoint_available_ = true;
    }
};

#endif //JOINTS_STREAM_WRITER_HPP
