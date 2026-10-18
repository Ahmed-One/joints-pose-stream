/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#ifndef POSE_STREAM_HPP
#define POSE_STREAM_HPP

#include <array>
#include <atomic>
#include <cstddef>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "common.h"
#include "drive.h"
#include "pose-stream-config.h"

// Ring buffer for lock free data exchange
template <std::size_t N>
class RingBuffer {
    static_assert(N > 0);

    struct Slot {
        JointsPose pose{};
        std::atomic<bool> available{false};
    };

    std::array<Slot, N> buffer_{};
    static_assert(std::atomic<bool>::is_always_lock_free,
        "std::atomic<int> is not always lock-free on this platform!");

    // Each index is accessed by only its owning thread.
    std::size_t head_ = 0; // Writer's next slot
    std::size_t tail_ = 0; // Reader's next slot

    // Reader-thread-only fallback state.
    // JointsPose last_{};
    // bool has_last_ = false;

public:
    // Call from the writer thread only.
    // Write a single pose
    bool try_write_pose(const JointsPose& pose) noexcept {
        Slot& slot = buffer_[head_];

        // Slot must be free before writing.
        if (slot.available.load(std::memory_order_acquire))
            return false;

        slot.pose = pose;

        // Publish the complete six-joint pose.
        slot.available.store(true, std::memory_order_release);

        head_ = (head_ + 1) % N;
        return true;
    }

    // Call from the writer thread only.
    // Write a stream of poses
    bool try_write_pose_stream(const std::array<JointsPose, BUFF_SIZE>& stream) noexcept {
        return std::all_of(stream.begin(), stream.end(),
            [this](const auto& pose){
                return try_write_pose(pose);
            });
    }

    // Call from the reader thread only.
    // If poses are unavailable, re-reads last pose
    bool try_read(JointsPose& output) noexcept {
        Slot& slot = buffer_[tail_];

        if (slot.available.load(std::memory_order_acquire)) {
            output = slot.pose;

            // Retain a private copy for underrun handling.
            // last_ = output;
            // has_last_ = true;

            // Release the slot only after copying is complete.
            slot.available.store(false, std::memory_order_release);

            tail_ = (tail_ + 1) % N;
            return true;
        }

        // // Next pose is not ready: hold the last pose.
        // if (has_last_) {
        //     output = last_;
        //     return true;
        // }

        // No pose has ever been published and read.
        return false;
    }
};

// ===================================================================
// ===================================================================
// ===================================================================

class JointsPoseSubscriber : public rclcpp::Node {
public:
    JointsPoseSubscriber(std::function<void()>& reader_callback) : reader_callback_(reader_callback),
    Node(ROS_SUBSCRIBER_NODE_NAME){
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                ROS_JOINT_POSE_TOPIC, ROS_QOS, reader_callback);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::function<void()> reader_callback_
};

// ===================================================================
// ===================================================================
// ===================================================================

// Reader class for status and implementation
class Reader{
private:    
    enum State{
        HOLD,
        STREAMING,
    };
    Drive drive_;
    RingBuffer<BUFF_SIZE>& buffer_;
    JointsPose to_pose_{};  // pose variable to save read pose
    JointsPose last_pose_{};

    bool last_pose_captured_ = false;

public:
    std::atomic<State> state{State::HOLD};
    static_assert(std::atomic<State>::is_always_lock_free,
        "std::atomic<State> is not always lock-free on this platform!");
   
    Reader(RingBuffer<BUFF_SIZE>& buffer, Drive& drive): buffer_(buffer), drive_(drive) {}
    
    // TODO: LOG every transaction
    void read() noexcept {
        // try to read the buffer
        if(buffer_.try_read(to_pose_)){
            self->state.store(State::STREAMING, std::memory_order_release);
            drive_.write_command(to_pose_);
            last_pose_ = to_pose_;
            last_pose_captured_ = true;
        } else {
            // reading failed
            this->state.store(State::HOLD, std::memory_order_release);
            if(!last_pose_captured_){
                // no pose has ever been published and read
                // capture actual pose to hold to prevent drift due to sensor jitter
                last_pose_ = drive_.read_feedback();
                last_pose_captured_ = true;
            }
            drive_.write_command(last_pose_);
        }
    }
};

// ===================================================================
// ===================================================================
// ===================================================================

class Writer{
private:
    enum State{
        IDLE,
        STREAMING
    };

    std::atomic<State> state{State::IDLE};
    static_assert(std::atomic<State>::is_always_lock_free,
        "std::atomic<State> is not always lock-free on this platform!");
    RingBuffer<BUFF_SIZE>& buffer_;
    Drive& drive_;
    JointsPose last_setpoint_{};  // pose variable to save last setpoint
    bool new_setpoint_available_ = false;  // flag to indicate if a new setpoint is available
    JointsPose current_setpoint_{};  // pose variable to save current setpoint
    JointsPoseSubscriber subscriber_{std::bind(&Writer::msg_read_callback, this)};  // subscriber to read from ROS topic

public:
    Writer(RingBuffer<BUFF_SIZE>& buffer, Drive& drive): buffer_(buffer), drive_(drive) {}

    void write() noexcept {
        if (new_setpoint_available_) {
            new_setpoint_available_ = false;
            // last_setpoint_ = current_setpoint_;
            // create new trajectory of poses for the reader
        }

        switch (state.load(std::memory_order_acquire)) {
            case State::IDLE:
                break;
            case State::STREAMING:
                break;
        }
    }

    // ubscriber callback member function to read from ROS topic
    void msg_read_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // read the ros topic and update latest setpoint
        for (int i = 0; i < NUM_JOINTS; ++i) {
            current_setpoint_.joint[i] = msg->position[i];
        }
        current_setpoint_.timestamp = std::chrono::steady_clock::now();
        new_setpoint_available_ = true;
    }
};




#endif //POSE_STREAM_HPP