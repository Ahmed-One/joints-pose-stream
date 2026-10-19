/*
 *  neura/Pose-stream package/drive
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#ifndef JOINTS_STREAM_DRIVE_HPP
#define JOINTS_STREAM_DRIVE_HPP

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "config.hpp"
#include "joints_pose.hpp"

class Drive {
public:
    explicit Drive(rclcpp::Node& node);

    // Receive a command from the reader.
    void write_command(const JointsPose& pose);

    // Read the simulated device feedback.
    JointsPose read_feedback() const noexcept;

private:
    rclcpp::Node& node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
    std::array<std::string, NUM_JOINTS> joint_names_;
    JointsPose command_{};
    JointsPose feedback_{};
};

#endif //JOINTS_STREAM_DRIVE_HPP
