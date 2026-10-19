/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */


#include "drive.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace
{
std::vector<std::string> default_joint_names()
{
    std::vector<std::string> names;
    names.reserve(NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        names.push_back("joint_" + std::to_string(i + 1));
    }
    return names;
}
}

Drive::Drive(rclcpp::Node& node)
    : node_(node),
      command_publisher_(
          node.create_publisher<sensor_msgs::msg::JointState>("/drive_command", ROS_QOS))
{
    const auto names = node_.declare_parameter<std::vector<std::string>>(
        "joint_names", default_joint_names());
    if (names.size() != NUM_JOINTS) {
        throw std::invalid_argument(
            "The joint_names parameter must contain exactly " +
            std::to_string(NUM_JOINTS) + " names");
    }
    std::copy(names.begin(), names.end(), joint_names_.begin());
}

void Drive::write_command(const JointsPose& pose) {
    command_ = pose;
    feedback_ = command_; // TODO: implement real-time random noise

    sensor_msgs::msg::JointState message;
    message.header.stamp = node_.now();
    message.name.assign(joint_names_.begin(), joint_names_.end());
    message.position.assign(std::begin(pose.joint), std::end(pose.joint));
    command_publisher_->publish(message);
}

JointsPose Drive::read_feedback() const noexcept {
    return feedback_;
}
