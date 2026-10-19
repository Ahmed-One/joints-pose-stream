#ifndef JOINTS_STREAM_JOINT_STATE_SUBSCRIBER_HPP
#define JOINTS_STREAM_JOINT_STATE_SUBSCRIBER_HPP

#include <functional>
#include <utility>

#include "config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using JointState = sensor_msgs::msg::JointState;
using JointStateCallback = std::function<void(JointState::ConstSharedPtr)>;

class JointsPoseSubscriber : public rclcpp::Node {
public:
    explicit JointsPoseSubscriber(JointStateCallback callback)
        : Node(ROS_SUBSCRIBER_NODE_NAME),
          writer_callback_(std::move(callback))
    {
        subscription_ = create_subscription<JointState>(
            ROS_JOINT_POSE_TOPIC, ROS_QOS,
            [this](JointState::ConstSharedPtr msg) {
                if (msg->position.size() < NUM_JOINTS) {
                    RCLCPP_WARN(
                        get_logger(), "Expected %d joint positions, received %zu",
                        NUM_JOINTS, msg->position.size());
                    return;
                }
                writer_callback_(msg);
            });
    }

private:
    rclcpp::Subscription<JointState>::SharedPtr subscription_;
    JointStateCallback writer_callback_;
};

#endif //JOINTS_STREAM_JOINT_STATE_SUBSCRIBER_HPP
