#ifndef JOINTS_STREAM_TRAJECTORY_HPP
#define JOINTS_STREAM_TRAJECTORY_HPP

#include <functional>

#include "joints_pose.hpp"

bool generate_trajectory(const JointsPose& start, const JointsPose& target,
    std::function<bool(const JointsPose&)> buffer_write_callback);

#endif //JOINTS_STREAM_TRAJECTORY_HPP
