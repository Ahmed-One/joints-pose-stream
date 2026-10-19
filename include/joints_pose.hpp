/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#ifndef JOINTS_STREAM_JOINTS_POSE_HPP
#define JOINTS_STREAM_JOINTS_POSE_HPP

#include "config.hpp"
#include <chrono>
#include <cmath>

struct JointsPoseDifference {
    double joint_q[NUM_JOINTS]{};
    std::chrono::steady_clock::duration elapsed{};
};

struct JointsPose {
    double joint[NUM_JOINTS]{};
    std::chrono::steady_clock::time_point timestamp{};

    bool operator==(const JointsPose& other) const noexcept {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (std::abs(joint[i] - other.joint[i]) > JOINT_ERROR_MARGIN) {
                return false;
            }
        }
        return true;
    }

    JointsPoseDifference operator-(const JointsPose& other) const noexcept {
        JointsPoseDifference result{};

        for (int i = 0; i < NUM_JOINTS; ++i) {
            result.joint_q[i] = joint[i] - other.joint[i];
        }

        result.elapsed = timestamp - other.timestamp;
        return result;
    }
};

struct JointVelocity {
    double joint[NUM_JOINTS]{};

    JointVelocity(const JointsPoseDifference& pose_diff) noexcept {
        auto elapsed_s = std::chrono::duration<double>(pose_diff.elapsed).count();
        if (elapsed_s > 0) {
            for (int i = 0; i < NUM_JOINTS; ++i) {
                joint[i] = pose_diff.joint_q[i] / static_cast<double>(elapsed_s);
            }
        }
    }
};

#endif //JOINTS_STREAM_JOINTS_POSE_HPP
