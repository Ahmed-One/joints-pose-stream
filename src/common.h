/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#ifndef COMMON_H
#define COMMON_H

#include "pose-stream-config.h"
#include <chrono>
#include <cmath>

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
};

#endif //COMMON_H

