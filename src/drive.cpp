/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */


#include "drive.h"

void Drive::write_command(const JointsPose& pose) noexcept {
    command_ = pose;
    feedback_ = command_; // TODO: implement real-time random noise
}

JointsPose Drive::read_feedback() const noexcept {
    return feedback_;
}
