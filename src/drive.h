/*
 *  neura/Pose-stream package/drive
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */

#ifndef DRIVE_H
#define DRIVE_H

#include "common.h"

class Drive {
public:
    Drive() = default;

    // Receive a command from the reader.
    void write_command(const JointsPose& pose) noexcept;

    // Read the simulated device feedback.
    JointsPose read_feedback() const noexcept;

private:
    JointsPose command_{};
    JointsPose feedback_{};
};


#endif //DRIVE_H
