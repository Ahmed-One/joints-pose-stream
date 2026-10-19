#ifndef JOINTS_STREAM_READER_HPP
#define JOINTS_STREAM_READER_HPP

#include "drive.hpp"
#include "ring_buffer.hpp"

// Reader class for status and implementation
class Reader{
private:
    Drive& drive_;
    RingBuffer<BUFF_SIZE>& buffer_;
    JointsPose to_pose_{};  // pose variable to save read pose
    JointsPose last_pose_{};

    bool last_pose_captured_ = false;
    bool to_hold_ = false;

public:
    Reader(RingBuffer<BUFF_SIZE>& buffer, Drive& drive): drive_(drive), buffer_(buffer) {}

    // TODO: LOG every transaction
    void read() noexcept {
        if (to_hold_) {
            // drive stopped by writer, hold last pose
            drive_.write_command(last_pose_);
            return;
        }
        // try to read the buffer
        if(buffer_.try_read(to_pose_)){
            drive_.write_command(to_pose_);
            last_pose_ = to_pose_;
            last_pose_captured_ = true;
        } else {
            // reading failed
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

#endif //JOINTS_STREAM_READER_HPP
