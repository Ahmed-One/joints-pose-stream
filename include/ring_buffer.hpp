#ifndef JOINTS_STREAM_RING_BUFFER_HPP
#define JOINTS_STREAM_RING_BUFFER_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>

#include "config.hpp"
#include "joints_pose.hpp"
#include "logger.hpp"

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
    bool writer_blocked_ = false;
    bool reader_empty_ = false;

public:
    // Call from the writer thread only.
    // Write a single pose
    bool try_write_pose(const JointsPose& pose) {
        Slot& slot = buffer_[head_];

        // Slot must be free before writing.
        if (slot.available.load(std::memory_order_acquire)) {
            if (!writer_blocked_) {
                LOG::Warn() << "RingBuffer full; writer paused at slot "
                            << std::to_string(head_);
                writer_blocked_ = true;
            }
            return false;
        }

        if (writer_blocked_) {
            LOG::Info() << "RingBuffer has space again; writer resumed at slot "
                        << std::to_string(head_);
            writer_blocked_ = false;
        }

        slot.pose = pose;

        // Publish the complete six-joint pose.
        slot.available.store(true, std::memory_order_release);

        head_ = (head_ + 1) % N;
        return true;
    }

    // Call from the writer thread only.
    // Write a stream of poses
    bool try_write_pose_stream(const std::array<JointsPose, BUFF_SIZE>& stream) {
        return std::all_of(stream.begin(), stream.end(),
            [this](const auto& pose){
                return try_write_pose(pose);
            });
    }

    // Call from the reader thread only.
    // If poses are unavailable, re-reads last pose
    bool try_read(JointsPose& output) {
        Slot& slot = buffer_[tail_];

        if (slot.available.load(std::memory_order_acquire)) {
            if (reader_empty_) {
                // LOG::Info() << "RingBuffer has data again; reader resumed at slot "
                //             << std::to_string(tail_);
                reader_empty_ = false;
            }
            output = slot.pose;

            // Release the slot only after copying is complete.
            slot.available.store(false, std::memory_order_release);

            tail_ = (tail_ + 1) % N;
            return true;
        }
        // No pose has ever been published and read.
        if (!reader_empty_) {
            LOG::Warn() << "RingBuffer empty; reader waiting at slot "
                        << std::to_string(tail_);
            reader_empty_ = true;
        }
        return false;
    }
};

#endif //JOINTS_STREAM_RING_BUFFER_HPP
