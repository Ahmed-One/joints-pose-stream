#ifndef JOINTS_STREAM_RING_BUFFER_HPP
#define JOINTS_STREAM_RING_BUFFER_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>

#include "config.hpp"
#include "joints_pose.hpp"

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

public:
    // Call from the writer thread only.
    // Write a single pose
    bool try_write_pose(const JointsPose& pose) noexcept {
        Slot& slot = buffer_[head_];

        // Slot must be free before writing.
        if (slot.available.load(std::memory_order_acquire))
            return false;

        slot.pose = pose;

        // Publish the complete six-joint pose.
        slot.available.store(true, std::memory_order_release);

        head_ = (head_ + 1) % N;
        return true;
    }

    // Call from the writer thread only.
    // Write a stream of poses
    bool try_write_pose_stream(const std::array<JointsPose, BUFF_SIZE>& stream) noexcept {
        return std::all_of(stream.begin(), stream.end(),
            [this](const auto& pose){
                return try_write_pose(pose);
            });
    }

    // Call from the reader thread only.
    // If poses are unavailable, re-reads last pose
    bool try_read(JointsPose& output) noexcept {
        Slot& slot = buffer_[tail_];

        if (slot.available.load(std::memory_order_acquire)) {
            output = slot.pose;

            // Release the slot only after copying is complete.
            slot.available.store(false, std::memory_order_release);

            tail_ = (tail_ + 1) % N;
            return true;
        }
        // No pose has ever been published and read.
        return false;
    }
};

#endif //JOINTS_STREAM_RING_BUFFER_HPP
