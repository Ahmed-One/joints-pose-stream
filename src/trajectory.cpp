#include "trajectory.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

bool generate_trajectory(const JointsPose& start, const JointsPose& target,
    std::function<bool(const JointsPose&)> buffer_write_callback)
{
    using namespace std::chrono;

    // FIX: Define explicit constants for your loop step sizing
    constexpr long long step_ms = READER_PERIOD_MS;
    // constexpr double dt = step_ms / 1000.0;
    constexpr double vmax = MAX_DELTA_RAD_S;
    constexpr double amax = MAX_DELTA_RAD_S2;

    struct Profile {
        double distance = 0.0;
        double direction = 1.0;
        double accel_time = 0.0;
        double cruise_time = 0.0;
        double peak_velocity = 0.0;
        double duration = 0.0;
    };

    std::array<Profile, NUM_JOINTS> profiles{};
    double min_duration = 0.0;

    // 1. Calculate the minimum-time profile for every joint.
    for (int i = 0; i < NUM_JOINTS; ++i) {
        const double delta = target.joint[i] - start.joint[i];
        const double distance = std::abs(delta);

        Profile& p = profiles[i];
        p.distance = distance;
        p.direction = delta < 0.0 ? -1.0 : 1.0;

        if (distance == 0.0)
            continue;

        const double threshold = vmax * vmax / amax;

        if (distance <= threshold) {
            p.peak_velocity = std::sqrt(distance * amax);
            p.accel_time = p.peak_velocity / amax;
            p.cruise_time = 0.0;
        } else {
            p.peak_velocity = vmax;
            p.accel_time = vmax / amax;
            p.cruise_time = (distance - threshold) / vmax;
        }

        p.duration = 2.0 * p.accel_time + p.cruise_time;
        min_duration = std::max(min_duration, p.duration);
    }

    // 2. Round up to an integer multiple of the active loop cycle rate.
    // FIX: Scaled round-up logic safely handles variable READER_PERIOD_MS sizes.
    const auto total_ms = static_cast<long long>(
        std::ceil(min_duration * 1000.0 / step_ms)) * step_ms;

    const double total_duration = total_ms / 1000.0;

    // 3. Sample the synchronized trajectory.
    // FIX: The index matches elapsed ms, incrementing strictly by step_ms.
    for (long long ms = 0; ms < total_ms; ms += step_ms) {
        const double t = ms / 1000.0; // True time index in seconds

        JointsPose pose{};
        pose.timestamp = start.timestamp + milliseconds(ms);

        for (int i = 0; i < NUM_JOINTS; ++i) {
            const Profile& p = profiles[i];

            if (p.distance == 0.0) {
                pose.joint[i] = start.joint[i];
                continue;
            }

            // OPTIMIZATION: Instead of dividing inside the inner loop step,
            // check if the scale factor is effectively 1.0 to skip scaling.
            const double local_t = (total_duration == p.duration) ? t : (t * p.duration / total_duration);
            const double ta = p.accel_time;
            const double tc = p.cruise_time;
            const double vp = p.peak_velocity;

            double x = 0.0;

            if (local_t < ta) {
                x = 0.5 * amax * local_t * local_t;
            } else if (local_t < ta + tc) {
                x = 0.5 * amax * ta * ta
                    + vp * (local_t - ta);
            } else if (local_t < p.duration) {
                const double td = local_t - ta - tc;
                const double cruise_distance = vp * tc;

                x = 0.5 * amax * ta * ta
                    + cruise_distance
                    + vp * td
                    - 0.5 * amax * td * td;
            } else {
                x = p.distance;
            }

            pose.joint[i] = start.joint[i]
                + p.direction * std::clamp(x, 0.0, p.distance);
        }

        // write to the buffer
        if(!buffer_write_callback(pose)){
            return false;
        }
    }

    // 4. Append the exact final pose.
    JointsPose final_pose = target;
    final_pose.timestamp = start.timestamp + milliseconds(total_ms);
    if(!buffer_write_callback(final_pose)){
            return false;
    }
    return true;
}
