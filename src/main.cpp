/*
 *  neura/Pose-stream package
 *  Author: Ahmed Abdelmawla
 *  email:  ahmed.ha.abdelmawla@gmail.com
 *  Date:   17.10.2026
 */


#include "config.hpp"
#include "drive.hpp"
#include "joint_state_subscriber.hpp"
#include "reader.hpp"
#include "ring_buffer.hpp"
#include "writer.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Shared application objects.
    RingBuffer<BUFF_SIZE> buffer;
    Writer writer{buffer};

    // ROS subscriber must be managed by a shared_ptr.
    auto subscriber = std::make_shared<JointsPoseSubscriber>(
    [&writer](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        writer.msg_read_callback(msg);
    });
    Drive drive{*subscriber};
    Reader reader{buffer, drive};

    // Create the executor and register the subscriber.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(subscriber);

    // Writer thread: 10 ms period (100 Hz).
    std::thread writer_thread([&]() {
        auto next = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            next += 10ms;
            writer.write();
            std::this_thread::sleep_until(next);
        }
    });

    // Reader thread: 1 ms period (1000 Hz).
    std::thread reader_thread([&]() {
        auto next = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            next += 1ms;
            reader.read();
            std::this_thread::sleep_until(next);
        }
    });

    // ROS callbacks execute here, independently of the two threads.
    executor.spin();

    // executor.spin() returns after ROS shutdown.
    if (writer_thread.joinable())
        writer_thread.join();

    if (reader_thread.joinable())
        reader_thread.join();

    rclcpp::shutdown();
    return 0;
}
