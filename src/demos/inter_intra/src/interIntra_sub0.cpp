
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include "unistd.h"
#include "thread"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "/home/srliu/myProject/mySpace/src/profile_liu/myProfile.hpp"

using namespace std::chrono_literals;

// Node that consumes messages.
struct Consumer_inter : public rclcpp::Node
{
    Consumer_inter(const std::string & name, const std::string & input)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))
    {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
        input,
        10,
        [](std_msgs::msg::Int32::UniquePtr msg) {
        std::thread::id thread_id = std::this_thread::get_id();
        printf("\nsb2_THREAD %d => Heard '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        });
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    auto consumer_inter = std::make_shared<Consumer_inter>("consumer_inter", "number");

    executor.add_node(consumer_inter);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
