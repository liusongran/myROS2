#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "/home/srliu/myProject/mySpace/src/profile_liu/myProfile.hpp"

using namespace std::chrono_literals;

//#define RCLCPP_INFO_LIU               RCLCPP_INFO
#define RCLCPP_INFO_LIU(logger, ...)    {}  
/*
 * A small convenience function for converting a thread ID to a string
 */
std::string string_thread_id() {
    auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
    return std::to_string(hashed);
}


// MARK: node
struct Consumer : public rclcpp::Node
{
public:
    Consumer(const std::string & name)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))
    {
        callback_group_subscriber1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        sub_ = this->create_subscription<std_msgs::msg::String>(
        "srliu_test",
        rclcpp::QoS(10),
        std::bind(
            &Consumer::subscriber_cb,
            this,
            std::placeholders::_1), 
            sub1_opt
        );
    }

private:
    std::string timing_string() {
        rclcpp::Time time = this->now();
        return std::to_string(time.nanoseconds());
    }

    void subscriber_cb(const std_msgs::msg::String::UniquePtr msg) {
    auto message_received_at = timing_string();
    // Extract current thread
    RCLCPP_INFO_LIU(
        this->get_logger(), "THREAD %s => Heard '%s' at %s",
        string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    //rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 80, true);
    
    auto consumer3  = std::make_shared<Consumer>("consumer3");
    /*auto consumer1  = std::make_shared<Consumer>("consumer1");
    auto consumer2  = std::make_shared<Consumer>("consumer2");
    auto consumer3  = std::make_shared<Consumer>("consumer3");
    auto consumer4  = std::make_shared<Consumer>("consumer4");
    auto consumer5  = std::make_shared<Consumer>("consumer5");
    auto consumer6  = std::make_shared<Consumer>("consumer6");
    auto consumer7  = std::make_shared<Consumer>("consumer7");
    auto consumer8  = std::make_shared<Consumer>("consumer8");
    auto consumer9  = std::make_shared<Consumer>("consumer9");*/

    executor.add_node(consumer3);
    /*executor.add_node(consumer1);
    executor.add_node(consumer2);
    executor.add_node(consumer3);
    executor.add_node(consumer4);
    executor.add_node(consumer5);
    executor.add_node(consumer6);
    executor.add_node(consumer7);
    executor.add_node(consumer8);
    executor.add_node(consumer9);*/

    executor.spin();
    rclcpp::shutdown();

    return 0;
}