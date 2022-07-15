#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "/home/srliu/myProject/mySpace/src/profile_liu/myProfile.hpp"

using namespace std::chrono_literals;

/*
 * A small convenience function for converting a thread ID to a string
 */
std::string string_thread_id() {
    auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
    return std::to_string(hashed);
}

/* MARK: Node0
 * For this example, we will be creating a publishing node like the one in minimal_publisher.
 * We will have a single subscriber node running 2 threads. Each thread loops at different speeds, and
 * just repeats what it sees from the publisher to the screen.
 */
class PublisherNode : public rclcpp::Node 
{
public:
    PublisherNode()
    : Node("PublisherNode"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("srliu_test", 10);
    auto timer_callback =
    [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello World! " + std::to_string(this->count_++);
        // Extract current thread
        auto curr_thread = string_thread_id();
        // Prep display message
        RCLCPP_INFO(
            this->get_logger(), "\n<<THREAD %s>> Publishing '%s'",
            curr_thread.c_str(), message.data.c_str());
            this->publisher_->publish(message);
        };
    timer_ = this->create_wall_timer(500ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class DualThreadedNode : public rclcpp::Node
{
public:
    DualThreadedNode()
    : Node("DualThreadedNode")
    {
    /* 
     * These define the callback groups
     * They don't really do much on their own, but they have to exist in order to
     * assign callbacks to them. They're also what the executor looks for when trying to run multiple threads
     */
    callback_group_subscriber1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_subscriber2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_subscriber2_;

    subscription1_ = this->create_subscription<std_msgs::msg::String>(
        "srliu_test",
        rclcpp::QoS(10),
        // std::bind is sort of C++'s way of passing a function
        // If you're used to function-passing, skip these comments
        std::bind(
            &DualThreadedNode::subscriber1_cb,  // First parameter is a reference to the function
            this,                               // What the function should be bound to
            std::placeholders::_1),             // At this point we're not positive of all the parameters being passed
                                                // So we just put a generic placeholder into the binder 

            sub1_opt                            // This is where we set the callback group.
                                                // This subscription will run with callback group subscriber1
        );
    subscription2_ = this->create_subscription<std_msgs::msg::String>(
        "srliu_test",
        rclcpp::QoS(10),
        std::bind(
        &DualThreadedNode::subscriber2_cb,
        this,
        std::placeholders::_1),
        sub2_opt
        );
    }

private:
    /*
     * Simple function for generating a timestamp
     * Used for somewhat ineffectually demonstrating that the multithreading doesn't cripple performace
     */
    std::string timing_string() {
        rclcpp::Time time = this->now();
        return std::to_string(time.nanoseconds());
    }

    /*
     * Every time the Publisher publishes something, all subscribers to the topic get poked
     * This function gets called when Subscriber1 is poked (due to the std::bind we used when defining it)
     */
    void subscriber1_cb(const std_msgs::msg::String::SharedPtr msg) {
    auto message_received_at = timing_string();
    // Extract current thread
    RCLCPP_INFO(
        this->get_logger(), "THREAD %s => Heard '%s' at %s",
        string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
    }

    /*
     * This function gets called when Subscriber2 is poked
     * Since it's running on a separate thread than Subscriber 1, it will run at (more-or-less) the same time!
     */
    void subscriber2_cb(const std_msgs::msg::String::SharedPtr msg) {
    auto message_received_at = timing_string();
    // Prep display message
    RCLCPP_INFO(
        this->get_logger(), "THREAD %s => Heard '%s' at %s",
        string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
};

// MARK: node
struct Consumer : public rclcpp::Node
{
public:
    Consumer(const std::string & name, const std::string & input)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::String>(
        input,
        rclcpp::QoS(10),
        &this->subscriber_cb);
    }

private:
    std::string timing_string() {
        rclcpp::Time time = this->now();
        return std::to_string(time.nanoseconds());
    }

    void subscriber_cb(const std_msgs::msg::String::UniquePtr msg) {
    auto message_received_at = timing_string();
    // Extract current thread
    RCLCPP_INFO(
        this->get_logger(), "THREAD %s => Heard '%s' at %s",
        string_thread_id().c_str(), msg->data.c_str(), message_received_at.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    auto pubnode = std::make_shared<PublisherNode>();
    //auto subnode = std::make_shared<DualThreadedNode>();  // BOTH subscriber callbacks will still run on different threads
    auto consumer0 = std::make_shared<Consumer>("consumer", "srliu_test");
    auto consumer1 = std::make_shared<Consumer>("consumer", "srliu_test");
    auto consumer2 = std::make_shared<Consumer>("consumer", "srliu_test");

    executor.add_node(pubnode);
    executor.add_node(consumer0);
    executor.add_node(consumer1);
    executor.add_node(consumer2);
    //executor.add_node(subnode);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}