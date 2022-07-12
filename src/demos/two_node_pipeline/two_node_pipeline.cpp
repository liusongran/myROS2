
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


// TODO: node0
struct Producer : public rclcpp::Node
{
  Producer(const std::string & name, const std::string & output)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) //TODO:
  {
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub]() -> void {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        static int32_t count = 0;
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32()); //TODO-3
        msg->data = count++;

        std::thread::id thread_id = std::this_thread::get_id();
        printf("\npb1_THREAD %d => Publishing '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        //printf("\nsb1_THREAD %d => Publishing '%d' at %lld in Process:%d.\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime(), getpid());

        pub_ptr->publish(std::move(msg));
      };
    timer_ = this->create_wall_timer(5s, callback);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// TODO: node1
struct Consumer : public rclcpp::Node
{
  Consumer(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) //TODO-2
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input,
      10,
      [](std_msgs::msg::Int32::UniquePtr msg) {

        std::thread::id thread_id = std::this_thread::get_id();
        printf("\nsb1_THREAD %d => Heard '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        //printf("\nsb1_THREAD %d => Heard '%d' at %lld in Process:%d.\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime(), getpid());
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};


// TODO: node2
struct Consumer2 : public rclcpp::Node
{
  Consumer2(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input,
      10,
      [](std_msgs::msg::Int32::UniquePtr msg) {

        std::thread::id thread_id = std::this_thread::get_id();
        printf("\nsb5_THREAD %d => Heard '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        //printf("\nsb1_THREAD %d => Heard '%d' at %lld in Process:%d.\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime(), getpid());
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};
/*
// TODO: node3
struct Consumer3 : public rclcpp::Node
{
  Consumer3(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input,
      10,
      [](std_msgs::msg::Int32::UniquePtr msg) {

        std::thread::id thread_id = std::this_thread::get_id();
        printf("\nsb3_THREAD %d => Heard '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        //printf("\nsb1_THREAD %d => Heard '%d' at %lld in Process:%d.\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime(), getpid());
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};
// TODO: node4
struct Consumer4 : public rclcpp::Node
{
  Consumer4(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input,
      10,
      [](std_msgs::msg::Int32::UniquePtr msg) {

        std::thread::id thread_id = std::this_thread::get_id();
        printf("\nsb4_THREAD %d => Heard '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        //printf("\nsb1_THREAD %d => Heard '%d' at %lld in Process:%d.\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime(), getpid());
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};
// TODO: node5
struct Consumer5 : public rclcpp::Node
{
  Consumer5(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input,
      10,
      [](std_msgs::msg::Int32::UniquePtr msg) {

        std::thread::id thread_id = std::this_thread::get_id();
        printf("\nsb5_THREAD %d => Heard '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        //printf("\nsb1_THREAD %d => Heard '%d' at %lld in Process:%d.\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime(), getpid());
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};
*/

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor; //TODO-1
  //rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 10, true);

  auto producer = std::make_shared<Producer>("producer", "number");
  auto consumer = std::make_shared<Consumer>("consumer", "number");
  auto consumer2 = std::make_shared<Consumer2>("consumer2", "number");
  /*
  auto consumer2 = std::make_shared<Consumer2>("consumer2", "number");
  auto consumer3 = std::make_shared<Consumer3>("consumer3", "number");
  auto consumer4 = std::make_shared<Consumer4>("consumer4", "number");
  auto consumer5 = std::make_shared<Consumer5>("consumer5", "number");
  */

  executor.add_node(producer);
  executor.add_node(consumer);
  executor.add_node(consumer2);
  /*
  executor.add_node(consumer2);
  executor.add_node(consumer3);
  executor.add_node(consumer4);
  executor.add_node(consumer5);
  */
  printf("start~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
