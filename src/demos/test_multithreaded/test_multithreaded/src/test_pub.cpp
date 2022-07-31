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

struct Producer : public rclcpp::Node
{
    Producer(const std::string & name, const std::string & output)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))
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
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32()); 
        msg->data = count++;

        std::thread::id thread_id = std::this_thread::get_id();
        printf("\npb1_THREAD %d => Publishing '%d' at %ld\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime());
        //printf("\nsb1_THREAD %d => Publishing '%d' at %lld in Process:%d.\n",  (*(uint32_t*)&thread_id), msg->data, get_clocktime(), getpid());

        pub_ptr->publish(std::move(msg));
        };
    timer_ = this->create_wall_timer(0.5s, callback);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


struct Consumer : public rclcpp::Node
{
    Consumer(const std::string & name, const std::string & input)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))
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



int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    FILE *fp = freopen("/home/srliu/myProject/myROS2/data_log/case1plus/doTrace_mul_inter_0.5s.log", "w", stdout);
    if(NULL == fp){
        printf("failed~\r\n");
        return -1;
    }
    LOG_LIU("|main|1. start to init............\r\n");
    rclcpp::init(argc, argv);

    LOG_LIU("|main|2. start to creat executor............\r\n");
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    //rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 20, true);

    LOG_LIU("|main|3. start to creat nodes............\r\n");
    auto producer = std::make_shared<Producer>("producer", "srliu_test");
    auto consumer0 = std::make_shared<Consumer>("consumer0", "srliu_test");
    /*auto consumer1  = std::make_shared<Consumer>("consumer1");
    auto consumer2  = std::make_shared<Consumer>("consumer2");
    auto consumer3  = std::make_shared<Consumer>("consumer3");
    auto consumer4  = std::make_shared<Consumer>("consumer4");
    auto consumer5  = std::make_shared<Consumer>("consumer5");
    auto consumer6  = std::make_shared<Consumer>("consumer6");
    auto consumer7  = std::make_shared<Consumer>("consumer7");
    auto consumer8  = std::make_shared<Consumer>("consumer8");
    auto consumer9  = std::make_shared<Consumer>("consumer9");

    auto consumer10 = std::make_shared<Consumer>("consumer10");
    auto consumer11 = std::make_shared<Consumer>("consumer11");
    auto consumer12 = std::make_shared<Consumer>("consumer12");
    auto consumer13 = std::make_shared<Consumer>("consumer13"); 
    auto consumer14 = std::make_shared<Consumer>("consumer14");
    auto consumer15 = std::make_shared<Consumer>("consumer15");
    auto consumer16 = std::make_shared<Consumer>("consumer16");
    auto consumer17 = std::make_shared<Consumer>("consumer17");
    auto consumer18 = std::make_shared<Consumer>("consumer18");
    auto consumer19 = std::make_shared<Consumer>("consumer19");*/

    /*auto consumer20 = std::make_shared<Consumer>("consumer20");
    auto consumer21 = std::make_shared<Consumer>("consumer21");
    auto consumer22 = std::make_shared<Consumer>("consumer22");
    auto consumer23 = std::make_shared<Consumer>("consumer23"); 
    auto consumer24 = std::make_shared<Consumer>("consumer24");
    auto consumer25 = std::make_shared<Consumer>("consumer25");
    auto consumer26 = std::make_shared<Consumer>("consumer26");
    auto consumer27 = std::make_shared<Consumer>("consumer27");
    auto consumer28 = std::make_shared<Consumer>("consumer28");
    auto consumer29 = std::make_shared<Consumer>("consumer29");

    auto consumer30 = std::make_shared<Consumer>("consumer30");
    auto consumer31 = std::make_shared<Consumer>("consumer31");
    auto consumer32 = std::make_shared<Consumer>("consumer32");
    auto consumer33 = std::make_shared<Consumer>("consumer33"); 
    auto consumer34 = std::make_shared<Consumer>("consumer34");
    auto consumer35 = std::make_shared<Consumer>("consumer35");
    auto consumer36 = std::make_shared<Consumer>("consumer36");
    auto consumer37 = std::make_shared<Consumer>("consumer37");
    auto consumer38 = std::make_shared<Consumer>("consumer38");
    auto consumer39 = std::make_shared<Consumer>("consumer39");

    auto consumer40 = std::make_shared<Consumer>("consumer40");
    auto consumer41 = std::make_shared<Consumer>("consumer41");
    auto consumer42 = std::make_shared<Consumer>("consumer42");
    auto consumer43 = std::make_shared<Consumer>("consumer43"); 
    auto consumer44 = std::make_shared<Consumer>("consumer44");
    auto consumer45 = std::make_shared<Consumer>("consumer45");
    auto consumer46 = std::make_shared<Consumer>("consumer46");
    auto consumer47 = std::make_shared<Consumer>("consumer47");
    auto consumer48 = std::make_shared<Consumer>("consumer48");
    auto consumer49 = std::make_shared<Consumer>("consumer49");*/

    LOG_LIU("|main|4. start to add nodes............\r\n");
    executor.add_node(producer);
    executor.add_node(consumer0);
    /*executor.add_node(consumer1);
    executor.add_node(consumer2);
    executor.add_node(consumer3);
    executor.add_node(consumer4);
    executor.add_node(consumer5);
    executor.add_node(consumer6);
    executor.add_node(consumer7);
    executor.add_node(consumer8);
    executor.add_node(consumer9);

    executor.add_node(consumer10);
    executor.add_node(consumer11);
    executor.add_node(consumer12);
    executor.add_node(consumer13);
    executor.add_node(consumer14);
    executor.add_node(consumer15);
    executor.add_node(consumer16);
    executor.add_node(consumer17);
    executor.add_node(consumer18);
    executor.add_node(consumer19);*/

    /*executor.add_node(consumer20);
    executor.add_node(consumer21);
    executor.add_node(consumer22);
    executor.add_node(consumer23);
    executor.add_node(consumer24);
    executor.add_node(consumer25);
    executor.add_node(consumer26);
    executor.add_node(consumer27);
    executor.add_node(consumer28);
    executor.add_node(consumer29);

    executor.add_node(consumer30);
    executor.add_node(consumer31);
    executor.add_node(consumer32);
    executor.add_node(consumer33);
    executor.add_node(consumer34);
    executor.add_node(consumer35);
    executor.add_node(consumer36);
    executor.add_node(consumer37);
    executor.add_node(consumer38);
    executor.add_node(consumer39);

    executor.add_node(consumer40);
    executor.add_node(consumer41);
    executor.add_node(consumer42);
    executor.add_node(consumer43);
    executor.add_node(consumer44);
    executor.add_node(consumer45);
    executor.add_node(consumer46);
    executor.add_node(consumer47);
    executor.add_node(consumer48);
    executor.add_node(consumer49);*/
    //executor.add_node(subnode);
    LOG_LIU("|main|5. start to spin............\r\n");

    printf("Let's GO!!!\r\n");
    executor.spin();
    rclcpp::shutdown();

    fclose(fp);
    return 0;
}