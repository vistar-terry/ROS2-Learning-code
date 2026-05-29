// hello_world_cpp/include/hello_world_cpp/chatter_publisher.hpp

#ifndef HELLO_WORLD_CPP_CHATTER_PUBLISHER_HPP
#define HELLO_WORLD_CPP_CHATTER_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <functional>

namespace hello_world_cpp
{
class ChatterPublisher
{
public:
    ChatterPublisher(rclcpp::Node::SharedPtr node, const std::string &topic_name);
    void start_publishing(std::chrono::seconds interval);
 
private:
    void timer_callback();
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t message_count_;
};
}

#endif // HELLO_WORLD_CPP_CHATTER_PUBLISHER_HPP