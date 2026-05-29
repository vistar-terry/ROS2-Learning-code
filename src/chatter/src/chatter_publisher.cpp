#include "hello_world_cpp/chatter_publisher.hpp"
#include <rclcpp/logging.hpp>

namespace hello_world_cpp
{

ChatterPublisher::ChatterPublisher(rclcpp::Node::SharedPtr node, const std::string &topic_name)
    : node_(node), message_count_(0)
{
    publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);
}
 
void ChatterPublisher::start_publishing(std::chrono::seconds interval)
{
    timer_ = node_->create_wall_timer(
        interval, std::bind(&ChatterPublisher::timer_callback, this));
}
 
void ChatterPublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2! Count: " + std::to_string(message_count_++);
    RCLCPP_INFO(node_->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

}