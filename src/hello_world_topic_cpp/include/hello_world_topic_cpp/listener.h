#ifndef __HELLO_WORLD_TOPIC_LISTENER_H__
#define __HELLO_WORLD_TOPIC_LISTENER_H__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CListener : public rclcpp::Node
{
public:
    CListener();
    ~CListener();

private:
    void topic_callback(const std_msgs::msg::String & msg) const;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscriber;
};

#endif // __HELLO_WORLD_TOPIC_LISTENER_H__