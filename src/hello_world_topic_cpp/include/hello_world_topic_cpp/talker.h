#ifndef __HELLO_WORLD_TOPIC_TALKER_H__
#define __HELLO_WORLD_TOPIC_TALKER_H__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CTalker : public rclcpp::Node
{
public:
    CTalker();
    ~CTalker();

private:
    void timercallback();

private:
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    size_t m_count;
};

#endif // __HELLO_WORLD_TOPIC_TALKER_H__