#ifndef __HELLO_WORLD_INTERFACE_MSG_PUB_NODE_H__
#define __HELLO_WORLD_INTERFACE_MSG_PUB_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include "hello_world_interface/msg/robot_status.hpp"

class CMsgPubNode : public rclcpp::Node
{

public:
    CMsgPubNode();
    ~CMsgPubNode();

private:
    void timerCallback();

private:
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<hello_world_interface::msg::RobotStatus>::SharedPtr m_robotStatusPuber;

};


#endif // __HELLO_WORLD_INTERFACE_MSG_PUB_NODE_H__