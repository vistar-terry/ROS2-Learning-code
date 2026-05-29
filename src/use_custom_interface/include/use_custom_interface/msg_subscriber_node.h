#ifndef __HELLO_WORLD_INTERFACE_MSG_SUB_NODE_H__
#define __HELLO_WORLD_INTERFACE_MSG_SUB_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include "hello_world_interface/msg/robot_status.hpp"

class CMsgSubNode : public rclcpp::Node
{

public:
    CMsgSubNode();
    ~CMsgSubNode();

private:
    void robotStatusCallback(const hello_world_interface::msg::RobotStatus &robotStatus);

private:
    rclcpp::Subscription<hello_world_interface::msg::RobotStatus>::SharedPtr m_robotStatusSuber;
};


#endif // __HELLO_WORLD_INTERFACE_MSG_SUB_NODE_H__