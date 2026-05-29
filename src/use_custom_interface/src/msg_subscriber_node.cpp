#include "use_custom_interface/msg_subscriber_node.h"


CMsgSubNode::CMsgSubNode() : Node("msg_subscriber_node")
{
    RCLCPP_INFO(this->get_logger(), "node_name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(), "node_namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "node_fully_qualified_name: %s", this->get_fully_qualified_name());

    m_robotStatusSuber = this->create_subscription<hello_world_interface::msg::RobotStatus>("/useInterface/msg/robotStatus", 10, 
        std::bind(&CMsgSubNode::robotStatusCallback, this, std::placeholders::_1));
}

CMsgSubNode::~CMsgSubNode()
{
}

void CMsgSubNode::robotStatusCallback(const hello_world_interface::msg::RobotStatus &robotStatus)
{
    RCLCPP_INFO(this->get_logger(), "robot_name: %s, battery_level: %d%%, is_connected: %s, temperature: %.1f°C, current_pose: (%.3lf, %.3lf, %.1lf)", 
        robotStatus.robot_name.c_str(), robotStatus.battery_level, robotStatus.is_connected?"true":"false", robotStatus.temperature, 
        robotStatus.current_pose.x, robotStatus.current_pose.y, robotStatus.current_pose.theta); 
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    auto clientNode = std::make_shared<CMsgSubNode>();
    rclcpp::spin(clientNode);

    rclcpp::shutdown();
    return 0;
}