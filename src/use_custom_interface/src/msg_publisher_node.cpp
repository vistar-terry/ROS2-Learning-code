#include "use_custom_interface/msg_publisher_node.h"

using namespace std::chrono_literals;

CMsgPubNode::CMsgPubNode() : Node("msg_publisher_node")
{
    m_robotStatusPuber = this->create_publisher<hello_world_interface::msg::RobotStatus>("/useInterface/msg/robotStatus", 10);
    m_timer = this->create_wall_timer(500ms, std::bind(&CMsgPubNode::timerCallback, this));
}

CMsgPubNode::~CMsgPubNode()
{
}

void CMsgPubNode::timerCallback()
{
    auto robotStatus = hello_world_interface::msg::RobotStatus();
    robotStatus.robot_name = "hello interface";
    robotStatus.battery_level = 80;
    robotStatus.is_connected = true;
    robotStatus.temperature = 25;
    robotStatus.current_pose.x = 1.2;
    robotStatus.current_pose.y = 2.6;
    robotStatus.current_pose.theta = 90.0;
    
    this->m_robotStatusPuber->publish(robotStatus);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    auto serverNode = std::make_shared<CMsgPubNode>();
    rclcpp::spin(serverNode);

    rclcpp::shutdown();
    return 0;
}