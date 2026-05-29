#include "hello_world_node_cpp/my_node.h"


CMyNode::CMyNode() : Node("my_node")
{
    RCLCPP_INFO(this->get_logger(), "node_name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(), "node_namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "node_fully_qualified_name: %s", this->get_fully_qualified_name());
}

CMyNode::~CMyNode()
{
}
