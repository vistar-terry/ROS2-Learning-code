#include "hello_world_node_cpp/my_node1.h"


CMyNode1::CMyNode1() : Node("my_node1")
{
    RCLCPP_INFO(this->get_logger(), "node_name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(), "node_namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "node_fully_qualified_name: %s", this->get_fully_qualified_name());

    std::vector<std::string> node_names = this->get_node_names();
    for (auto name : node_names)
    {
        RCLCPP_INFO(this->get_logger(), "node: %s", name.c_str());
    }
    
}

CMyNode1::~CMyNode1()
{
}
