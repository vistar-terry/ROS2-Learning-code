#include "hello_world_node_cpp/my_node.h"
#include "hello_world_node_cpp/my_node1.h"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CMyNode>();
    auto node1 = std::make_shared<CMyNode1>();
    // 节点逻辑
    rclcpp::spin(node);
    rclcpp::spin(node1);
    rclcpp::shutdown();
    return 0;
}