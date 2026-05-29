// hello_world_cpp/src/chatter_node_manager.cpp

#include "hello_world_cpp/chatter_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

class ChatterNodeManager : public rclcpp::Node
{
public:
    ChatterNodeManager()
        : Node("chatter_node_manager")
    {
        rclcpp::Node::SharedPtr node = rclcpp::Node::SharedPtr(this);
        publisher_ = std::make_shared<hello_world_cpp::ChatterPublisher>(node, "chatter");
        publisher_->start_publishing(std::chrono::seconds(1));
    }
 
private:
    std::shared_ptr<hello_world_cpp::ChatterPublisher> publisher_;
};
 
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChatterNodeManager>());
    rclcpp::shutdown();
    return 0;
}