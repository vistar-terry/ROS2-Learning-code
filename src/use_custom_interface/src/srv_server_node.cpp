#include "use_custom_interface/srv_server_node.h"

CSrvServerNode::CSrvServerNode() : Node("srv_server_node")
{
    m_setLEDServer = this->create_service<hello_world_interface::srv::SetLED>(
        "/useInterface/srv/setLED",
        std::bind(&CSrvServerNode::handleService, this,
                    std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "setLED service is ready...");
}

CSrvServerNode::~CSrvServerNode()
{

}

void CSrvServerNode::handleService(
    const std::shared_ptr<hello_world_interface::srv::SetLED::Request> request,
    std::shared_ptr<hello_world_interface::srv::SetLED::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "setLED %d %s", request->led_id, request->led_on ? "on" : "off");

    response->success = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSrvServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}