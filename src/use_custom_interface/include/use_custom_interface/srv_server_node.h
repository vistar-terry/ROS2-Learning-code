#ifndef __HELLO_WORLD_INTERFACE_SRV_SERVER_NODE_H__
#define __HELLO_WORLD_INTERFACE_SRV_SERVER_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include "hello_world_interface/srv/set_led.hpp"

using namespace std::chrono_literals;

class CSrvServerNode : public rclcpp::Node
{
public:
    CSrvServerNode();
    ~CSrvServerNode();

private:
    void handleService(
        const std::shared_ptr<hello_world_interface::srv::SetLED::Request> request,
        std::shared_ptr<hello_world_interface::srv::SetLED::Response> response);

    rclcpp::Service<hello_world_interface::srv::SetLED>::SharedPtr m_setLEDServer;
};

#endif // __HELLO_WORLD_INTERFACE_SRV_SERVER_NODE_H__