#ifndef __HELLO_WORLD_INTERFACE_SRV_CLIENT_NODE_H__
#define __HELLO_WORLD_INTERFACE_SRV_CLIENT_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include "hello_world_interface/srv/set_led.hpp"

class CSrvClientNode : public rclcpp::Node
{
public:
    CSrvClientNode();
    ~CSrvClientNode();
 
    bool setLED(uint8_t led_id, bool led_on);

private:
    rclcpp::Client<hello_world_interface::srv::SetLED>::SharedPtr m_setLEDClient;
};

#endif // __HELLO_WORLD_INTERFACE_SRV_CLIENT_NODE_H__