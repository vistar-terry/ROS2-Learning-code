#include "use_custom_interface/srv_client_node.h"

using namespace std::chrono_literals;

CSrvClientNode::CSrvClientNode() : Node("srv_client_node")
{
    m_setLEDClient = this->create_client<hello_world_interface::srv::SetLED>("/useInterface/srv/setLED");
}

CSrvClientNode::~CSrvClientNode()
{
}

bool CSrvClientNode::setLED(uint8_t led_id, bool led_on)
{
    RCLCPP_INFO(this->get_logger(), "setLED %d %s", led_id, led_on ? "on" : "off");

    if (!m_setLEDClient->wait_for_service(10s))
    {
        RCLCPP_ERROR(this->get_logger(), "setLED, wait for service init timeout.");

        return false;
    }

    auto request = std::make_shared<hello_world_interface::srv::SetLED::Request>();
    request->led_id = led_id;
    request->led_on = led_on;

    auto future = m_setLEDClient->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    if (result == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();

        return response->success;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "call service failed.");

        return false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<CSrvClientNode>();
    bool success = client_node->setLED(1, true);
    RCLCPP_INFO(rclcpp::get_logger("main"), "setLED %s", success ? "succeed" : "failed");

    rclcpp::shutdown();
    return 0;
}