#include "hello_world_topic_cpp/listener.h"

CListener::CListener() : Node("listener_node")
{
    m_subscriber = this->create_subscription<std_msgs::msg::String>("/hello_world_topic", 10, 
        std::bind(&CListener::topic_callback, this, std::placeholders::_1));
}

CListener::~CListener()
{
}

void CListener::topic_callback(const std_msgs::msg::String &msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto listener_node = std::make_shared<CListener>();

    rclcpp::spin(listener_node);
    rclcpp::shutdown();
    return 0;
}