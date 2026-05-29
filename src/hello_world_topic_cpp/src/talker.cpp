#include "hello_world_topic_cpp/talker.h"

using namespace std::chrono_literals;

CTalker::CTalker() : Node("talker_node"), m_count(0)
{
    m_publisher = this->create_publisher<std_msgs::msg::String>("/hello_world_topic", 10);
    m_timer = this->create_wall_timer(500ms, std::bind(&CTalker::timercallback, this));
}

CTalker::~CTalker()
{
}

void CTalker::timercallback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello world ROS2 Topic! " + std::to_string(this->m_count++);
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    this->m_publisher->publish(message);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto talker_node = std::make_shared<CTalker>();

    rclcpp::spin(talker_node);
    rclcpp::shutdown();
    return 0;
}