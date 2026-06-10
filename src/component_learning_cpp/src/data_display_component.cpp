#include "component_learning_cpp/data_display_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

/// 数据显示组件实现
DataDisplayComponent::DataDisplayComponent(const rclcpp::NodeOptions & options)
: Node("data_display", options)
{
  // 声明参数
  this->declare_parameter("input_topic", std::string("filtered_temperature"));

  std::string input_topic = this->get_parameter("input_topic").as_string();

  sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
    input_topic, 10,
    std::bind(&DataDisplayComponent::on_filtered_temperature, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "DataDisplayComponent initialized, sub=%s",
    input_topic.c_str());
}

void DataDisplayComponent::on_filtered_temperature(
  const sensor_msgs::msg::Temperature::SharedPtr msg)
{
  count_++;
  RCLCPP_INFO(
    this->get_logger(),
    "[%4zu] Filtered temperature: %.2f C (variance: %.4f)",
    count_, msg->temperature, msg->variance);
}

RCLCPP_COMPONENTS_REGISTER_NODE(DataDisplayComponent)
