#include "component_learning_cpp/data_filter_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

/// 数据滤波组件实现
DataFilterComponent::DataFilterComponent(const rclcpp::NodeOptions & options)
: Node("data_filter", options)
{
  // 声明参数
  this->declare_parameter("window_size", 5);
  this->declare_parameter("input_topic", std::string("raw_temperature"));
  this->declare_parameter("output_topic", std::string("filtered_temperature"));

  window_size_ = this->get_parameter("window_size").as_int();
  std::string input_topic = this->get_parameter("input_topic").as_string();
  std::string output_topic = this->get_parameter("output_topic").as_string();

  // 创建订阅器和发布器
  sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
    input_topic, 10,
    std::bind(&DataFilterComponent::on_temperature, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(output_topic, 10);

  RCLCPP_INFO(
    this->get_logger(),
    "DataFilterComponent initialized, window_size=%zu, sub=%s, pub=%s",
    window_size_, input_topic.c_str(), output_topic.c_str());
}

void DataFilterComponent::on_temperature(
  const sensor_msgs::msg::Temperature::SharedPtr msg)
{
  buffer_.push_back(msg->temperature);
  if (buffer_.size() > window_size_) {
    buffer_.pop_front();
  }

  // 计算滑动平均
  double sum = 0.0;
  for (double v : buffer_) {
    sum += v;
  }
  double avg = sum / static_cast<double>(buffer_.size());

  // 发布滤波后的数据
  auto out = sensor_msgs::msg::Temperature();
  out.header = msg->header;
  out.temperature = avg;
  out.variance = 0.0;  // 滤波后方差设为 0（简化处理）

  pub_->publish(std::move(out));

  RCLCPP_DEBUG(
    this->get_logger(),
    "Raw: %.2f -> Filtered: %.2f (window: %zu)",
    msg->temperature, avg, buffer_.size());
}

RCLCPP_COMPONENTS_REGISTER_NODE(DataFilterComponent)
