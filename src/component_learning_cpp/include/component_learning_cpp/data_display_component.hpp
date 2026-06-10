#ifndef DATA_DISPLAY_COMPONENT_HPP_
#define DATA_DISPLAY_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

/// 数据显示组件 —— 订阅滤波后的温度数据并打印
class DataDisplayComponent : public rclcpp::Node
{
public:
  explicit DataDisplayComponent(const rclcpp::NodeOptions & options);

private:
  void on_filtered_temperature(const sensor_msgs::msg::Temperature::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr sub_;
  size_t count_ = 0;  // 收到的消息计数
};

#endif  // DATA_DISPLAY_COMPONENT_HPP_
