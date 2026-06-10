#ifndef DATA_FILTER_COMPONENT_HPP_
#define DATA_FILTER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <deque>

/// 数据滤波组件 —— 对传感器数据进行滑动平均滤波
class DataFilterComponent : public rclcpp::Node
{
public:
  explicit DataFilterComponent(const rclcpp::NodeOptions & options);

private:
  void on_temperature(const sensor_msgs::msg::Temperature::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_;
  std::deque<double> buffer_;
  size_t window_size_ = 5;  // 滑动窗口大小
};

#endif  // DATA_FILTER_COMPONENT_HPP_
