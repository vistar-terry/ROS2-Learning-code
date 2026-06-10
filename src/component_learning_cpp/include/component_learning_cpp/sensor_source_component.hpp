#ifndef SENSOR_SOURCE_COMPONENT_HPP_
#define SENSOR_SOURCE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <chrono>

/// 传感器数据源组件 —— 模拟温度传感器发布数据
class SensorSourceComponent : public rclcpp::Node
{
public:
  // ★ 组件必须使用 NodeOptions 参数的构造函数
  explicit SensorSourceComponent(const rclcpp::NodeOptions & options);

private:
  void publish_temperature();

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double temperature_ = 25.0;  // 初始温度值
};

#endif  // SENSOR_SOURCE_COMPONENT_HPP_
