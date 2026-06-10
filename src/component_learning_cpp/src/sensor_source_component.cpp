#include "component_learning_cpp/sensor_source_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>  // ★ 必须包含注册宏头文件
#include <random>

/// 传感器数据源组件实现
SensorSourceComponent::SensorSourceComponent(const rclcpp::NodeOptions & options)
: Node("sensor_source", options)
{
  // 声明参数
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("base_temperature", 25.0);
  this->declare_parameter("noise_amplitude", 2.0);

  temperature_ = this->get_parameter("base_temperature").as_double();
  double rate = this->get_parameter("publish_rate").as_double();
  double noise = this->get_parameter("noise_amplitude").as_double();

  // 创建发布器
  pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("raw_temperature", 10);

  // 创建定时器
  auto period = std::chrono::duration<double>(1.0 / rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SensorSourceComponent::publish_temperature, this));

  RCLCPP_INFO(
    this->get_logger(),
    "SensorSourceComponent initialized, rate=%.1f Hz, base_temp=%.1f, noise=%.1f",
    rate, temperature_, noise);
}

void SensorSourceComponent::publish_temperature()
{
  auto msg = sensor_msgs::msg::Temperature();
  msg.header.stamp = this->now();
  msg.header.frame_id = "sensor_frame";

  // 添加高斯噪声模拟真实传感器
  static std::mt19937 gen(42);
  static std::normal_distribution<double> dist(0.0, 1.0);
  double noise_amp = this->get_parameter("noise_amplitude").as_double();
  msg.temperature = temperature_ + dist(gen) * noise_amp;
  msg.variance = noise_amp * noise_amp;

  // ★ 使用 unique_ptr 发布，启用进程内零拷贝优化
  pub_->publish(std::move(msg));
}

// ★ 注册组件：将 SensorSourceComponent 注册为可动态加载的组件
RCLCPP_COMPONENTS_REGISTER_NODE(SensorSourceComponent)
