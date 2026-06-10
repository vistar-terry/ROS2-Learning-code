/**
 * @file logging_robot_demo.cpp
 * @brief ⑤ 机器人实战案例 —— 使用命名日志器和分层日志的多模块节点
 *
 * 知识点：
 * - 命名日志器的实战应用
 * - 多模块节点的日志分层
 * - 机器人状态监控的日志策略
 * - 条件日志在安全系统中的应用
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <chrono>
#include <random>

class RobotLoggingDemoNode : public rclcpp::Node
{
public:
  // Jazzy 中 rclcpp::Logger 默认构造函数为 private，
  // 必须在成员初始化列表中初始化
  RobotLoggingDemoNode() : Node("robot_logging_demo"),
    sensor_logger_(this->get_logger().get_child("sensor")),
    control_logger_(this->get_logger().get_child("control")),
    safety_logger_(this->get_logger().get_child("safety"))
  {
    // 安全模块设置较低阈值（WARN 以上），传感器模块可看 DEBUG
    safety_logger_.set_level(rclcpp::Logger::Level::Warn);
    sensor_logger_.set_level(rclcpp::Logger::Level::Debug);

    RCLCPP_INFO(this->get_logger(), "=== Robot Logging Demo Started ===");

    // ================================================================
    // 传感器模拟 —— 定时发布温度数据
    // ================================================================
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("robot/temperature", 10);

    sensor_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&RobotLoggingDemoNode::sensor_callback, this));

    // ================================================================
    // 安全监控 —— 定时检查安全状态
    // ================================================================
    safety_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotLoggingDemoNode::safety_callback, this));
  }

private:
  // 命名日志器
  rclcpp::Logger sensor_logger_;
  rclcpp::Logger control_logger_;
  rclcpp::Logger safety_logger_;

  // 发布器和定时器
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // 状态变量
  double temperature_ = 25.0;
  int sensor_count_ = 0;
  bool emergency_ = false;

  void sensor_callback()
  {
    sensor_count_++;

    // 模拟温度传感器数据（含噪声和异常）
    static std::mt19937 gen(42);
    static std::normal_distribution<double> noise(0.0, 2.0);
    temperature_ = 25.0 + noise(gen);

    // 偶尔模拟异常高温
    if (sensor_count_ % 20 == 0) {
      temperature_ = 95.0 + noise(gen);
    }

    // 发布数据
    auto msg = sensor_msgs::msg::Temperature();
    msg.header.stamp = this->now();
    msg.temperature = temperature_;
    msg.variance = 4.0;
    temp_pub_->publish(msg);

    // 使用命名日志器记录传感器信息
    RCLCPP_DEBUG(sensor_logger_, "Temperature reading: %.2f C", temperature_);

    // 限流记录传感器状态
    RCLCPP_INFO_THROTTLE(sensor_logger_, *this->get_clock(), 3000,
      "Sensor status OK (count=%d, temp=%.1f)", sensor_count_, temperature_);

    // 温度异常警告
    if (temperature_ > 80.0) {
      RCLCPP_WARN(sensor_logger_, "High temperature detected: %.1f C", temperature_);
    }
    if (temperature_ > 100.0) {
      RCLCPP_ERROR(sensor_logger_, "Critical temperature: %.1f C!", temperature_);
      emergency_ = true;
    }
  }

  void safety_callback()
  {
    if (emergency_) {
      RCLCPP_ERROR(safety_logger_, "EMERGENCY STOP! Temperature critical");
      RCLCPP_FATAL_THROTTLE(safety_logger_, *this->get_clock(), 2000,
        "System shutdown required - temperature out of safe range");
      emergency_ = false;  // 重置（演示用）
    } else {
      // 安全状态下仅在 DEBUG 级别输出
      RCLCPP_DEBUG(safety_logger_, "Safety check passed");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotLoggingDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
