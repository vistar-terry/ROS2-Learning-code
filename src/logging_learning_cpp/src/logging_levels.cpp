/**
 * @file logging_levels.cpp
 * @brief ② 日志级别控制 —— 运行时动态调整与外部配置
 *
 * 知识点：
 * - 通过参数设置日志级别
 * - 命名日志器的独立级别控制
 * - 日志器层级与继承
 * - 运行时动态切换日志级别
 */

#include <rclcpp/rclcpp.hpp>

class LoggingLevelsNode : public rclcpp::Node
{
public:
  // Jazzy 中 rclcpp::Logger 默认构造函数为 private，
  // 必须在成员初始化列表中初始化，不能先默认构造再赋值
  LoggingLevelsNode() : Node("logging_levels"),
    sensor_logger_(this->get_logger().get_child("sensor")),
    motor_logger_(this->get_logger().get_child("motor"))
  {
    // ================================================================
    // 方式 1：通过参数设置日志级别
    // ================================================================
    this->declare_parameter("log_level", std::string("info"));

    std::string log_level_str = this->get_parameter("log_level").as_string();
    auto level = level_from_string(log_level_str);
    this->get_logger().set_level(level);
    RCLCPP_INFO(this->get_logger(), "Log level set via parameter: %s", log_level_str.c_str());

    // ================================================================
    // 命名日志器 —— 为子模块创建独立日志器
    // ================================================================

    // 子日志器已在初始化列表中创建
    // （层级关系：logging_levels.sensor / logging_levels.motor）

    // 为子日志器设置不同级别
    sensor_logger_.set_level(rclcpp::Logger::Level::Debug);  // 传感器模块看调试日志
    motor_logger_.set_level(rclcpp::Logger::Level::Warn);    // 电机模块只看警告以上

    RCLCPP_INFO(sensor_logger_, "Sensor logger initialized (DEBUG level)");
    RCLCPP_DEBUG(sensor_logger_, "Sensor debug info visible");
    RCLCPP_INFO(motor_logger_, "Motor logger initialized (WARN level, this INFO is invisible)");
    RCLCPP_WARN(motor_logger_, "Motor warning is visible");

    // ================================================================
    // 定时器演示日志级别动态切换
    // ================================================================
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&LoggingLevelsNode::timer_callback, this));
  }

private:
  rclcpp::Logger sensor_logger_;
  rclcpp::Logger motor_logger_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;

  void timer_callback()
  {
    count_++;

    // 每隔 10 次切换一次日志级别
    if (count_ % 10 == 0) {
      auto new_level = (count_ / 10) % 2 == 0
        ? rclcpp::Logger::Level::Debug
        : rclcpp::Logger::Level::Info;
      this->get_logger().set_level(new_level);
      RCLCPP_INFO(this->get_logger(), "Log level switched to %s",
        new_level == rclcpp::Logger::Level::Debug ? "DEBUG" : "INFO");
    }

    RCLCPP_DEBUG(this->get_logger(), "Callback #%d (DEBUG)", count_);
    RCLCPP_INFO(this->get_logger(), "Callback #%d (INFO)", count_);
    RCLCPP_WARN(this->get_logger(), "Callback #%d (WARN)", count_);
  }

  static rclcpp::Logger::Level level_from_string(const std::string & level_str)
  {
    if (level_str == "debug") return rclcpp::Logger::Level::Debug;
    if (level_str == "info")  return rclcpp::Logger::Level::Info;
    if (level_str == "warn")  return rclcpp::Logger::Level::Warn;
    if (level_str == "error") return rclcpp::Logger::Level::Error;
    if (level_str == "fatal") return rclcpp::Logger::Level::Fatal;
    return rclcpp::Logger::Level::Info;  // 默认
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoggingLevelsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
