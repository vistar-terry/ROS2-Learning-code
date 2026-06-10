/**
 * @file logging_basic.cpp
 * @brief ① 基础日志使用 —— 五种日志级别与基本宏
 *
 * 知识点：
 * - RCLCPP_DEBUG / INFO / WARN / ERROR / FATAL 五种基本日志宏
 * - 日志级别过滤原理
 * - 日志消息格式（printf 风格与流式风格）
 * - 日志级别的获取与设置
 */

#include <rclcpp/rclcpp.hpp>
#include <iomanip>

class LoggingBasicNode : public rclcpp::Node
{
public:
  LoggingBasicNode() : Node("logging_basic")
  {
    RCLCPP_INFO(this->get_logger(), "=== Logging Basic Demo ===");

    // ================================================================
    // 五种日志级别的基本使用
    // ================================================================

    // DEBUG 级别：调试信息，默认不输出（默认级别为 INFO）
    RCLCPP_DEBUG(this->get_logger(), "This is a DEBUG message (invisible by default)");

    // INFO 级别：一般信息，默认输出
    RCLCPP_INFO(this->get_logger(), "This is an INFO message");

    // WARN 级别：警告信息
    RCLCPP_WARN(this->get_logger(), "This is a WARN message");

    // ERROR 级别：错误信息
    RCLCPP_ERROR(this->get_logger(), "This is an ERROR message");

    // FATAL 级别：致命错误
    RCLCPP_FATAL(this->get_logger(), "This is a FATAL message");

    // ================================================================
    // 格式化输出 —— 支持 printf 风格的格式化
    // ================================================================
    int count = 42;
    double value = 3.14159;
    std::string name = "sensor";

    RCLCPP_INFO(this->get_logger(), "Integer: %d, Double: %.2f, String: %s",
      count, value, name.c_str());

    // ================================================================
    // 流式日志 —— 使用 RCLCPP_INFO_STREAM 等宏（C++ 风格）
    // ================================================================
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Stream style: count=" << count << ", value=" << std::fixed << std::setprecision(2) << value);

    // ================================================================
    // 获取和设置日志级别
    // ================================================================
    // Jazzy 中使用 get_effective_level() 获取有效日志级别
    // （会考虑祖先 logger 的级别继承）
    auto current_level = this->get_logger().get_effective_level();
    RCLCPP_INFO(this->get_logger(), "Current effective log level: %d", static_cast<int>(current_level));

    // 设置日志级别为 DEBUG，此时所有级别的日志都会输出
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_DEBUG(this->get_logger(), "Now DEBUG messages are visible!");

    // 设置日志级别为 WARN，只输出 WARN 及以上
    this->get_logger().set_level(rclcpp::Logger::Level::Warn);
    RCLCPP_INFO(this->get_logger(), "This INFO will NOT be shown");
    RCLCPP_WARN(this->get_logger(), "This WARN WILL be shown");

    // 恢复 INFO 级别
    this->get_logger().set_level(rclcpp::Logger::Level::Info);
    RCLCPP_INFO(this->get_logger(), "Back to INFO level");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoggingBasicNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
