/**
 * @file logging_performance.cpp
 * @brief ④ 日志性能与最佳实践
 *
 * 知识点：
 * - 日志级别检查（避免不必要的参数求值）
 * - 日志对实时性的影响
 * - 大量日志的限流策略
 * - 日志格式建议
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sstream>

class LoggingPerformanceNode : public rclcpp::Node
{
public:
  LoggingPerformanceNode() : Node("logging_performance")
  {
    RCLCPP_INFO(this->get_logger(), "=== Logging Performance Demo ===");

    // ================================================================
    // 1. 避免不必要的参数求值（重要性能优化）
    // ================================================================

    // ❌ 错误：即使 DEBUG 级别被过滤，heavy_computation() 仍然会被调用
    // RCLCPP_DEBUG(this->get_logger(), "Result: %s", heavy_computation().c_str());

    // ✅ 正确：先检查日志级别，避免不必要的计算
    // Jazzy 中没有 RCLCPP_LOG_ENABLED 宏，使用 get_effective_level() 手动判断
    if (this->get_logger().get_effective_level() <= rclcpp::Logger::Level::Debug) {
      std::string result = heavy_computation();
      RCLCPP_DEBUG(this->get_logger(), "Result: %s", result.c_str());
    }

    // ================================================================
    // 2. 日志性能基准测试
    // ================================================================
    benchmark_logging();

    // ================================================================
    // 3. 定时器演示不同日志策略
    // ================================================================
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LoggingPerformanceNode::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;

  /// 模拟重计算操作
  std::string heavy_computation()
  {
    std::ostringstream oss;
    for (int i = 0; i < 1000; ++i) {
      oss << i << ",";
    }
    return oss.str();
  }

  /// 日志性能基准测试
  void benchmark_logging()
  {
    const int N = 100000;

    // 测试被过滤日志的开销（DEBUG 级别，默认不输出）
    this->get_logger().set_level(rclcpp::Logger::Level::Info);
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; ++i) {
      RCLCPP_DEBUG(this->get_logger(), "Filtered message %d", i);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto filtered_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // 测试级别预检查的开销（Jazzy 中使用 get_effective_level() 替代 RCLCPP_LOG_ENABLED）
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; ++i) {
      if (this->get_logger().get_effective_level() <= rclcpp::Logger::Level::Debug) {
        RCLCPP_DEBUG(this->get_logger(), "Filtered message %d", i);
      }
    }
    end = std::chrono::high_resolution_clock::now();
    auto checked_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    RCLCPP_INFO(this->get_logger(),
      "Benchmark (%d iterations): filtered=%.0f us, checked=%.0f us",
      N, static_cast<double>(filtered_us), static_cast<double>(checked_us));
    RCLCPP_INFO(this->get_logger(),
      "Overhead per filtered log: %.3f us",
      static_cast<double>(filtered_us) / N);
  }

  void timer_callback()
  {
    count_++;

    // ✅ 推荐：高频回调中使用 THROTTLE 限流
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "High-frequency callback (count=%d)", count_);

    // ✅ 推荐：关键事件使用 ONCE 或 EXPRESSION
    if (count_ == 100) {
      RCLCPP_INFO(this->get_logger(), "Reached 100 callbacks");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoggingPerformanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
