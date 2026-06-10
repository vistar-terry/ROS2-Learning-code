/**
 * @file logging_advanced.cpp
 * @brief ③ 高级日志 —— 条件日志、限流日志、首次日志
 *
 * 知识点：
 * - RCLCPP_INFO_ONCE：仅输出一次
 * - RCLCPP_INFO_THROTTLE：限流输出（指定时间间隔）
 * - RCLCPP_INFO_SKIPFIRST：跳过首次输出
 * - RCLCPP_INFO_EXPRESSION：条件表达式日志
 * - RCLCPP_INFO_FUNCTION：函数条件日志
 */

#include <rclcpp/rclcpp.hpp>

class LoggingAdvancedNode : public rclcpp::Node
{
public:
  LoggingAdvancedNode() : Node("logging_advanced")
  {
    RCLCPP_INFO(this->get_logger(), "=== Logging Advanced Demo ===");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LoggingAdvancedNode::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;

  void timer_callback()
  {
    count_++;

    // ================================================================
    // ONCE 宏：无论调用多少次，只输出第一次
    // 适用场景：初始化确认、一次性状态声明
    // ================================================================
    RCLCPP_INFO_ONCE(this->get_logger(),
      "[ONCE] This message appears only once (count=%d)", count_);

    // ================================================================
    // THROTTLE 宏：限流输出，指定时间间隔（毫秒）
    // 适用场景：高频循环中的日志，避免日志洪水
    // ================================================================
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "[THROTTLE 3s] Periodic log (count=%d)", count_);

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[THROTTLE 5s] Temperature warning check (count=%d)", count_);

    // ================================================================
    // SKIPFIRST 宏：跳过第一次输出，后续每次都输出
    // 适用场景：第一次是预期行为不需要记录，后续出现才值得关注
    // ================================================================
    RCLCPP_INFO_SKIPFIRST(this->get_logger(),
      "[SKIPFIRST] Skipped first, now showing (count=%d)", count_);

    // ================================================================
    // EXPRESSION 宏：条件表达式为 true 时才输出
    // 适用场景：只在特定条件下记录日志
    // ================================================================
    RCLCPP_INFO_EXPRESSION(this->get_logger(), count_ % 5 == 0,
      "[EXPRESSION] Count is multiple of 5 (count=%d)", count_);

    RCLCPP_WARN_EXPRESSION(this->get_logger(), count_ > 10,
      "[EXPRESSION] Count exceeded 10 (count=%d)", count_);

    // ================================================================
    // FUNCTION 宏：通过函数指针返回 bool 决定是否输出
    // 适用场景：无状态的复杂条件判断
    // 注意：要求传入函数指针（可被 * 解引用），
    //       不支持含捕获的 lambda 或 std::bind 返回值
    // ================================================================
    RCLCPP_INFO_FUNCTION(this->get_logger(),
      &LoggingAdvancedNode::should_log_now,
      "[FUNCTION] Conditional function log (count=%d)", count_);

    // 对于需要访问实例变量的条件判断，推荐改用 if + 普通日志宏：
    if (should_log(count_)) {
      RCLCPP_INFO(this->get_logger(),
        "[FUNCTION-STYLE] Stateful conditional log (count=%d)", count_);
    }

    // ================================================================
    // 条件日志组合：THROTTLE + EXPRESSION
    // 注意：ROS2 没有直接的组合宏，需要用条件语句包裹
    // ================================================================
    if (count_ % 3 == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[COMBO] Every 3rd callback, throttled 2s (count=%d)", count_);
    }
  }

  /// 函数条件日志的判断函数（无状态，供 RCLCPP_INFO_FUNCTION 使用）
  /// RCLCPP_INFO_FUNCTION 要求传入函数指针（支持 * 解引用），
  /// 无法传入含捕获的 lambda，因此该函数不能依赖实例变量
  static bool should_log_now()
  {
    static int n = 0;
    return ++n % 4 == 0;
  }

  /// 有状态的条件判断函数（供 if + 普通日志宏使用）
  static bool should_log(int count)
  {
    return count % 4 == 0;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoggingAdvancedNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
