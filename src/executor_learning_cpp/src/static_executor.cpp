/**
 * @file static_executor.cpp
 * @brief ③ 静态单线程执行器 —— 零分配高性能执行
 *
 * 知识点：
 * - StaticSingleThreadedExecutor 的设计原理
 * - 与 SingleThreadedExecutor 的区别
 * - 适用场景：实时系统、嵌入式平台、确定性执行
 * - "零分配"的含义与性能优势
 * - 限制：不支持动态添加/移除节点
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// ================================================================
// 高频控制节点 —— 演示 StaticSingleThreadedExecutor 的实时性
// ================================================================
class HighRateControlNode : public rclcpp::Node {
public:
    HighRateControlNode()
        : Node("high_rate_control"), loop_count_(0)
    {
        // ================================================================
        // 100Hz 控制循环 —— 典型机器人控制频率
        //    StaticSingleThreadedExecutor 的优势：
        //    1. 不在 spin 循环中分配内存 → 无 GC 压力
        //    2. 固定的执行顺序 → 确定性延迟
        //    3. 更少的互斥锁操作 → 更低的 CPU 开销
        // ================================================================
        control_timer_ = this->create_wall_timer(
            10ms,  // 100Hz 控制循环
            [this]() {
                auto now = this->now();
                auto jitter = measure_jitter();

                loop_count_++;
                if (loop_count_ % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(),
                        "[控制循环 100Hz] 第%d次, 抖动=%.3fms",
                        loop_count_, jitter);
                }
            });

        // 10Hz 状态监测
        monitor_timer_ = this->create_wall_timer(
            100ms,
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                    "[监测 10Hz] 控制循环计数=%d", loop_count_);
            });

        // 模拟传感器订阅
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sensor_data", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                // 在 StaticSingleThreadedExecutor 中，
                // 订阅回调和定时器回调严格串行，
                // 不会出现数据竞争
                RCLCPP_DEBUG(this->get_logger(),
                    "收到传感器数据: %s", msg->data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "=== 高频控制节点已启动 ===");
        RCLCPP_INFO(this->get_logger(), "使用 StaticSingleThreadedExecutor 可获得最低抖动");
    }

private:
    // 测量定时器抖动（与预期触发时间的偏差）
    double measure_jitter() {
        static auto last_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(now - last_time).count();
        last_time = now;
        // 期望 10ms，偏差即为抖动
        return std::abs(elapsed - 10.0);
    }

    int loop_count_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HighRateControlNode>();

    // ================================================================
    // 三种执行器的对比（此处使用 StaticSingleThreadedExecutor）
    //
    // SingleThreadedExecutor:
    //   - 每次 spin_once 重新收集就绪回调 → 有内存分配
    //   - 支持动态 add_node / remove_node
    //   - 适用：一般应用、需要动态管理节点
    //
    // StaticSingleThreadedExecutor:
    //   - 首次 spin 时一次性收集所有回调 → 之后零分配
    //   - 不支持动态 add_node / remove_node（添加后不会生效）
    //   - 适用：实时系统、高频控制、嵌入式平台
    //
    // MultiThreadedExecutor:
    //   - 线程池并发执行 → 有互斥锁开销
    //   - 适用：多回调组并发、I/O 密集型
    // ================================================================

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "=== 使用 StaticSingleThreadedExecutor ===");
    RCLCPP_INFO(node->get_logger(), "特点: 零内存分配、确定性执行、最低抖动");
    RCLCPP_INFO(node->get_logger(), "限制: 不支持运行时添加/移除节点");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
