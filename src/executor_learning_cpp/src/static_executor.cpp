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
class HighRateControlNode : public rclcpp::Node
{
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
            10ms, // 100Hz 控制循环
            [this]()
            {
                auto jitter = measure_jitter();

                loop_count_++;
                if (loop_count_ % 100 == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[ControlLoop 100Hz] #%d, jitter: %.3fms",
                                loop_count_, jitter);
                }
            });

        // 10Hz 状态监测
        monitor_timer_ = this->create_wall_timer(
            100ms,
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Monitor 10Hz] control loop count: %d", loop_count_);
            });

        // 模拟传感器订阅
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sensor_data", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                // 在 StaticSingleThreadedExecutor 中，
                // 订阅回调和定时器回调严格串行，
                // 不会出现数据竞争
                RCLCPP_INFO(this->get_logger(),
                             "Received sensor data: %s", msg->data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "=== High-rate control node started ===");
        RCLCPP_INFO(this->get_logger(), "Using StaticSingleThreadedExecutor for minimum jitter");
    }

private:
    // 测量定时器抖动（与预期触发时间的偏差）
    double measure_jitter()
    {
        static auto last_time = std::chrono::steady_clock::now(); // static 保留上一次触发时间
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(now - last_time).count(); // 实际间隔(ms)
        last_time = now;
        // 期望 10ms，偏差即为抖动
        return std::abs(elapsed - 10.0);
    }

    int loop_count_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2

    auto node = std::make_shared<HighRateControlNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor; // 零分配的静态执行器
    executor.add_node(node);                                  // 首次 add_node 后，回调集合固定

    RCLCPP_INFO(node->get_logger(), "=== Using StaticSingleThreadedExecutor ===");
    RCLCPP_INFO(node->get_logger(), "Features: zero allocation, deterministic execution, minimum jitter");
    RCLCPP_INFO(node->get_logger(), "Limitation: runtime add/remove node not supported");

    executor.spin(); // 阻塞执行，回调集合不再变化

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
