/**
 * @file single_threaded_basic.cpp
 * @brief ① 单线程执行器基础 —— 理解 rclcpp::spin() 背后的原理
 *
 * 知识点：
 * - rclcpp::spin() 的内部工作机制
 * - SingleThreadedExecutor 的手动创建与使用
 * - spin / spin_once / spin_some 的区别
 * - 单线程执行器中回调的串行执行特性
 * - 执行器如何从等待队列中取出就绪的回调
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SingleThreadedBasicNode : public rclcpp::Node {
public:
    SingleThreadedBasicNode()
        : Node("single_threaded_basic"), count_(0)
    {
        // ================================================================
        // 1. 创建两个定时器 —— 模拟不同频率的回调
        //    在 SingleThreadedExecutor 中，回调严格串行执行
        // ================================================================
        fast_timer_ = this->create_wall_timer(
            100ms,  // 10Hz
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                    "[快定时器 10Hz] 计数=%d, 线程=%zu",
                    ++count_, get_thread_id());
            });

        slow_timer_ = this->create_wall_timer(
            500ms,  // 2Hz
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                    "[慢定时器 2Hz] ---- 开始模拟耗时操作 ----");
                // 模拟 200ms 的处理延迟
                std::this_thread::sleep_for(200ms);
                RCLCPP_INFO(this->get_logger(),
                    "[慢定时器 2Hz] ---- 耗时操作完成 ----");
            });

        // ================================================================
        // 2. 创建订阅 —— 观察定时器延迟对订阅的影响
        //    在单线程执行器中，如果慢定时器回调正在执行，
        //    订阅回调只能等待 → 消息处理延迟
        // ================================================================
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/test_topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(),
                    "[订阅] 收到: '%s', 线程=%zu",
                    msg->data.c_str(), get_thread_id());
            });

        RCLCPP_INFO(this->get_logger(), "=== 单线程执行器基础节点已启动 ===");
        RCLCPP_INFO(this->get_logger(), "观察: 慢定时器执行时，快定时器和订阅回调会被阻塞");
        RCLCPP_INFO(this->get_logger(), "所有回调在同一线程中串行执行");
    }

private:
    // 获取当前线程 ID（简化显示）
    std::size_t get_thread_id() {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int count_;
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SingleThreadedBasicNode>();

    // ================================================================
    // 方式一：rclcpp::spin() —— 最简单的方式
    //    内部创建 SingleThreadedExecutor 并调用 spin()
    //    适用于大多数简单场景
    // ================================================================
    // rclcpp::spin(node);

    // ================================================================
    // 方式二：手动创建 SingleThreadedExecutor
    //    更灵活，可以精确控制执行流程
    // ================================================================
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "使用 SingleThreadedExecutor 手动创建");
    RCLCPP_INFO(node->get_logger(), "调用 executor.spin() 开始执行回调...");

    // spin() 会阻塞当前线程，持续从等待集中取出就绪回调并执行
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
