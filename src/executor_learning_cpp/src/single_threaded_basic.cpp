/**
 * @file single_threaded_basic.cpp
 * @brief ① 单线程执行器基础 —— 理解 rclcpp::spin() 背后的原理
 *
 * 知识点：
 * - rclcpp::spin() 的内部工作机制
 * - SingleThreadedExecutor 的手动创建与使用
 * - 单线程执行器中回调的串行执行特性
 * - 执行器如何从等待队列中取出就绪的回调
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SingleThreadedBasicNode : public rclcpp::Node
{
public:
    SingleThreadedBasicNode()
        : Node("single_threaded_basic")
        , count_(0)
    {
        // ================================================================
        // 1. 创建两个定时器 —— 模拟不同频率的回调
        //    在 SingleThreadedExecutor 中，回调严格串行执行
        // ================================================================
        fast_timer_ = this->create_wall_timer(
            200ms, // 5Hz
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                            "[FastTimer 5Hz][thread %zu] count: %d",
                            get_thread_id(), ++count_);
            });

        slow_timer_ = this->create_wall_timer(
            1000ms, // 1Hz
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                    "[SlowTimer 1Hz][thread %zu] ---- start simulated heavy task ----", 
                    get_thread_id());

                // 模拟 800ms 的处理延迟
                std::this_thread::sleep_for(800ms);

                RCLCPP_INFO(this->get_logger(),
                    "[SlowTimer 1Hz][thread %zu] ---- heavy task completed ----", 
                    get_thread_id());
            });

        // ================================================================
        // 2. 创建订阅 —— 观察定时器延迟对订阅的影响
        //    在单线程执行器中，如果慢定时器回调正在执行，
        //    订阅回调只能等待 → 消息处理延迟
        // ================================================================
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/test_topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(),
                    "[Subscription][thread %zu] received: '%s'",
                    get_thread_id(), msg->data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), 
            "=== SingleThreadedExecutor basic node started ===");
        RCLCPP_INFO(this->get_logger(), 
            "Observe: fast timer and subscription callbacks are blocked while slow timer runs");
        RCLCPP_INFO(this->get_logger(), 
            "All callbacks execute serially in the same thread");
    }

private:
    // 获取当前线程 ID（简化显示）
    // 将原生线程 ID 转为字符串后哈希取模，方便观察对比
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int count_;
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2 通信层

    auto node = std::make_shared<SingleThreadedBasicNode>(); // 创建节点智能指针

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
    rclcpp::executors::SingleThreadedExecutor executor; // 手动创建单线程执行器
    executor.add_node(node);                            // 将节点注册到执行器

    // spin() 会阻塞当前线程，持续从等待集中取出就绪回调并执行
    executor.spin();

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
