/**
 * @file events_cbg_executor.cpp
 * @brief ⑪ EventsCBGExecutor —— 事件驱动 + 回调组（CBG）的执行器
 *
 * 知识点：
 * - EventsCBGExecutor 的设计原理（事件队列 + 定时器管理器，无轮询）
 * - 与 SingleThreadedExecutor / MultiThreadedExecutor（轮询式）的核心区别
 * - 每个回调组（CBG）拥有独立事件队列
 * - 线程池（worker）并发取走就绪事件执行
 * - 实体必须显式归属回调组（事件驱动模型的核心约束）
 * - 适用场景：高频率话题、低延迟实时系统、事件密集型应用
 */

#include <rclcpp/rclcpp.hpp>
#include <cm_executors/events_cbg_executor.hpp>  // EventsCBGExecutor 来自 cm_executors 包（cellumation），非 rclcpp 自带
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

class EventsCBGNode : public rclcpp::Node
{
public:
    EventsCBGNode()
        : Node("events_cbg_node")
        , sensor_count_(0)
        , proc_count_(0)
    {
        // ================================================================
        // 1. 创建回调组（CBG）
        //    EventsCBGExecutor 的核心：每个回调组拥有**独立事件队列**。
        //    实体（定时器/订阅/服务/客户端）必须显式归属某个回调组，
        //    事件到达时由底层 DDS / 定时器直接 push 到对应队列，
        //    由线程池 worker 取走执行 —— 完全不需要执行器反复轮询等待集。
        // ================================================================

        // 传感器组：高频数据，必须低延迟、不被长任务阻塞
        sensor_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 处理组：低频重计算，耗时操作
        process_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 报告组：可重入，允许与外部消息事件并发处理
        report_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // ================================================================
        // 2. 高频"传感器"定时器（50Hz）—— 绑定 sensor_group_
        //    模拟高频率数据到达事件，强调事件驱动的低延迟特性。
        //    用 DEBUG 级别避免刷屏，可用 --log-level debug 查看每一次事件。
        // ================================================================
        sensor_timer_ = this->create_wall_timer(
            20ms, // 50Hz
            [this]()
            {
                sensor_count_++;
                RCLCPP_DEBUG(this->get_logger(),
                             "[Sensor 50Hz][thread %zu] event #%d",
                             get_thread_id(), sensor_count_.load());
            },
            sensor_group_);

        // ================================================================
        // 3. 重计算定时器（3s，耗时 1.2s）—— 绑定 process_group_
        //    即使该回调长时间占用一个 worker 线程，
        //    也不会阻塞 sensor_group_ 的事件队列（线程池并发）。
        // ================================================================
        process_timer_ = this->create_wall_timer(
            3000ms,
            [this]()
            {
                proc_count_++;
                RCLCPP_INFO(this->get_logger(),
                            "[Process][thread %zu] start heavy task #%d (sleep 1.2s)",
                            get_thread_id(), proc_count_.load());
                std::this_thread::sleep_for(1200ms); // 模拟耗时计算
                RCLCPP_INFO(this->get_logger(),
                            "[Process][thread %zu] heavy task #%d done",
                            get_thread_id(), proc_count_.load());
            },
            process_group_);

        // ================================================================
        // 4. 状态报告定时器（1Hz）—— 绑定 report_group_（可重入）
        // ================================================================
        report_timer_ = this->create_wall_timer(
            1000ms,
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Report][thread %zu] sensor: %d, process: %d",
                            get_thread_id(), sensor_count_.load(), proc_count_.load());
            },
            report_group_);

        // ================================================================
        // 5. 订阅 —— 绑定 report_group_（可重入组）
        //    真实外部消息到达时，rmw 层直接通知执行器 push 事件，
        //    无需执行器主动轮询等待集。
        // ================================================================
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = report_group_;

        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/events_demo_topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Subscription][thread %zu] event received: '%s'",
                            get_thread_id(), msg->data.c_str());
            },
            sub_opts);

        RCLCPP_INFO(this->get_logger(), "=== EventsCBGExecutor demo node started ===");
        RCLCPP_INFO(this->get_logger(),
                    "Entity events are pushed to per-CBG queues; worker threads execute them (event-driven, no polling)");
        RCLCPP_INFO(this->get_logger(),
                    "Publish to /events_demo_topic to observe incoming-message events");
    }

private:
    // 获取当前线程 ID（简化显示）
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    std::atomic<int> sensor_count_;
    std::atomic<int> proc_count_;

    // 回调组（每个 CBG 对应一个独立事件队列）
    rclcpp::CallbackGroup::SharedPtr sensor_group_;
    rclcpp::CallbackGroup::SharedPtr process_group_;
    rclcpp::CallbackGroup::SharedPtr report_group_;

    rclcpp::TimerBase::SharedPtr sensor_timer_;
    rclcpp::TimerBase::SharedPtr process_timer_;
    rclcpp::TimerBase::SharedPtr report_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2 通信层

    auto node = std::make_shared<EventsCBGNode>();

    // ================================================================
    // 创建 EventsCBGExecutor
    //
    // 构造函数：
    //   EventsCBGExecutor(
    //     ExecutorOptions options   = {},          // 通用执行器选项
    //     size_t number_of_threads  = 0,           // worker 线程数，0=CPU 核数(最少2)
    //     nanoseconds timeout       = -1)          // worker 等待事件的超时，-1=无限等待
    //
    // 与轮询式执行器对比：
    //   SingleThreadedExecutor / MultiThreadedExecutor：
    //     while(ok) { wait_for_ready_callbacks(timeout); execute(); }
    //     → 周期性唤醒等待集，存在固定轮询延迟，空闲也占用 CPU
    //   EventsCBGExecutor：
    //     DDS 有数据 / 定时器触发 → 直接 push 事件到 CBG 队列
    //     → worker 线程立即取走执行，延迟更低、空闲零 CPU 占用
    // ================================================================
    rclcpp::executors::EventsCBGExecutor executor(
        rclcpp::ExecutorOptions(), 4); // 4 个 worker 线程，足以并发处理不同回调组

    RCLCPP_INFO(node->get_logger(),
                "Using EventsCBGExecutor with %zu worker threads",
                executor.get_number_of_threads());

    executor.add_node(node); // 注册节点（其回调组随之注册到执行器）
    executor.spin();         // 阻塞当前线程，worker 线程池后台驱动事件

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
