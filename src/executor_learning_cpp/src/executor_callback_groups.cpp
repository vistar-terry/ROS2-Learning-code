/**
 * @file executor_callback_groups.cpp
 * @brief ⑧ 执行器与回调组配合 —— 回调组如何影响执行器调度
 *
 * 知识点：
 * - 回调组 + 执行器的完整配合矩阵
 * - 同组互斥在不同执行器下的行为差异
 * - 服务调用场景下的死锁与解决
 * - executor.is_ready() / executor.spin_some() 高级用法
 * - 回调组优先级设计的实际技巧
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class ExecutorCallbackGroupNode : public rclcpp::Node
{
public:
    ExecutorCallbackGroupNode()
        : Node("executor_callback_groups"), timer_count_(0)
    {
        // ================================================================
        // 创建三种回调组配置
        // ================================================================

        // 组1: 互斥组 —— 用于定时器（保证串行执行）
        timer_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 组2: 互斥组 —— 用于服务端（保护服务处理逻辑）
        service_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 组3: 可重入组 —— 用于并发订阅处理
        reentrant_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // ================================================================
        // 1. 定时器 —— 绑定到 timer_group_
        // ================================================================
        fast_timer_ = this->create_wall_timer(
            100ms,
            [this]()
            {
                timer_count_++;
                RCLCPP_INFO(this->get_logger(),
                            "[Timer 10Hz] #%d, thread=%zu, group=timer_group",
                            timer_count_, get_thread_id());
            },
            timer_group_);

        // ================================================================
        // 2. 服务端 —— 绑定到 service_group_
        //    关键：服务端和调用方不能在同一互斥组！
        // ================================================================
        srv_ = this->create_service<example_interfaces::srv::Trigger>(
            "/test_trigger",
            [this](const std::shared_ptr<example_interfaces::srv::Trigger::Request> /*req*/,
                   std::shared_ptr<example_interfaces::srv::Trigger::Response> resp)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Service server] received request, thread=%zu, group=service_group",
                            get_thread_id());
                // 模拟耗时服务处理
                std::this_thread::sleep_for(200ms);
                resp->success = true;
                resp->message = "Service processing completed";
                RCLCPP_INFO(this->get_logger(), "[Service server] processing completed");
            },
            rclcpp::ServicesQoS(), service_group_);

        // ================================================================
        // 3. 客户端 —— 不指定回调组（使用默认组）
        //    注意：客户端不指定回调组时使用默认组
        //    默认组 = MutuallyExclusive，与定时器组不同 → 不会死锁
        // ================================================================
        client_ = this->create_client<example_interfaces::srv::Trigger>(
            "/test_trigger", rclcpp::ServicesQoS());

        // ================================================================
        // 4. 可重入组订阅 —— 演示并发处理
        //    C++ 中绑定回调组必须通过 SubscriptionOptions
        // ================================================================
        rclcpp::SubscriptionOptions reentrant_opts;
        reentrant_opts.callback_group = reentrant_group_; // 将订阅绑定到可重入组

        sub1_ = this->create_subscription<std_msgs::msg::String>(
            "/topic_a", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Subscription A] received: '%s', thread=%zu, group=reentrant",
                            msg->data.c_str(), get_thread_id());
                std::this_thread::sleep_for(100ms);
            },
            reentrant_opts);

        sub2_ = this->create_subscription<std_msgs::msg::String>(
            "/topic_b", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Subscription B] received: '%s', thread=%zu, group=reentrant",
                            msg->data.c_str(), get_thread_id());
                std::this_thread::sleep_for(100ms);
            },
            reentrant_opts);

        // ================================================================
        // 5. 周期调用服务 —— 测试回调组隔离效果
        // ================================================================
        call_timer_ = this->create_wall_timer(
            2s,
            [this]()
            {
                if (!client_->wait_for_service(100ms))
                { // 等待服务端就绪，超时100ms
                    RCLCPP_WARN(this->get_logger(), "Service not ready");
                    return;
                }
                auto req = std::make_shared<example_interfaces::srv::Trigger::Request>();
                // 异步调用服务（不等待结果，避免在回调中阻塞）
                client_->async_send_request(req, // 发送异步请求，回调中处理响应
                                            [this](rclcpp::Client<example_interfaces::srv::Trigger>::SharedFuture future)
                                            {
                                                auto result = future.get(); // 获取异步响应结果
                                                RCLCPP_INFO(this->get_logger(),
                                                            "[Client] service response: success=%d, msg='%s'",
                                                            result->success, result->message.c_str());
                                            });
                RCLCPP_INFO(this->get_logger(),
                            "[Client] async request sent");
            },
            timer_group_);

        RCLCPP_INFO(this->get_logger(), "=== Executor + callback groups node started ===");
        RCLCPP_INFO(this->get_logger(),
                    "Observe: server in service_group, client in timer_group, no mutual blocking");
        RCLCPP_INFO(this->get_logger(),
                    "Two subscriptions in reentrant group can process messages concurrently");
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int timer_count_;

    // 回调组
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;

    // ROS 实体
    rclcpp::TimerBase::SharedPtr fast_timer_;
    rclcpp::TimerBase::SharedPtr call_timer_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_;
    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2

    auto node = std::make_shared<ExecutorCallbackGroupNode>();

    // ================================================================
    // 使用 MultiThreadedExecutor 才能发挥回调组的作用！
    //
    // 配合矩阵（MultiThreadedExecutor 下）：
    //
    //   回调A \ 回调B │ 同一互斥组 │ 不同互斥组 │ 同一可重入组
    //   ──────────────┼────────────┼────────────┼──────────────
    //   同一互斥组     │   串行     │   并发     │   并发
    //   不同互斥组     │   并发     │   并发     │   并发
    //   同一可重入组   │   并发     │   并发     │   并发
    //
    // 如果用 SingleThreadedExecutor：所有回调都串行执行，
    // 回调组的区分毫无意义！
    // ================================================================

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4); // 4个线程，确保回调组间可充分并发
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(),
                "Using MultiThreadedExecutor (4 threads) for callback groups to take effect");

    executor.spin(); // 阻塞执行，回调组在多线程下才能发挥作用

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
