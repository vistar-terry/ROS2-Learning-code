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

class ExecutorCallbackGroupNode : public rclcpp::Node {
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
            [this]() {
                timer_count_++;
                RCLCPP_INFO(this->get_logger(),
                    "[定时器 10Hz] #%d, 线程=%zu, 组=timer_group",
                    timer_count_, get_thread_id());
            }, timer_group_);

        // ================================================================
        // 2. 服务端 —— 绑定到 service_group_
        //    关键：服务端和调用方不能在同一互斥组！
        // ================================================================
        srv_ = this->create_service<example_interfaces::srv::Trigger>(
            "/test_trigger",
            [this](const std::shared_ptr<example_interfaces::srv::Trigger::Request> /*req*/,
                   std::shared_ptr<example_interfaces::srv::Trigger::Response> resp) {
                RCLCPP_INFO(this->get_logger(),
                    "[服务端] 收到请求, 线程=%zu, 组=service_group",
                    get_thread_id());
                // 模拟耗时服务处理
                std::this_thread::sleep_for(200ms);
                resp->success = true;
                resp->message = "服务处理完成";
                RCLCPP_INFO(this->get_logger(), "[服务端] 处理完成");
            }, rclcpp::ServicesQoS(), service_group_);

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
        reentrant_opts.callback_group = reentrant_group_;

        sub1_ = this->create_subscription<std_msgs::msg::String>(
            "/topic_a", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(),
                    "[订阅A] 收到: '%s', 线程=%zu, 组=reentrant",
                    msg->data.c_str(), get_thread_id());
                std::this_thread::sleep_for(100ms);
            }, reentrant_opts);

        sub2_ = this->create_subscription<std_msgs::msg::String>(
            "/topic_b", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(),
                    "[订阅B] 收到: '%s', 线程=%zu, 组=reentrant",
                    msg->data.c_str(), get_thread_id());
                std::this_thread::sleep_for(100ms);
            }, reentrant_opts);

        // ================================================================
        // 5. 周期调用服务 —— 测试回调组隔离效果
        // ================================================================
        call_timer_ = this->create_wall_timer(
            2s,
            [this]() {
                if (!client_->wait_for_service(100ms)) {
                    RCLCPP_WARN(this->get_logger(), "服务未就绪");
                    return;
                }
                auto req = std::make_shared<example_interfaces::srv::Trigger::Request>();
                // 异步调用服务（不等待结果，避免在回调中阻塞）
                client_->async_send_request(req,
                    [this](rclcpp::Client<example_interfaces::srv::Trigger>::SharedFuture future) {
                        auto result = future.get();
                        RCLCPP_INFO(this->get_logger(),
                            "[客户端] 服务响应: success=%d, msg='%s'",
                            result->success, result->message.c_str());
                    });
                RCLCPP_INFO(this->get_logger(),
                    "[客户端] 已发送异步请求");
            }, timer_group_);

        RCLCPP_INFO(this->get_logger(), "=== 执行器+回调组配合节点已启动 ===");
        RCLCPP_INFO(this->get_logger(),
            "观察: 服务端在service_group，客户端在timer_group，互不阻塞");
        RCLCPP_INFO(this->get_logger(),
            "可重入组的两个订阅可以并发处理消息");
    }

private:
    std::size_t get_thread_id() {
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

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
        rclcpp::ExecutorOptions(), 4);  // 4个线程
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(),
        "使用 MultiThreadedExecutor (4线程)，回调组才能发挥作用");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
