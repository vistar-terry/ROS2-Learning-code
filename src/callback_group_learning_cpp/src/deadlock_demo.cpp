/**
 * ===========================================================================
 *  ⑧ 死锁演示与解决 (C++版)
 * ===========================================================================
 *
 * 死锁场景1（经典）：
 *   - 节点有一个服务端和一个定时器
 *   - 定时器回调中调用节点自己的服务
 *   - 如果服务端和定时器在同一个 MutuallyExclusive 回调组中
 *   - 定时器回调持有组锁，等待服务响应
 *   - 服务回调需要获取组锁才能执行
 *   - 结果：死锁！
 *
 * C++ 死锁相关 API：
 *   - 创建服务端: node->create_service<SrvType>(name, callback, group)
 *   - 创建客户端: node->create_client<SrvType>(name, group)
 *   - 异步调用: client->async_send_request(request)
 *   - C++ 中不存在 Python 那样的 rclpy.spin_until_future_complete
 *     但可以用 executor.spin_until_future_complete(future)
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <sstream>

using namespace std::chrono_literals;

// ================================================================
// 死锁版本 —— 演示问题
// ================================================================

class DeadlockNode : public rclcpp::Node
{
public:
    DeadlockNode() : Node("deadlock_node")
    {
        // ================================================================
        // 错误设计：服务端和定时器在同一个 MutuallyExclusive 回调组
        // 这会导致定时器回调中调用服务时产生死锁！
        // ================================================================
        bad_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 服务端 —— 在 bad_group_ 中
        srv_ = this->create_service<example_interfaces::srv::Trigger>(
            "/deadlock_trigger",
            std::bind(&DeadlockNode::service_callback, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), bad_group_);

        // 客户端 —— 用于调用服务
        client_ = this->create_client<example_interfaces::srv::Trigger>(
            "/deadlock_trigger",
            rclcpp::ServicesQoS(), bad_group_);

        // 定时器 —— 也在 bad_group_ 中
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&DeadlockNode::timer_callback, this),
            bad_group_);

        RCLCPP_WARN(this->get_logger(), "================================================================");
        RCLCPP_WARN(this->get_logger(), "死锁演示节点启动 (C++)");
        RCLCPP_WARN(this->get_logger(), "  服务端和定时器在同一 MutuallyExclusive 回调组");
        RCLCPP_WARN(this->get_logger(), "  定时器回调中调用自己的服务 → 会死锁！");
        RCLCPP_WARN(this->get_logger(), "================================================================");
    }

private:
    /**
     * 服务回调 —— 需要 bad_group_ 的锁才能执行
     */
    void service_callback(
        const std::shared_ptr<example_interfaces::srv::Trigger::Request> /*request*/,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[服务] 收到请求，处理中...");
        std::this_thread::sleep_for(500ms);
        response->success = true;
        response->message = "服务处理完成";
    }

    /**
     * 定时器回调 —— 持有 bad_group_ 的锁
     */
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "[定时器] 调用自己的服务...");

        // ============================================================
        // 死锁发生点！
        // 1. timer_callback 获取了 bad_group_ 的互斥锁
        // 2. 调用服务需要服务回调执行
        // 3. 服务回调也在 bad_group_ 中，需要获取同一个互斥锁
        // 4. 但互斥锁已被 timer_callback 持有
        // 5. 死锁！
        // ============================================================
        auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
        auto future = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "[定时器] 服务调用已发送，等待响应...");

        // 在 MutuallyExclusive 回调组中，服务回调永远不会被执行
        // 因为组锁被定时器回调持有
    }

    rclcpp::CallbackGroup::SharedPtr bad_group_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_;
    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// ================================================================
// 正确版本 —— 解决死锁
// ================================================================

class FixedNode : public rclcpp::Node
{
public:
    FixedNode() : Node("fixed_node"), call_count_(0)
    {
        // ================================================================
        // 正确设计：服务端和定时器使用不同的回调组
        // ================================================================
        timer_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        service_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 服务端 —— 在 service_group_ 中
        srv_ = this->create_service<example_interfaces::srv::Trigger>(
            "/fixed_trigger",
            std::bind(&FixedNode::service_callback, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), service_group_);

        // 客户端 —— 调用服务
        client_ = this->create_client<example_interfaces::srv::Trigger>(
            "/fixed_trigger",
            rclcpp::ServicesQoS(), service_group_);

        // 定时器 —— 在 timer_group_ 中
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&FixedNode::timer_callback, this),
            timer_group_);

        // 订阅 —— 在 sub_group_ 中
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_group_;
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/status", 10,
            std::bind(&FixedNode::status_callback, this, std::placeholders::_1),
            sub_opts);

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "正确设计节点启动 (C++)");
        RCLCPP_INFO(this->get_logger(), "  服务端 → service_group (独立)");
        RCLCPP_INFO(this->get_logger(), "  定时器 → timer_group (独立)");
        RCLCPP_INFO(this->get_logger(), "  订阅   → sub_group (独立)");
        RCLCPP_INFO(this->get_logger(), "  使用 MultiThreadedExecutor → 不会死锁");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    void service_callback(
        const std::shared_ptr<example_interfaces::srv::Trigger::Request> /*request*/,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[服务] 收到请求，处理中...");
        std::this_thread::sleep_for(500ms);
        call_count_++;
        response->success = true;
        response->message = "服务处理完成，第 " + std::to_string(call_count_) + " 次调用";
    }

    void timer_callback()
    {
        call_count_++;
        RCLCPP_INFO(this->get_logger(),
            "[定时器] 调用服务 (第%d次)...", call_count_);

        // 使用 async_send_request 异步调用
        // 服务端在不同的回调组中，可以并发执行
        auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
        auto future = client_->async_send_request(request);

        // 注意：不要在这里同步等待 future 的结果！
        // 如果需要结果，使用 spin_until_future_complete 或回调
        RCLCPP_INFO(this->get_logger(), "[定时器] 服务调用已发送");
    }

    void status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[订阅] 状态: %s", msg->data.c_str());
    }

    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::CallbackGroup::SharedPtr sub_group_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_;
    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    int call_count_;
};


// ================================================================
// 最佳实践版本 —— 避免回调中调用自己的服务
// ================================================================

class BestPracticeNode : public rclcpp::Node
{
public:
    BestPracticeNode() : Node("best_practice_node")
    {
        // 只需要一个回调组
        group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        timer_ = this->create_wall_timer(
            2s,
            std::bind(&BestPracticeNode::timer_callback, this),
            group_);

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "最佳实践节点启动 (C++)");
        RCLCPP_INFO(this->get_logger(), "  不在回调中调用自己的服务");
        RCLCPP_INFO(this->get_logger(), "  而是直接调用内部方法");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    /**
     * 核心逻辑封装在普通方法中（而非服务回调中）
     * ✅ 最佳实践：直接调用方法，避免服务调用的开销和死锁风险
     */
    std::pair<bool, std::string> do_something()
    {
        RCLCPP_INFO(this->get_logger(), "[核心逻辑] 执行处理...");
        std::this_thread::sleep_for(500ms);
        RCLCPP_INFO(this->get_logger(), "[核心逻辑] 处理完成");
        return {true, "处理成功"};
    }

    void timer_callback()
    {
        auto [success, message] = do_something();
        RCLCPP_INFO(this->get_logger(),
            "[定时器] 结果: success=%s, msg=\"%s\"",
            success ? "true" : "false", message.c_str());
    }

    rclcpp::CallbackGroup::SharedPtr group_;
    rclcpp::TimerBase::SharedPtr timer_;
};


// ================================================================
// 主函数 —— 根据命令行参数选择模式
// ================================================================

void print_usage(const char* program)
{
    std::cout << "\n用法: " << program << " [模式]\n"
              << "  1 - 死锁演示（MultiThreadedExecutor + 同组服务调用）\n"
              << "  2 - 正确版本（MultiThreadedExecutor + 独立回调组）\n"
              << "  3 - 最佳实践（直接方法调用，默认）\n" << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int mode = 3;  // 默认模式3
    if (argc > 1) {
        try {
            mode = std::stoi(argv[1]);
        } catch (...) {
            print_usage(argv[0]);
            rclcpp::shutdown();
            return 1;
        }
    }

    switch (mode) {
        case 1: {
            // 死锁演示
            auto node = std::make_shared<DeadlockNode>();
            rclcpp::executors::MultiThreadedExecutor executor(
                rclcpp::ExecutorOptions(), 4);
            executor.add_node(node);
            try { executor.spin(); } catch (...) {}
            break;
        }
        case 2: {
            // 正确版本
            auto node = std::make_shared<FixedNode>();
            rclcpp::executors::MultiThreadedExecutor executor(
                rclcpp::ExecutorOptions(), 4);
            executor.add_node(node);
            try { executor.spin(); } catch (...) {}
            break;
        }
        default: {
            // 最佳实践
            auto node = std::make_shared<BestPracticeNode>();
            rclcpp::spin(node);
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}
