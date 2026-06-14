/**
 * @file action_client_basic.cpp
 * @brief Action Client 基础 —— 斐波那契数列客户端
 *
 * 知识点：
 * - rclcpp_action::create_client() 创建 Action Client
 * - wait_for_action_server() 等待服务端上线
 * - async_send_goal() 发送目标
 * - SendGoalOptions 的三个回调：
 *     goal_response_callback —— 服务端接受/拒绝目标的回调
 *     feedback_callback      —— 收到反馈的回调
 *     result_callback        —— 收到最终结果的回调
 *
 * 运行（先启动 action_server_basic）：
 *   ros2 run action_learning_cpp action_client_basic
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/fibonacci.hpp>
#include <chrono>
#include <sstream>
#include <memory>

using Fibonacci = action_learning_cpp::action::Fibonacci;
using namespace std::chrono_literals;

class FibonacciClient : public rclcpp::Node
{
public:
    FibonacciClient() : Node("fibonacci_client")
    {
        // ================================================================
        // 创建 Action Client
        //    参数：
        //    1. 节点指针
        //    2. Action 名称（必须与服务端一致）
        // ================================================================
        client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

        RCLCPP_INFO(this->get_logger(), "=== Fibonacci Action Client created ===");

        // 等待服务端上线
        if (!client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Action Server /fibonacci not available, exiting");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Action Server is available, sending goal...");

        // 发送目标：计算前 10 项斐波那契数列
        send_goal(10);
    }

private:
    // 将数列格式转为字符串，方便日志输出
    std::string format_sequence(const std::vector<int64_t> &seq)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < seq.size(); ++i)
        {
            if (i > 0)
                oss << ", ";
            oss << seq[i];
        }
        oss << "]";
        return oss.str();
    }

    // ── 成员函数回调1: goal_response_callback ──
    // 服务端接受或拒绝目标时触发
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Goal accepted, waiting for execution...");
        }
    }

    // ── 成员函数回调2: feedback_callback ──
    // 服务端发布反馈时触发
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Feedback received: %s",
                    format_sequence(feedback->current_sequence).c_str());
    }

    // ── 成员函数回调3: result_callback ──
    // 目标执行完成时触发（成功/失败/取消）
    void result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
                        "Goal succeeded! Fibonacci sequence: %s",
                        format_sequence(result.result->sequence).c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal aborted!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),
                        "Goal canceled, partial sequence: %s",
                        format_sequence(result.result->sequence).c_str());
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    }

    void send_goal(int64_t order)
    {
        // ── 创建目标消息 ──
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = order;

        // ================================================================
        // 配置 SendGoalOptions —— 三个核心回调
        //
        // Action Client 的通信流程：
        //   Client                     Server
        //     |--- Goal Request -------->|     goal_response_callback 触发
        //     |<-- Accept/Reject --------|
        //     |                          |
        //     |--- Result Request ------>|     阻塞等待 goal 执行完成
        //     |                          |
        //     |<-- Feedback -------------|     feedback_callback 触发（多次）
        //     |<-- Feedback -------------|
        //     |                          |
        //     |<-- Result Response ------|     result_callback 触发
        // ================================================================
        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

        // ── 绑定成员函数作为回调 ──
        // 方法1: 使用 std::bind
        send_goal_options.goal_response_callback =
            std::bind(&FibonacciClient::goal_response_callback, this, std::placeholders::_1);

        send_goal_options.feedback_callback = 
            std::bind(&FibonacciClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        // 方法2: 使用 lambda 包装成员函数（更现代的风格）
        send_goal_options.result_callback = 
            [this](const auto &result)
            {
                this->result_callback(result);
            };

        // ── 发送目标 ──
        RCLCPP_INFO(this->get_logger(), "Sending goal, order: %ld", order);
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<Fibonacci>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}