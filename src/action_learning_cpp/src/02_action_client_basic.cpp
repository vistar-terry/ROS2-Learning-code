/**
 * @file 02_action_client_basic.cpp
 * @brief Action Client 基础 —— 最简单的 Action 客户端
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
 * 运行（先启动 01_action_server_basic）：
 *   ros2 run action_learning_cpp 02_action_client_basic
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/count_up.hpp>
#include <chrono>

using CountUp = action_learning_cpp::action::CountUp;
using namespace std::chrono_literals;

class CountUpClient : public rclcpp::Node {
public:
    CountUpClient() : Node("count_up_client") {
        // ================================================================
        // 创建 Action Client
        //    参数：
        //    1. 节点指针
        //    2. Action 名称（必须与服务端一致）
        // ================================================================
        client_ = rclcpp_action::create_client<CountUp>(this, "count_up");

        RCLCPP_INFO(this->get_logger(), "=== CountUp Action Client 已创建 ===");

        // 等待服务端上线
        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(),
                "Action Server /count_up 未上线，退出");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Action Server 已上线，发送目标...");

        // 发送目标
        send_goal(10);
    }

private:
    void send_goal(int64_t target) {
        // ── 创建目标消息 ──
        auto goal_msg = CountUp::Goal();
        goal_msg.target = target;

        // ================================================================
        // 配置 SendGoalOptions —— 三个核心回调
        //
        // Action Client 的通信流程：
        //   Client                     Server
        //     |--- Goal Request -------->|     goal_response_callback 触发
        //     |<-- Accept/Reject --------|
        //     |                          |
        //     |<-- Feedback -------------|     feedback_callback 触发（多次）
        //     |<-- Feedback -------------|
        //     |                          |
        //     |<-- Result ---------------|     result_callback 触发
        // ================================================================
        auto send_goal_options = rclcpp_action::Client<CountUp>::SendGoalOptions();

        // ── 回调1: goal_response_callback ──
        //    服务端接受或拒绝目标时触发
        //    参数：GoalHandle 的 SharedFuture
        send_goal_options.goal_response_callback =
            [this](const rclcpp_action::ClientGoalHandle<CountUp>::SharedPtr&
                       goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "目标被拒绝!");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "目标被接受，等待执行...");
                }
            };

        // ── 回调2: feedback_callback ──
        //    服务端发布反馈时触发
        //    参数：GoalHandle, Feedback
        send_goal_options.feedback_callback =
            [this](
                rclcpp_action::ClientGoalHandle<CountUp>::SharedPtr,
                const std::shared_ptr<const CountUp::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(),
                    "收到反馈: current = %ld, progress = %.1f%%",
                    feedback->current_count, feedback->progress_percent);
            };

        // ── 回调3: result_callback ──
        //    目标执行完成时触发（成功/失败/取消）
        //    参数：WrappedResult（包含 result, code, goal_handle）
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<CountUp>::WrappedResult&
                       result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),
                        "✓ 目标成功! final_count = %ld",
                        result.result->final_count);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "目标被中止!");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "目标被取消");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "未知结果代码");
                    break;
                }
                // 结果中的序列
                RCLCPP_INFO(this->get_logger(),
                    "序列长度: %zu", result.result->sequence.size());
            };

        // ── 发送目标 ──
        RCLCPP_INFO(this->get_logger(), "发送目标: target = %ld", target);
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<CountUp>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUpClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
