/**
 * @file 05_action_cancel_demo.cpp
 * @brief Action 取消演示 —— 服务端 + 客户端（同一进程）
 *
 * 知识点：
 * - async_cancel_goal() 主动取消目标
 * - 服务端 is_canceling() 检测取消
 * - 服务端 goal_handle->canceled(result) 优雅终止
 * - 取消后的资源清理
 * - 定时器触发取消
 *
 * 运行：
 *   ros2 run action_learning_cpp 05_action_cancel_demo
 *   （自包含：服务端+客户端在同一进程中）
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/count_up.hpp>
#include <thread>
#include <chrono>

using CountUp = action_learning_cpp::action::CountUp;
using namespace std::chrono_literals;

// ================================================================
// 服务端：支持优雅取消
// ================================================================
class CancelableServer : public rclcpp::Node
{
public:
    CancelableServer() : Node("cancelable_server")
    {
        action_server_ = rclcpp_action::create_server<CountUp>(
            this,
            "count_up",
            std::bind(&CancelableServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&CancelableServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&CancelableServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[Server] Cancelable Action Server started");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const CountUp::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "[Server] Goal accepted: target = %ld", goal->target);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "[Server] Cancel request received -> accepted");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        std::thread{std::bind(&CancelableServer::execute, this,
                              std::placeholders::_1),
                    goal_handle}
            .detach();
    }

    // ── 可取消的执行逻辑 ──
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<CountUp::Result>();
        auto feedback = std::make_shared<CountUp::Feedback>();

        RCLCPP_INFO(this->get_logger(),
                    "[Server] Executing (target=%ld)...", goal->target);

        rclcpp::Rate loop_rate(1);

        for (int64_t i = 0; i <= goal->target; ++i)
        {
            // ── 关键：每一步都检查是否被取消 ──
            if (goal_handle->is_canceling())
            {
                // ── 优雅停止：清理资源、保存中间结果 ──
                RCLCPP_INFO(this->get_logger(),
                            "[Server] Execution canceled, cleaning up resources...");

                // 模拟资源清理（关闭文件、停止硬件等）
                std::this_thread::sleep_for(200ms);

                result->final_count = i;
                result->sequence.push_back(i);
                goal_handle->canceled(result);

                RCLCPP_INFO(this->get_logger(),
                            "[Server] Canceled, final value: %ld", i);
                return;
            }

            feedback->current_count = i;
            feedback->progress_percent =
                static_cast<float>(i) / static_cast<float>(goal->target) * 100.0f;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),
                        "[Server] Progress: %ld/%ld", i, goal->target);

            result->sequence.push_back(i);
            loop_rate.sleep();
        }

        result->final_count = goal->target;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
                    "[Server] Completed! final = %ld", result->final_count);
    }

    rclcpp_action::Server<CountUp>::SharedPtr action_server_;
};

// ================================================================
// 客户端：发送目标后定时取消
// ================================================================
class CancelClient : public rclcpp::Node
{
public:
    CancelClient() : Node("cancel_client")
    {
        client_ = rclcpp_action::create_client<CountUp>(this, "count_up");

        RCLCPP_INFO(this->get_logger(), "[Client] Waiting for Action Server...");

        if (!client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "[Client] Server not available");
            return;
        }

        // ── 发送一个长时间目标 ──
        auto goal_msg = CountUp::Goal();
        goal_msg.target = 20; // 20秒才能完成

        auto send_goal_options =
            rclcpp_action::Client<CountUp>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const rclcpp_action::ClientGoalHandle<CountUp>::SharedPtr &
                       goal_handle)
        {
            if (goal_handle)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Client] Goal accepted, will cancel in 3s...");
                // 保存 goal_handle 用于后续取消
                current_goal_handle_ = goal_handle;

                // ── 3秒后自动取消 ──
                cancel_timer_ = this->create_wall_timer(3s, [this]()
                                                        {
                                                            RCLCPP_INFO(this->get_logger(),
                                                                        "[Client] >>> Sending cancel request!");
                                                            client_->async_cancel_goal(current_goal_handle_);
                                                            cancel_timer_->cancel(); // 只触发一次
                                                        });
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[Client] Goal rejected");
            }
        };

        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<CountUp>::SharedPtr,
                   const std::shared_ptr<const CountUp::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[Client] Feedback: %ld (%.1f%%)",
                        feedback->current_count, feedback->progress_percent);
        };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<CountUp>::WrappedResult &
                       result)
        {
            if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_WARN(this->get_logger(),
                            "[Client] Goal canceled! final = %ld",
                            result.result->final_count);
            }
            else if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Client] Goal succeeded! final = %ld",
                            result.result->final_count);
            }
        };

        RCLCPP_INFO(this->get_logger(),
                    "[Client] Sending goal: target = 20 (will cancel in 3s)");
        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<CountUp>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<CountUp>::SharedPtr current_goal_handle_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
};

// ================================================================
// main：同一进程中运行 Server + Client
// ================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 使用 MultiThreadedExecutor 让 Server 和 Client 并发运行
    rclcpp::executors::MultiThreadedExecutor executor;

    auto server = std::make_shared<CancelableServer>();
    auto client = std::make_shared<CancelClient>();

    executor.add_node(server);
    executor.add_node(client);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
