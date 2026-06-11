/**
 * @file 03_action_server_feedback.cpp
 * @brief Action Server 进阶 —— 丰富的反馈与执行控制
 *
 * 知识点：
 * - 高频 Feedback 发布（10Hz）
 * - 执行进度精确计算
 * - 取消时的优雅停止与资源清理
 * - 多目标互斥（同一时刻只执行一个目标）
 * - GoalHandle 状态查询
 *
 * 与 01 的区别：
 * - 01 是最简版，1Hz 低频反馈
 * - 本版更接近生产代码：高频反馈、互斥执行、优雅取消
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/count_up.hpp>
#include <mutex>
#include <thread>

using CountUp = action_learning_cpp::action::CountUp;

class FeedbackServer : public rclcpp::Node
{
public:
    FeedbackServer() : Node("feedback_server"), current_goal_handle_(nullptr)
    {
        action_server_ = rclcpp_action::create_server<CountUp>(
            this,
            "count_up",
            std::bind(&FeedbackServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&FeedbackServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&FeedbackServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== Feedback Server 已启动 ===");
        RCLCPP_INFO(this->get_logger(), "特点: 10Hz反馈、互斥执行、优雅取消");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const CountUp::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(), "收到目标: target = %ld", goal->target);

        if (goal->target <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "拒绝: target ≤ 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // ── 互斥检查：如果正在执行，拒绝新目标 ──
        //    生产环境中可选择：抢占当前目标 / 排队 / 拒绝
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (current_goal_handle_ && current_goal_handle_->is_active())
        {
            RCLCPP_WARN(this->get_logger(),
                        "拒绝: 正在执行另一个目标 (current target = %ld)",
                        current_goal_handle_->get_goal()->target);
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "允许取消目标");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = goal_handle;
        std::thread{std::bind(&FeedbackServer::execute, this,
                              std::placeholders::_1),
                    goal_handle}
            .detach();
    }

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<CountUp::Result>();
        auto feedback = std::make_shared<CountUp::Feedback>();

        RCLCPP_INFO(this->get_logger(), "开始执行: target = %ld", goal->target);

        // ── 高频反馈：10Hz ──
        //    实际机器人控制中通常需要 10~100Hz 的反馈频率
        rclcpp::Rate loop_rate(10);

        int64_t current = 0;
        // 每个大步骤耗时 1 秒，10Hz 反馈意味着每步发布 10 次 feedback
        int64_t steps = goal->target;
        int sub_steps_per_sec = 10; // 10Hz → 每秒10个子步骤

        for (int64_t step = 0; step <= steps; ++step)
        {
            for (int sub = 0; sub < sub_steps_per_sec; ++sub)
            {
                // ── 检查取消 ──
                if (goal_handle->is_canceling())
                {
                    result->final_count = current;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(),
                                "目标已取消, 当前值: %.1f",
                                static_cast<double>(current));
                    cleanup_goal();
                    return;
                }

                // ── 计算当前插值 ──
                double fraction = static_cast<double>(sub) / sub_steps_per_sec;
                current = step + static_cast<int64_t>(
                                     fraction * 1.0); // 插值到下一个整数

                // ── 发布 Feedback ──
                feedback->current_count = step;
                feedback->progress_percent =
                    static_cast<float>(step + fraction) /
                    static_cast<float>(steps) * 100.0f;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            // 保存到结果序列
            result->sequence.push_back(step);
        }

        // ── 成功完成 ──
        result->final_count = goal->target;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
                    "✓ 完成! final_count = %ld", result->final_count);
        cleanup_goal();
    }

    void cleanup_goal()
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = nullptr;
    }

    rclcpp_action::Server<CountUp>::SharedPtr action_server_;
    std::mutex goal_mutex_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> current_goal_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeedbackServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
