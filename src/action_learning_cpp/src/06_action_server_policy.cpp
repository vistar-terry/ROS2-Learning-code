/**
 * @file 06_action_server_policy.cpp
 * @brief Action Server 目标策略 —— 接受/拒绝/抢占
 *
 * 知识点：
 * - 目标验证与拒绝策略
 * - 抢占（Preempt）：新目标替换正在执行的目标
 * - 三种策略模式：REJECT（拒绝新目标）/ PREEMPT（抢占旧目标）
 * - handle_goal 中的并发控制
 *
 * ⚠️ 抢占的正确实现：
 *   不能在 handle_goal 中直接调用 goal_handle->canceled()
 *   正确做法：execute 函数中检测自己是否仍是当前目标，
 *   如果不是则自行取消（被新目标抢占）
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/count_up.hpp>
#include <mutex>
#include <thread>

using CountUp = action_learning_cpp::action::CountUp;

// ── 目标策略枚举 ──
enum class GoalPolicy
{
    REJECT,  // 忙碌时拒绝新目标
    PREEMPT, // 忙碌时抢占（取消旧目标，执行新目标）
};

class PolicyServer : public rclcpp::Node
{
public:
    PolicyServer() : Node("policy_server"),
                     current_goal_handle_(nullptr), policy_(GoalPolicy::PREEMPT)
    {
        // 通过参数配置策略
        this->declare_parameter("policy", std::string("preempt"));
        auto policy_str = this->get_parameter("policy").as_string();
        if (policy_str == "reject")
        {
            policy_ = GoalPolicy::REJECT;
            RCLCPP_INFO(this->get_logger(), "策略: REJECT（忙碌时拒绝新目标）");
        }
        else
        {
            policy_ = GoalPolicy::PREEMPT;
            RCLCPP_INFO(this->get_logger(), "策略: PREEMPT（忙碌时抢占旧目标）");
        }

        action_server_ = rclcpp_action::create_server<CountUp>(
            this,
            "count_up",
            std::bind(&PolicyServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&PolicyServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&PolicyServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== Policy Server 已启动 ===");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const CountUp::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(),
                    "收到目标: target = %ld", goal->target);

        if (goal->target <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "拒绝: target <= 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::lock_guard<std::mutex> lock(goal_mutex_);

        if (current_goal_handle_ && current_goal_handle_->is_active())
        {
            switch (policy_)
            {
            // ── 策略1: REJECT ──
            case GoalPolicy::REJECT:
                RCLCPP_WARN(this->get_logger(),
                            "REJECT: 正在执行目标 (target=%ld)，拒绝新目标",
                            current_goal_handle_->get_goal()->target);
                return rclcpp_action::GoalResponse::REJECT;

            // ── 策略2: PREEMPT ──
            //    接受新目标，旧目标会在 execute 中检测到被抢占并自行取消
            case GoalPolicy::PREEMPT:
                RCLCPP_INFO(this->get_logger(),
                            "PREEMPT: 将抢占当前目标 (target=%ld)，接受新目标",
                            current_goal_handle_->get_goal()->target);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "允许取消");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        // 替换当前目标 → 旧目标的 execute 线程会检测到
        // current_goal_handle_ 已不再指向自己，从而自行取消
        current_goal_handle_ = goal_handle;
        std::thread{std::bind(&PolicyServer::execute, this,
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

        RCLCPP_INFO(this->get_logger(),
                    "执行目标: target = %ld", goal->target);

        rclcpp::Rate loop_rate(1);

        for (int64_t i = 0; i <= goal->target; ++i)
        {
            // ── 检查是否被抢占（PREEMPT 策略）──
            //    如果 current_goal_handle_ 不再指向自己，
            //    说明有新目标已经取代了当前目标
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                if (current_goal_handle_ != goal_handle)
                {
                    result->final_count = i;
                    result->sequence.push_back(i);
                    goal_handle->canceled(result);
                    RCLCPP_WARN(this->get_logger(),
                                "目标被抢占 (target=%ld), 当前: %ld",
                                goal->target, i);
                    return;
                }
            }

            // ── 检查是否被客户端取消 ──
            if (goal_handle->is_canceling())
            {
                result->final_count = i;
                goal_handle->canceled(result);
                RCLCPP_WARN(this->get_logger(),
                            "目标被客户端取消 (target=%ld), 当前: %ld",
                            goal->target, i);
                return;
            }

            feedback->current_count = i;
            feedback->progress_percent =
                static_cast<float>(i) / static_cast<float>(goal->target) * 100.0f;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),
                        "执行中: %ld/%ld (%.1f%%)", i, goal->target, feedback->progress_percent);

            result->sequence.push_back(i);
            loop_rate.sleep();
        }

        result->final_count = goal->target;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
                    "✓ 完成! final = %ld", result->final_count);

        // 清理
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (current_goal_handle_ == goal_handle)
        {
            current_goal_handle_ = nullptr;
        }
    }

    rclcpp_action::Server<CountUp>::SharedPtr action_server_;
    std::mutex goal_mutex_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> current_goal_handle_;
    GoalPolicy policy_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolicyServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
