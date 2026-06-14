/**
 * @file 07_action_lifecycle_server.cpp
 * @brief LifecycleNode 作为 Action Server —— 按状态启停 Action 服务
 *
 * 知识点：
 * - LifecycleNode 与 rclcpp_action 的配合
 * - on_configure 中创建 Action Server
 * - on_activate / on_deactivate 控制是否接受新目标
 * - on_cleanup / on_shutdown 销毁 Action Server
 * - 生命周期状态对 Action 行为的影响
 *
 * 生命周期与 Action 的关系：
 *   Unconfigured → 不提供服务
 *   Inactive     → 服务存在但拒绝所有新目标（可查询、可取消旧目标）
 *   Active       → 正常接受和执行目标
 *   Finalized    → 服务已销毁
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/count_up.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <mutex>
#include <thread>
#include <atomic>

using CountUp = action_learning_cpp::action::CountUp;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleActionServer()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_action_server"),
          accepting_goals_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== Lifecycle Action Server created ===");
    }

    // ── on_configure：创建 Action Server ──
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "[on_configure] Creating Action Server...");

        // 在 configure 阶段创建 Action Server
        // 此时服务端点已注册，但不接受目标
        action_server_ = rclcpp_action::create_server<CountUp>(
            this->shared_from_this(),
            "count_up",
            std::bind(&LifecycleActionServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&LifecycleActionServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&LifecycleActionServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[on_configure] Action Server created (not accepting goals yet)");
        return CallbackReturn::SUCCESS;
    }

    // ── on_activate：开始接受目标 ──
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "[on_activate] Start accepting goals");
        accepting_goals_ = true;
        return CallbackReturn::SUCCESS;
    }

    // ── on_deactivate：停止接受新目标 ──
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "[on_deactivate] Stop accepting new goals");
        accepting_goals_ = false;
        return CallbackReturn::SUCCESS;
    }

    // ── on_cleanup：销毁 Action Server ──
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "[on_cleanup] Destroying Action Server");
        action_server_.reset();
        return CallbackReturn::SUCCESS;
    }

    // ── on_shutdown：最终清理 ──
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "[on_shutdown] Shutting down");
        accepting_goals_ = false;
        action_server_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const CountUp::Goal> goal)
    {
        (void)uuid;

        // ── 根据生命周期状态决定是否接受目标 ──
        if (!accepting_goals_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Goal rejected: node not active (state=%s)",
                        this->get_current_state().label().c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->target <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "Rejected: target <= 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted: target = %ld", goal->target);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel accepted");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle)
    {
        std::thread{std::bind(&LifecycleActionServer::execute, this,
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

        RCLCPP_INFO(this->get_logger(), "Executing: target = %ld", goal->target);

        rclcpp::Rate loop_rate(1);

        for (int64_t i = 0; i <= goal->target; ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->final_count = i;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Canceled, current: %ld", i);
                return;
            }

            // 检查节点是否仍在 Active 状态
            if (!accepting_goals_)
            {
                // 节点被 deactivate，中止当前目标
                RCLCPP_WARN(this->get_logger(),
                            "Node deactivated, aborting current goal");
                result->final_count = i;
                goal_handle->abort(result);
                return;
            }

            feedback->current_count = i;
            feedback->progress_percent =
                static_cast<float>(i) / static_cast<float>(goal->target) * 100.0f;
            goal_handle->publish_feedback(feedback);

            result->sequence.push_back(i);
            loop_rate.sleep();
        }

        result->final_count = goal->target;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
                    "Completed! final = %ld", result->final_count);
    }

    rclcpp_action::Server<CountUp>::SharedPtr action_server_;
    std::atomic<bool> accepting_goals_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleActionServer>();

    // ⚠️ LifecycleNode 必须使用 executor 模式
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
