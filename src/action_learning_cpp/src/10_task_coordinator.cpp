/**
 * @file 10_task_coordinator.cpp
 * @brief 多 Action 协调器 —— 同时管理机械臂和导航
 *
 * 知识点：
 * - 同时向多个 Action Server 发送目标
 * - 并行 Action 执行与结果汇聚
 * - 任务编排：顺序 vs 并行 vs 混合模式
 * - 超时处理
 * - 任一失败时的处理策略
 *
 * 场景描述：
 *   机器人执行一个复合任务：
 *   1. 并行启动：导航到目标位置 + 机械臂准备
 *   2. 导航完成后：机械臂执行抓取
 *   3. 抓取完成后：导航返回 + 机械臂收回
 *
 * 这模拟了真实机器人工作流：
 *   "导航到货架 → 抓取物品 → 返回起点"
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/move_arm.hpp>
#include <action_learning_cpp/action/navigate.hpp>
#include <chrono>
#include <future>
#include <map>
#include <string>

using MoveArm = action_learning_cpp::action::MoveArm;
using Navigate = action_learning_cpp::action::Navigate;
using namespace std::chrono_literals;

class TaskCoordinator : public rclcpp::Node
{
public:
    TaskCoordinator() : Node("task_coordinator")
    {
        arm_client_ = rclcpp_action::create_client<MoveArm>(this, "move_arm");
        nav_client_ = rclcpp_action::create_client<Navigate>(this, "navigate");

        RCLCPP_INFO(this->get_logger(), "=== Task Coordinator created ===");

        // 等待两个 Action Server 上线
        RCLCPP_INFO(this->get_logger(), "Waiting for Action Server...");
        bool arm_ok = arm_client_->wait_for_action_server(5s);
        bool nav_ok = nav_client_->wait_for_action_server(5s);

        if (!arm_ok || !nav_ok)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Not all Action Servers available (arm=%d, nav=%d)", arm_ok, nav_ok);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "All Action Servers are available");

        // 延迟 2 秒后启动任务流程
        timer_ = this->create_wall_timer(2s, [this]()
                                         {
            timer_->cancel();
            run_task_pipeline(); });
    }

private:
    // ================================================================
    // 任务流水线
    //
    //   Phase 1 (并行):  导航到目标 ─┐
    //                    机械臂准备 ─┤
    //                                ├─→ 等待全部完成
    //   Phase 2 (顺序):  机械臂抓取 ─┘
    //
    //   Phase 3 (并行):  导航返回 ─┐
    //                    机械臂收回 ─┤─→ 等待全部完成
    //
    // ================================================================
    void run_task_pipeline()
    {
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "====== Robot compound task pipeline started ======");
        RCLCPP_INFO(this->get_logger(), " ");

        // ── Phase 1: 并行启动 ──
        RCLCPP_INFO(this->get_logger(), "--- Phase 1: Parallel start ---");

        // 同时发送导航和机械臂准备目标
        auto nav_future = send_nav_goal(3.0, 4.0, 1.0, "Navigate to target position");
        auto arm_future = send_arm_goal(1.57, 1.0, "Arm ready pose (90 deg)");

        // 等待两个并行任务都完成
        RCLCPP_INFO(this->get_logger(), "Waiting for Phase 1 parallel tasks to complete...");
        bool nav_ok = wait_for_result(nav_future, "Navigation");
        bool arm_ok = wait_for_result(arm_future, "Arm");

        if (!nav_ok || !arm_ok)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Phase 1 failed (nav=%d, arm=%d), aborting pipeline", nav_ok, arm_ok);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Phase 1 completed!");

        // ── Phase 2: 顺序执行抓取 ──
        RCLCPP_INFO(this->get_logger(), "--- Phase 2: Arm grasping ---");

        auto grasp_future = send_arm_goal(0.5, 0.5, "Grasp object (28.6 deg)");
        bool grasp_ok = wait_for_result(grasp_future, "Grasp");

        if (!grasp_ok)
        {
            RCLCPP_ERROR(this->get_logger(), "Phase 2 failed, aborting pipeline");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Phase 2 completed!");

        // ── Phase 3: 并行返回 ──
        RCLCPP_INFO(this->get_logger(), "--- Phase 3: Parallel return ---");

        auto return_nav_future = send_nav_goal(0.0, 0.0, 1.5, "Navigate back to start");
        auto return_arm_future = send_arm_goal(0.0, 1.0, "Arm return to home pose");

        bool return_nav_ok = wait_for_result(return_nav_future, "Return nav");
        bool return_arm_ok = wait_for_result(return_arm_future, "Return arm");

        RCLCPP_INFO(this->get_logger(), " ");
        if (return_nav_ok && return_arm_ok)
        {
            RCLCPP_INFO(this->get_logger(), "====== All task pipeline completed! ======");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "====== Pipeline partially failed ======");
        }
    }

    // ── 发送导航目标，返回 result future ──
    std::shared_future<rclcpp_action::ClientGoalHandle<Navigate>::WrappedResult>
    send_nav_goal(double x, double y, double speed, const std::string &desc)
    {
        auto goal_msg = Navigate::Goal();
        goal_msg.target_x = x;
        goal_msg.target_y = y;
        goal_msg.speed = speed;

        auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();

        options.goal_response_callback =
            [this, desc](const rclcpp_action::ClientGoalHandle<Navigate>::SharedPtr &
                             handle)
        {
            if (handle)
            {
                RCLCPP_INFO(this->get_logger(), "  [Nav] > %s — accepted", desc.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "  [Nav] X %s — rejected", desc.c_str());
            }
        };

        options.feedback_callback =
            [this, desc](
                rclcpp_action::ClientGoalHandle<Navigate>::SharedPtr,
                const std::shared_ptr<const Navigate::Feedback> fb)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "  [Nav] %s | (%.2f,%.2f) dist=%.2f %.0f%%",
                         desc.c_str(), fb->current_x, fb->current_y,
                         fb->distance_remaining, fb->progress_percent);
        };

        // ── 关键：使用 promise/future 模式获取异步结果 ──
        auto promise = std::make_shared<
            std::promise<rclcpp_action::ClientGoalHandle<Navigate>::WrappedResult>>();
        auto future = promise->get_future().share();

        options.result_callback =
            [this, desc, promise](
                const rclcpp_action::ClientGoalHandle<Navigate>::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(),
                            "  [Nav] ✓ %s | (%.2f,%.2f) %.2fs",
                            desc.c_str(), result.result->final_x,
                            result.result->final_y, result.result->elapsed_time);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "  [Nav] ! %s — failed", desc.c_str());
            }
            promise->set_value(result);
        };

        RCLCPP_INFO(this->get_logger(), "  [Nav] >>> Sending: %s", desc.c_str());
        nav_client_->async_send_goal(goal_msg, options);

        return future;
    }

    // ── 发送机械臂目标，返回 result future ──
    std::shared_future<rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult>
    send_arm_goal(double angle, double speed, const std::string &desc)
    {
        auto goal_msg = MoveArm::Goal();
        goal_msg.target_angle = angle;
        goal_msg.speed = speed;

        auto options = rclcpp_action::Client<MoveArm>::SendGoalOptions();

        options.goal_response_callback =
            [this, desc](const rclcpp_action::ClientGoalHandle<MoveArm>::SharedPtr &
                             handle)
        {
            if (handle)
            {
                RCLCPP_INFO(this->get_logger(), "  [Arm] > %s — accepted", desc.c_str());
            }
        };

        options.feedback_callback =
            [this, desc](
                rclcpp_action::ClientGoalHandle<MoveArm>::SharedPtr,
                const std::shared_ptr<const MoveArm::Feedback> fb)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "  [Arm] %s | %.2f rad (%.0f%%) %s",
                         desc.c_str(), fb->current_angle,
                         fb->progress_percent, fb->status.c_str());
        };

        auto promise = std::make_shared<
            std::promise<rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult>>();
        auto future = promise->get_future().share();

        options.result_callback =
            [this, desc, promise](
                const rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(),
                            "  [Arm] ✓ %s | %.2f rad %.2fs",
                            desc.c_str(), result.result->final_angle,
                            result.result->elapsed_time);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "  [Arm] ! %s — failed", desc.c_str());
            }
            promise->set_value(result);
        };

        RCLCPP_INFO(this->get_logger(), "  [Arm] >>> Sending: %s", desc.c_str());
        arm_client_->async_send_goal(goal_msg, options);

        return future;
    }

    // ── 等待结果（轮询方式，不阻塞 executor）──
    template <typename T>
    bool wait_for_result(std::shared_future<T> &future, const std::string &name)
    {
        auto timeout = 30s;
        auto start = std::chrono::steady_clock::now();

        while (rclcpp::ok())
        {
            auto status = future.wait_for(100ms);
            if (status == std::future_status::ready)
            {
                auto result = future.get();
                // 检查 WrappedResult 的 code
                return (result.code == rclcpp_action::ResultCode::SUCCEEDED);
            }
            // 检查超时
            if (std::chrono::steady_clock::now() - start > timeout)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "%s timed out (%lds)", name.c_str(),
                             timeout.count() / 1000000000L);
                return false;
            }
        }
        return false;
    }

    rclcpp_action::Client<MoveArm>::SharedPtr arm_client_;
    rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 协调器使用 SingleThreadedExecutor
    // 因为 wait_for_result 使用 wait_for(100ms) 轮询，
    // 不阻塞 executor（100ms 后会返回让 executor 处理其他回调）
    auto coordinator = std::make_shared<TaskCoordinator>();
    rclcpp::spin(coordinator);

    rclcpp::shutdown();
    return 0;
}
