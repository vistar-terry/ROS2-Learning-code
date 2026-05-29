/**
 * @file 01_action_server_basic.cpp
 * @brief Action Server 基础 —— 最简单的 Action 服务端
 *
 * 知识点：
 * - rclcpp_action::create_server() 创建 Action Server
 * - handle_goal：决定接受/拒绝目标
 * - handle_cancel：决定允许/拒绝取消
 * - handle_accepted：启动执行线程
 * - execute：执行逻辑 + 发布 Feedback + 返回 Result
 *
 * 运行：
 *   ros2 run action_learning_cpp 01_action_server_basic
 *   ros2 action send_goal /count_up action_learning_cpp/action/CountUp "{target: 5}"
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/count_up.hpp>
#include <thread>

using CountUp = action_learning_cpp::action::CountUp;

class CountUpServer : public rclcpp::Node {
public:
    CountUpServer() : Node("count_up_server") {
        // ================================================================
        // 创建 Action Server
        //    参数：
        //    1. 节点指针
        //    2. Action 名称（客户端通过此名称连接）
        //    3. handle_goal   —— 收到目标请求时的回调
        //    4. handle_cancel —— 收到取消请求时的回调
        //    5. handle_accepted —— 目标被接受后的回调（开始执行）
        // ================================================================
        action_server_ = rclcpp_action::create_server<CountUp>(
            this,
            "count_up",
            std::bind(&CountUpServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&CountUpServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&CountUpServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== CountUp Action Server 已启动 ===");
        RCLCPP_INFO(this->get_logger(), "Action 名称: /count_up");
        RCLCPP_INFO(this->get_logger(), "等待客户端发送目标...");
    }

private:
    // ================================================================
    // handle_goal —— 目标请求回调
    //
    // 触发时机：客户端调用 async_send_goal() 时
    // 参数：
    //   uuid - 目标的唯一标识符（可用于日志追踪）
    //   goal - 客户端发送的目标内容（只读）
    //
    // 返回值：
    //   GoalResponse::ACCEPT_AND_EXECUTE — 接受目标，进入执行
    //   GoalResponse::REJECT          — 拒绝目标
    //   GoalResponse::ACCEPT_AND_DEFER — 接受但延迟执行（高级用法）
    // ================================================================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const CountUp::Goal> goal) {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(), "收到目标请求: target = %ld", goal->target);

        // ── 目标合法性校验 ──
        if (goal->target <= 0) {
            RCLCPP_WARN(this->get_logger(), "拒绝目标: target 必须大于 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "接受目标: target = %ld", goal->target);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // ================================================================
    // handle_cancel —— 取消请求回调
    //
    // 触发时机：客户端调用 async_cancel_goal() 时
    // 参数：
    //   goal_handle - 正在执行的目标句柄
    //
    // 返回值：
    //   CancelResponse::ACCEPT — 允许取消
    //   CancelResponse::REJECT — 拒绝取消（目标继续执行）
    // ================================================================
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "收到取消请求，允许取消");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // ================================================================
    // handle_accepted —— 目标被接受后的回调
    //
    // 触发时机：handle_goal 返回 ACCEPT 后
    // 作用：启动执行逻辑
    //
    // ⚠️ 重要：不能在此回调中直接执行耗时操作！
    //    此回调在 executor 线程中运行，阻塞会导致无法处理其他回调。
    //    必须在新线程中执行耗时逻辑。
    // ================================================================
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle) {
        // 在新线程中执行，避免阻塞 executor
        std::thread{
            std::bind(&CountUpServer::execute, this, std::placeholders::_1),
            goal_handle
        }.detach();
    }

    // ================================================================
    // execute —— 目标执行逻辑（在独立线程中运行）
    //
    // GoalHandle 的三种终止方式：
    //   goal_handle->succeed(result)  — 执行成功
    //   goal_handle->abort(result)    — 执行失败（异常终止）
    //   goal_handle->canceled(result) — 被取消
    //
    // ⚠️ 每个 GoalHandle 只能调用一次终止方法！
    // ⚠️ 调用终止方法后不能再 publish_feedback！
    // ================================================================
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUp>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "开始执行目标...");

        // 从 GoalHandle 获取目标
        auto goal = goal_handle->get_goal();
        // 创建 Result 和 Feedback 对象
        auto result = std::make_shared<CountUp::Result>();
        auto feedback = std::make_shared<CountUp::Feedback>();

        rclcpp::Rate loop_rate(1);  // 1Hz — 每秒计数一次

        for (int64_t i = 0; i <= goal->target; ++i) {
            // ── 检查是否被取消 ──
            if (goal_handle->is_canceling()) {
                result->final_count = i;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(),
                    "目标被取消，当前值: %ld", i);
                return;
            }

            // ── 发布 Feedback ──
            feedback->current_count = i;
            feedback->progress_percent =
                static_cast<float>(i) / static_cast<float>(goal->target) * 100.0f;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "进度: %ld/%ld (%.1f%%)",
                i, goal->target, feedback->progress_percent);

            // 保存到结果序列
            result->sequence.push_back(i);

            loop_rate.sleep();
        }

        // ── 执行成功 ──
        result->final_count = goal->target;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
            "✓ 目标完成! 最终值: %ld", result->final_count);
    }

    rclcpp_action::Server<CountUp>::SharedPtr action_server_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUpServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
