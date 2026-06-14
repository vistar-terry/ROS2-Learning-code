/**
 * @file action_server_basic.cpp
 * @brief Action Server 基础 —— 斐波那契数列服务端
 *
 * 知识点：
 * - rclcpp_action::create_server() 创建 Action Server
 * - handle_goal：决定接受/拒绝目标
 * - handle_cancel：决定允许/拒绝取消
 * - handle_accepted：启动执行线程
 * - execute：执行逻辑 + 发布 Feedback + 返回 Result
 *
 * 功能：
 *   客户端指定 order（项数），服务端逐步计算斐波那契数列，
 *   每计算出一项就通过 Feedback 发布当前已生成的部分数列，
 *   最终通过 Result 返回完整的斐波那契数列。
 *
 * 运行：
 *   ros2 run action_learning_cpp action_server_basic
 *   ros2 action send_goal /fibonacci action_learning_cpp/action/Fibonacci "{order: 10}"
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/fibonacci.hpp>
#include <thread>

using Fibonacci = action_learning_cpp::action::Fibonacci;

class FibonacciServer : public rclcpp::Node
{
public:
    FibonacciServer() : Node("fibonacci_server")
    {
        // ================================================================
        // 创建 Action Server
        //    参数：
        //    1. 节点指针
        //    2. Action 名称（客户端通过此名称连接）
        //    3. handle_goal   —— 收到目标请求时的回调
        //    4. handle_cancel —— 收到取消请求时的回调
        //    5. handle_accepted —— 目标被接受后的回调（开始执行）
        // ================================================================
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&FibonacciServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&FibonacciServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== Fibonacci Action Server started ===");
        RCLCPP_INFO(this->get_logger(), "Action name: /fibonacci");
        RCLCPP_INFO(this->get_logger(), "Waiting for client to send goal...");
    }

private:
    // ================================================================
    // handle_goal —— 目标请求回调
    //
    // 触发时机：客户端调用 async_send_goal() 时
    // 参数：
    //   uuid - 目标请求会话的唯一标识符（可用于日志追踪）
    //   goal - 客户端发送的目标内容（只读）
    //
    // 返回值：
    //   GoalResponse::ACCEPT_AND_EXECUTE — 接受目标，进入执行
    //   GoalResponse::REJECT          — 拒绝目标
    //   GoalResponse::ACCEPT_AND_DEFER — 接受但延迟执行（高级用法）
    // ================================================================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(), "Received goal request with order: %ld", goal->order);

        // ── 目标合法性校验 ──
        // 斐波那契数列至少需要 1 项
        if (goal->order <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "Goal rejected: order must be > 0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted with order: %ld", goal->order);
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
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel request received, cancel accepted");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // ================================================================
    // handle_accepted —— 目标被接受后的回调
    //
    // 触发时机：handle_goal 返回 ACCEPT 后
    // 作用：启动执行逻辑
    //
    // 重要：不能在此回调中直接执行耗时操作！
    //    此回调在 executor 线程中运行，阻塞会导致无法处理其他回调。
    //    必须在新线程中执行耗时逻辑。
    // ================================================================
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        // 在新线程中执行，避免阻塞 executor
        std::thread{
            std::bind(&FibonacciServer::execute, this, std::placeholders::_1),
            goal_handle
        }.detach();
    }

    // ================================================================
    // execute —— 目标执行逻辑（在独立线程中运行）
    //
    // 斐波那契数列计算逻辑：
    //   F(0)=0, F(1)=1, F(n)=F(n-1)+F(n-2)
    //   order=1 → [0]
    //   order=2 → [0, 1]
    //   order=5 → [0, 1, 1, 2, 3]
    //   order=10 → [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
    //
    // GoalHandle 的三种终止方式：
    //   goal_handle->succeed(result)  — 执行成功
    //   goal_handle->abort(result)    — 执行失败（异常终止）
    //   goal_handle->canceled(result) — 被取消
    //
    // 注意：
    //   每个 GoalHandle 只能调用一次终止方法！
    //   调用终止方法后不能再 publish_feedback！
    // ================================================================
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");

        // 从 GoalHandle 获取目标
        auto goal = goal_handle->get_goal();
        // 创建 Result 和 Feedback 对象
        auto result = std::make_shared<Fibonacci::Result>();
        auto feedback = std::make_shared<Fibonacci::Feedback>();

        rclcpp::Rate loop_rate(1); // 1Hz — 每秒计算一项

        // 初始化斐波那契数列的前两项
        int64_t a = 0, b = 1;

        for (int64_t i = 0; i < goal->order; ++i)
        {
            // ── 检查是否被取消 ──
            if (goal_handle->is_canceling())
            {
                // 返回已计算出的部分数列
                result->sequence = feedback->current_sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(),
                            "Goal canceled, computed %ld/%ld terms",
                            i, goal->order);
                return;
            }

            // ── 计算当前项 ──
            int64_t current_value;
            if (i == 0)
            {
                current_value = 0;
            }
            else if (i == 1)
            {
                current_value = 1;
            }
            else
            {
                current_value = a + b;
                a = b;
                b = current_value;
            }

            // ── 更新并发布 Feedback（当前已生成的部分数列）──
            feedback->current_sequence.push_back(current_value);
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),
                        "Progress: %ld/%ld, current term: %ld",
                        i + 1, goal->order, current_value);

            // 保存到结果序列
            result->sequence.push_back(current_value);

            loop_rate.sleep();
        }

        // ── 执行成功 ──
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
                    "Goal completed! Fibonacci sequence has %ld terms", result->sequence.size());
    }

    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
