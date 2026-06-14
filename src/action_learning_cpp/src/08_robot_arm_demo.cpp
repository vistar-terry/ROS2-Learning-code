/**
 * @file 08_robot_arm_demo.cpp
 * @brief 机器人手臂控制 —— Action 实战案例
 *
 * 知识点：
 * - 自定义 Action 消息 (MoveArm) 的完整使用
 * - 模拟关节运动的平滑插值
 * - 基于时间的进度反馈（角度、百分比、状态）
 * - 速度因子控制
 * - 运动异常处理（越界、速度过快）
 *
 * 场景描述：
 *   一个单关节机械臂，客户端发送目标角度和速度，
 *   服务端模拟关节从当前角度平滑运动到目标角度，
 *   实时反馈当前角度和进度。
 *
 * 运行：
 *   ros2 run action_learning_cpp 08_robot_arm_demo
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/move_arm.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using MoveArm = action_learning_cpp::action::MoveArm;
using namespace std::chrono_literals;

// ── 机械臂关节模拟器 ──
class JointSimulator
{
public:
    JointSimulator() : current_angle_(0.0) {}

    // 设置当前角度
    void set_angle(double angle) { current_angle_ = angle; }
    double get_angle() const { return current_angle_; }

    // 向目标角度移动一步，返回移动后的角度
    double step_toward(double target_angle, double speed_factor, double dt)
    {
        double diff = target_angle - current_angle_;
        double max_step = speed_factor * max_angular_velocity_ * dt;

        if (std::abs(diff) < max_step)
        {
            current_angle_ = target_angle;
        }
        else
        {
            current_angle_ += (diff > 0 ? max_step : -max_step);
        }
        return current_angle_;
    }

    bool reached(double target) const
    {
        return std::abs(current_angle_ - target) < 0.001;
    }

private:
    double current_angle_;
    static constexpr double max_angular_velocity_ = 1.0; // rad/s
};

// ================================================================
// 机械臂 Action Server
// ================================================================
class RobotArmServer : public rclcpp::Node
{
public:
    RobotArmServer() : Node("robot_arm_server")
    {
        action_server_ = rclcpp_action::create_server<MoveArm>(
            this,
            "move_arm",
            std::bind(&RobotArmServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotArmServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&RobotArmServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== Robot Arm Action Server started ===");
        RCLCPP_INFO(this->get_logger(), "Action: /move_arm");
        RCLCPP_INFO(this->get_logger(), "Joint range: [-3.14, 3.14] rad");
        RCLCPP_INFO(this->get_logger(), "Initial angle: 0.0 rad");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveArm::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(),
                    "Received move request: target=%.2f rad, speed=%.1fx",
                    goal->target_angle, goal->speed);

        // ── 目标验证 ──
        // 关节角度范围 [-π, π]
        if (goal->target_angle < -M_PI || goal->target_angle > M_PI)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejected: target angle %.2f out of range [-pi, pi]",
                        goal->target_angle);
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 速度因子范围 [0.1, 2.0]
        if (goal->speed < 0.1 || goal->speed > 2.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Rejected: speed factor %.1f out of range [0.1, 2.0]",
                        goal->speed);
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveArm>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel accepted");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveArm>> goal_handle)
    {
        std::thread{std::bind(&RobotArmServer::execute, this,
                              std::placeholders::_1),
                    goal_handle}
            .detach();
    }

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveArm>> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveArm::Result>();
        auto feedback = std::make_shared<MoveArm::Feedback>();

        JointSimulator joint;
        joint.set_angle(sim_current_angle_);

        double target = goal->target_angle;
        double speed = goal->speed;
        double start_angle = joint.get_angle();
        double total_distance = std::abs(target - start_angle);

        RCLCPP_INFO(this->get_logger(),
                    "Moving: %.2f -> %.2f rad (distance=%.2f, speed=%.1fx)",
                    start_angle, target, total_distance, speed);

        auto start_time = std::chrono::steady_clock::now();
        rclcpp::Rate loop_rate(20); // 20Hz 控制频率

        while (!joint.reached(target))
        {
            // ── 检查取消 ──
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->final_angle = joint.get_angle();
                result->message = "Motion canceled";
                result->elapsed_time = elapsed_since(start_time);
                goal_handle->canceled(result);

                sim_current_angle_ = joint.get_angle();
                RCLCPP_INFO(this->get_logger(),
                            "Motion canceled, current angle: %.2f rad", result->final_angle);
                return;
            }

            // ── 执行一步运动 ──
            double dt = 0.05; // 50ms 步长
            joint.step_toward(target, speed, dt);

            // ── 发布 Feedback ──
            double current = joint.get_angle();
            double remaining = std::abs(target - current);
            float progress = 0.0f;
            if (total_distance > 0.001)
            {
                progress = (1.0f - static_cast<float>(remaining / total_distance)) * 100.0f;
            }

            feedback->current_angle = current;
            feedback->progress_percent = progress;
            feedback->status = "moving";
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        // ── 运动完成 ──
        double elapsed = elapsed_since(start_time);
        result->success = true;
        result->final_angle = joint.get_angle();
        result->elapsed_time = elapsed;
        result->message = "Motion completed";
        goal_handle->succeed(result);

        sim_current_angle_ = joint.get_angle();
        RCLCPP_INFO(this->get_logger(),
                    "Motion completed! angle=%.2f rad, elapsed=%.2fs",
                    result->final_angle, elapsed);
    }

    double elapsed_since(std::chrono::steady_clock::time_point start)
    {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - start).count();
    }

    rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
    double sim_current_angle_ = 0.0; // 模拟的当前关节角度
};

// ================================================================
// 机械臂 Action Client —— 发送多组运动指令
// ================================================================
class RobotArmClient : public rclcpp::Node
{
public:
    RobotArmClient() : Node("robot_arm_client"), goal_index_(0)
    {
        client_ = rclcpp_action::create_client<MoveArm>(this, "move_arm");

        // 预定义的运动序列：模拟机器人抓取任务
        // 1. 旋转到物体上方
        // 2. 下降到物体
        // 3. 返回初始位置
        goals_ = {
            {1.57, 1.0, "Rotate above object (90 deg)"},
            {0.79, 0.5, "Descend to object (45 deg)"},
            {0.0, 1.5, "Return to home (0 deg)"},
        };

        RCLCPP_INFO(this->get_logger(), "[Client] Waiting for Action Server...");

        if (!client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "Server not available");
            return;
        }

        // 延迟1秒后发送第一个目标
        timer_ = this->create_wall_timer(1s, [this]()
                                         {
            timer_->cancel();
            send_next_goal(); });
    }

private:
    void send_next_goal()
    {
        if (goal_index_ >= goals_.size())
        {
            RCLCPP_INFO(this->get_logger(),
                        "[Client] ====== All motion commands completed ======");
            return;
        }

        auto &[angle, speed, desc] = goals_[goal_index_];

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
                RCLCPP_INFO(this->get_logger(),
                            "[Client] > %s — accepted", desc.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),
                             "[Client] X %s — rejected", desc.c_str());
            }
        };

        options.feedback_callback =
            [this, desc](
                rclcpp_action::ClientGoalHandle<MoveArm>::SharedPtr,
                const std::shared_ptr<const MoveArm::Feedback> fb)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[Client] %s | %.2f rad (%.1f%%) %s",
                        desc.c_str(), fb->current_angle,
                        fb->progress_percent, fb->status.c_str());
        };

        options.result_callback =
            [this, desc](const rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult &
                             result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Client] ✓ %s done | final=%.2f rad, time=%.2fs",
                            desc.c_str(), result.result->final_angle,
                            result.result->elapsed_time);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "[Client] ! %s failed", desc.c_str());
            }
            // 发送下一个目标
            goal_index_++;
            send_next_goal();
        };

        RCLCPP_INFO(this->get_logger(),
                    "[Client] >>> Sending: %s", desc.c_str());
        client_->async_send_goal(goal_msg, options);
    }

    rclcpp_action::Client<MoveArm>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t goal_index_;
    std::vector<std::tuple<double, double, std::string>> goals_;
};

// ================================================================
// main：Server + Client 在同一进程
// ================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto server = std::make_shared<RobotArmServer>();
    auto client = std::make_shared<RobotArmClient>();

    executor.add_node(server);
    executor.add_node(client);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
