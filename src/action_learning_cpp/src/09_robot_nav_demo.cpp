/**
 * @file 09_robot_nav_demo.cpp
 * @brief 机器人导航 —— Action 实战案例
 *
 * 知识点：
 * - 自定义 Action 消息 (Navigate) 的完整使用
 * - 2D 平面导航模拟（起点→终点）
 * - 路径跟踪与距离反馈
 * - 模拟障碍物检测与避障
 * - 速度因子影响导航速度
 *
 * 场景描述：
 *   模拟机器人在 2D 平面上导航到目标位置，
 *   服务端模拟机器人沿直线运动到目标点，
 *   实时反馈当前位置、剩余距离和进度。
 *   支持障碍物检测（模拟）和取消。
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/navigate.hpp>
#include <cmath>
#include <thread>
#include <chrono>
#include <random>

using Navigate = action_learning_cpp::action::Navigate;
using namespace std::chrono_literals;

// ── 2D 点 ──
struct Point2D
{
    double x, y;
    double distance_to(const Point2D &other) const
    {
        return std::hypot(other.x - x, other.y - y);
    }
    Point2D move_toward(const Point2D &target, double step) const
    {
        double d = distance_to(target);
        if (d < step)
            return target;
        double ratio = step / d;
        return {x + (target.x - x) * ratio, y + (target.y - y) * ratio};
    }
};

// ================================================================
// 导航 Action Server
// ================================================================
class NavServer : public rclcpp::Node
{
public:
    NavServer() : Node("nav_server")
    {
        action_server_ = rclcpp_action::create_server<Navigate>(
            this,
            "navigate",
            std::bind(&NavServer::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavServer::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&NavServer::handle_accepted, this,
                      std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== 导航 Action Server 已启动 ===");
        RCLCPP_INFO(this->get_logger(), "Action: /navigate");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Navigate::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(this->get_logger(),
                    "收到导航请求: (%.2f, %.2f), speed=%.1fx",
                    goal->target_x, goal->target_y, goal->speed);

        // 目标位置不能离原点太远
        double dist = std::hypot(goal->target_x, goal->target_y);
        if (dist > 100.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "拒绝: 目标距离 %.2f 超过最大范围 100", dist);
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->speed < 0.1 || goal->speed > 2.0)
        {
            RCLCPP_WARN(this->get_logger(), "拒绝: 速度因子超范围");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Navigate>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "允许取消导航");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Navigate>> goal_handle)
    {
        std::thread{std::bind(&NavServer::execute, this,
                              std::placeholders::_1),
                    goal_handle}
            .detach();
    }

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<Navigate>> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Navigate::Result>();
        auto feedback = std::make_shared<Navigate::Feedback>();

        Point2D target{goal->target_x, goal->target_y};
        Point2D current = sim_position_;
        double total_distance = current.distance_to(target);
        double base_speed = 1.0; // m/s
        double speed = base_speed * goal->speed;
        double step = speed * 0.05; // 50ms 步长

        RCLCPP_INFO(this->get_logger(),
                    "开始导航: (%.2f,%.2f) → (%.2f,%.2f), 距离=%.2fm",
                    current.x, current.y, target.x, target.y, total_distance);

        auto start_time = std::chrono::steady_clock::now();
        rclcpp::Rate loop_rate(20); // 20Hz

        // 模拟随机障碍物检测（10%概率触发避障）
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        while (current.distance_to(target) > 0.05)
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->final_x = current.x;
                result->final_y = current.y;
                result->message = "导航被取消";
                result->elapsed_time = elapsed_since(start_time);
                goal_handle->canceled(result);
                sim_position_ = current;

                RCLCPP_INFO(this->get_logger(),
                            "导航取消, 位置: (%.2f, %.2f)", current.x, current.y);
                return;
            }

            // ── 模拟障碍物检测 ──
            if (dist(rng) < 0.02)
            { // 2% 概率遇到障碍
                RCLCPP_WARN(this->get_logger(),
                            "⚠ 检测到障碍物! 暂停 0.5s 避障...");
                feedback->status = "obstacle_avoidance";
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(500ms);
                // 模拟微小偏移避障
                current.x += (dist(rng) - 0.5) * 0.1;
                current.y += (dist(rng) - 0.5) * 0.1;
                feedback->status = "navigating";
            }

            // ── 执行一步运动 ──
            current = current.move_toward(target, step);
            double remaining = current.distance_to(target);

            // ── 发布 Feedback ──
            float progress = 0.0f;
            if (total_distance > 0.01)
            {
                progress = (1.0f - static_cast<float>(remaining / total_distance)) * 100.0f;
            }
            feedback->current_x = current.x;
            feedback->current_y = current.y;
            feedback->progress_percent = progress;
            feedback->distance_remaining = remaining;
            feedback->status = "navigating";
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        // ── 导航完成 ──
        double elapsed = elapsed_since(start_time);
        result->success = true;
        result->final_x = current.x;
        result->final_y = current.y;
        result->elapsed_time = elapsed;
        result->message = "导航完成";
        goal_handle->succeed(result);
        sim_position_ = current;

        RCLCPP_INFO(this->get_logger(),
                    "✓ 导航完成! (%.2f, %.2f), 耗时=%.2fs",
                    result->final_x, result->final_y, elapsed);
    }

    double elapsed_since(std::chrono::steady_clock::time_point start)
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - start)
            .count();
    }

    rclcpp_action::Server<Navigate>::SharedPtr action_server_;
    Point2D sim_position_{0.0, 0.0}; // 模拟当前位置
};

// ================================================================
// 导航 Action Client —— 发送多个路径点
// ================================================================
class NavClient : public rclcpp::Node
{
public:
    NavClient() : Node("nav_client"), waypoint_index_(0)
    {
        client_ = rclcpp_action::create_client<Navigate>(this, "navigate");

        // 预定义路径点：模拟巡逻任务
        waypoints_ = {
            {3.0, 0.0, 1.0, "前进到 (3,0)"},
            {3.0, 4.0, 1.0, "右转到 (3,4)"},
            {0.0, 4.0, 1.0, "后退到 (0,4)"},
            {0.0, 0.0, 1.5, "返回原点 (0,0) — 加速"},
        };

        if (!client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(this->get_logger(), "Server 未上线");
            return;
        }

        timer_ = this->create_wall_timer(1s, [this]()
                                         {
            timer_->cancel();
            navigate_next(); });
    }

private:
    void navigate_next()
    {
        if (waypoint_index_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(),
                        "[Client] ====== 巡逻任务完成 ======");
            return;
        }

        auto &[x, y, speed, desc] = waypoints_[waypoint_index_];

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
                RCLCPP_INFO(this->get_logger(),
                            "[Client] ▶ %s — 已接受", desc.c_str());
            }
        };

        options.feedback_callback =
            [this, desc](
                rclcpp_action::ClientGoalHandle<Navigate>::SharedPtr,
                const std::shared_ptr<const Navigate::Feedback> fb)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[Client] %s | (%.2f,%.2f) dist=%.2f %.1f%% %s",
                        desc.c_str(), fb->current_x, fb->current_y,
                        fb->distance_remaining, fb->progress_percent,
                        fb->status.c_str());
        };

        options.result_callback =
            [this, desc](const rclcpp_action::ClientGoalHandle<Navigate>::WrappedResult &
                             result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Client] ✓ %s | (%.2f,%.2f) %.2fs",
                            desc.c_str(), result.result->final_x,
                            result.result->final_y, result.result->elapsed_time);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "[Client] ⚠ %s 未成功", desc.c_str());
            }
            waypoint_index_++;
            navigate_next();
        };

        RCLCPP_INFO(this->get_logger(),
                    "[Client] >>> 导航: %s", desc.c_str());
        client_->async_send_goal(goal_msg, options);
    }

    rclcpp_action::Client<Navigate>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t waypoint_index_;
    std::vector<std::tuple<double, double, double, std::string>> waypoints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto server = std::make_shared<NavServer>();
    auto client = std::make_shared<NavClient>();

    executor.add_node(server);
    executor.add_node(client);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
