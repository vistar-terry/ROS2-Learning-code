/**
 * @file 04_action_client_feedback.cpp
 * @brief Action Client 进阶 —— 反馈处理与进度条显示
 *
 * 知识点：
 * - 反馈数据的实际处理（进度条、ETA 估算）
 * - 动态目标值（通过 ROS2 参数）
 * - 交互式命令发送（通过话题触发）
 * - 多次发送目标
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_learning_cpp/action/count_up.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using CountUp = action_learning_cpp::action::CountUp;
using namespace std::chrono_literals;

class FeedbackClient : public rclcpp::Node {
public:
    FeedbackClient() : Node("feedback_client") {
        client_ = rclcpp_action::create_client<CountUp>(this, "count_up");

        // 声明参数：目标值可配置
        this->declare_parameter("target", 5);

        // 订阅命令话题：可随时发送新目标
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/client_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "send") {
                    auto target = this->get_parameter("target").as_int();
                    send_goal(target);
                }
            });

        RCLCPP_INFO(this->get_logger(), "=== Feedback Client 已创建 ===");
        RCLCPP_INFO(this->get_logger(), "等待 Action Server...");

        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action Server 未上线");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Action Server 已上线");

        // 自动发送一次目标
        auto target = this->get_parameter("target").as_int();
        send_goal(target);
    }

private:
    void send_goal(int64_t target) {
        auto goal_msg = CountUp::Goal();
        goal_msg.target = target;

        auto send_goal_options = rclcpp_action::Client<CountUp>::SendGoalOptions();

        // ── 目标响应回调 ──
        send_goal_options.goal_response_callback =
            [this, target](const rclcpp_action::ClientGoalHandle<CountUp>::SharedPtr&
                               goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(),
                        "目标被拒绝 (target=%ld)", target);
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "✓ 目标已接受 (target=%ld)，开始执行...", target);
                    start_time_ = this->now();
                }
            };

        // ── 反馈回调 —— 进度条 + ETA ──
        send_goal_options.feedback_callback =
            [this](
                rclcpp_action::ClientGoalHandle<CountUp>::SharedPtr,
                const std::shared_ptr<const CountUp::Feedback> feedback) {
                // ── 绘制进度条 ──
                //  [████████░░░░░░░░] 50.0% | current=5, ETA=5.0s
                int bar_width = 20;
                float progress = feedback->progress_percent / 100.0f;
                int filled = static_cast<int>(bar_width * progress);

                std::string bar(bar_width, ' ');
                for (int i = 0; i < bar_width; ++i) {
                    bar[i] = (i < filled) ? '#' : '-';
                }

                // ── 估算剩余时间 (ETA) ──
                std::string eta_str = "计算中...";
                if (start_time_.seconds() > 0 && progress > 0.01f) {
                    auto elapsed = (this->now() - start_time_).seconds();
                    double eta = elapsed / progress * (1.0 - progress);
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(1) << eta << "s";
                    eta_str = oss.str();
                }

                RCLCPP_INFO(this->get_logger(),
                    "[%s] %.1f%% | current=%ld, ETA=%s",
                    bar.c_str(), feedback->progress_percent,
                    feedback->current_count, eta_str.c_str());
            };

        // ── 结果回调 ──
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<CountUp>::WrappedResult&
                       result) {
                auto elapsed = (this->now() - start_time_).seconds();

                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),
                        "✓ 成功! final=%ld, 耗时=%.1fs, 序列长度=%zu",
                        result.result->final_count, elapsed,
                        result.result->sequence.size());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(),
                        "✗ 被中止, 耗时=%.1fs", elapsed);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(),
                        "⚠ 被取消, 耗时=%.1fs", elapsed);
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "未知结果");
                    break;
                }
            };

        RCLCPP_INFO(this->get_logger(), ">>> 发送目标: target = %ld", target);
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<CountUp>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Time start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeedbackClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
