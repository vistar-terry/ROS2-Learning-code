/**
 * @file lifecycle_manager_node.cpp
 * @brief ⑤ 外部状态管理器 —— 通过服务调用管理其他生命周期节点
 *
 * 知识点：
 * - 生命周期节点的底层服务接口
 * - change_state 服务：触发状态转换
 * - get_available_states 服务：查询可用状态
 * - get_available_transitions 服务：查询可用转换
 * - get_state 服务：查询当前状态
 * - 自定义生命周期管理器的实现
 *
 * ⚠️ 重要：单线程 Executor 下的服务调用模式
 *   在 ROS2 单线程 executor 中，future.wait_for() 会阻塞 executor，
 *   导致服务响应回调无法被投递 → future 永远不会 ready → 死锁超时。
 *   正确做法：使用 async_send_request(request, callback) 回调模式，
 *   让 executor 自由处理回调。
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/get_available_transitions.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

class LifecycleManagerNode : public rclcpp::Node {
public:
    LifecycleManagerNode()
        : Node("lifecycle_manager"), managed_node_("/lifecycle_basic"),
          desired_state_(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE),
          restart_count_(0), max_restart_attempts_(3),
          auto_manage_enabled_(true)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 生命周期管理器已创建 ===");
        RCLCPP_INFO(this->get_logger(),
            "被管理节点: %s", managed_node_.c_str());

        // 声明参数
        this->declare_parameter("managed_node", std::string("/lifecycle_basic"));
        this->declare_parameter("max_restart_attempts", 3);
        this->declare_parameter("auto_manage", true);
        this->declare_parameter("check_interval_ms", 2000);

        managed_node_ = this->get_parameter("managed_node").as_string();
        max_restart_attempts_ = this->get_parameter("max_restart_attempts").as_int();
        auto_manage_enabled_ = this->get_parameter("auto_manage").as_bool();
        auto interval = std::chrono::milliseconds(
            this->get_parameter("check_interval_ms").as_int());

        // ================================================================
        // 生命周期节点自动暴露的服务
        //    每个生命周期节点都会自动创建以下服务：
        //
        //    <node_name>/change_state           — 触发状态转换
        //    <node_name>/get_state              — 查询当前状态
        //    <node_name>/get_available_states    — 查询所有可达状态
        //    <node_name>/get_available_transitions — 查询可用转换
        // ================================================================

        // 创建服务客户端
        change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            managed_node_ + "/change_state");

        get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            managed_node_ + "/get_state");

        get_transitions_client_ = this->create_client<lifecycle_msgs::srv::GetAvailableTransitions>(
            managed_node_ + "/get_available_transitions");

        // ================================================================
        // 自动管理定时器 —— 周期性检查被管理节点的健康状态
        //    这是 nav2_lifecycle_manager 的简化版
        //    每个定时周期只推进一个转换步骤，下次定时器再检查
        // ================================================================
        auto_manage_timer_ = this->create_wall_timer(
            interval,
            [this]() {
                auto_manage_step();
            });

        // 命令订阅 —— 手动控制
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/manager_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(),
                    "收到命令: '%s'", msg->data.c_str());
                if (msg->data == "startup") {
                    desired_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
                    restart_count_ = 0;
                    auto_manage_enabled_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        ">>> 启动: 期望 %s 到达 Active", managed_node_.c_str());
                } else if (msg->data == "shutdown") {
                    desired_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
                    RCLCPP_INFO(this->get_logger(),
                        ">>> 关闭: 期望 %s 到达 Finalized", managed_node_.c_str());
                } else if (msg->data == "pause") {
                    desired_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
                    RCLCPP_INFO(this->get_logger(),
                        ">>> 暂停: 期望 %s 到达 Inactive", managed_node_.c_str());
                } else if (msg->data == "resume") {
                    desired_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
                    restart_count_ = 0;
                    auto_manage_enabled_ = true;
                    RCLCPP_INFO(this->get_logger(),
                        ">>> 恢复: 期望 %s 到达 Active", managed_node_.c_str());
                } else if (msg->data == "status") {
                    // 异步查询状态，在回调中打印
                    query_state_async([this](uint8_t state) {
                        RCLCPP_INFO(this->get_logger(),
                            "[状态] %s: %s, 期望: %s, 重试: %d/%d, 自动管理: %s",
                            managed_node_.c_str(),
                            state_to_string(state).c_str(),
                            state_to_string(desired_state_).c_str(),
                            restart_count_, max_restart_attempts_,
                            auto_manage_enabled_ ? "开" : "关");
                    });
                } else if (msg->data == "enable_auto") {
                    auto_manage_enabled_ = true;
                    restart_count_ = 0;
                    RCLCPP_INFO(this->get_logger(), "已启用自动管理");
                } else if (msg->data == "disable_auto") {
                    auto_manage_enabled_ = false;
                    RCLCPP_INFO(this->get_logger(), "已禁用自动管理");
                }
            });

        // 告警发布器 —— 当自动管理遇到严重问题时发布告警
        alert_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/lifecycle_alerts", 10);

        // 启动自动管理（仅设置期望状态，由定时器驱动实际转换）
        RCLCPP_INFO(this->get_logger(),
            ">>> 启动序列: 期望 %s 到达 Active 状态，由自动管理定时器驱动",
            managed_node_.c_str());
    }

private:
    // ================================================================
    // 异步查询当前状态（回调模式）
    //    ⚠️ 不能用 future.wait_for() —— 会阻塞 executor 导致死锁
    //    正确做法：async_send_request(request, callback)
    // ================================================================
    void query_state_async(std::function<void(uint8_t)> callback) {
        if (!get_state_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(),
                "服务 %s/get_state 不可用", managed_node_.c_str());
            callback(lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN);
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        get_state_client_->async_send_request(request,
            [this, callback](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                auto response = future.get();
                uint8_t state_id = response->current_state.id;
                RCLCPP_DEBUG(this->get_logger(),
                    "查询到 %s 状态: %s (id=%d)",
                    managed_node_.c_str(),
                    response->current_state.label.c_str(), state_id);
                callback(state_id);
            });
    }

    // ================================================================
    // 异步发送状态转换请求（回调模式）
    //    transition_id: 转换 ID
    //    transition_name: 可读名称（用于日志）
    //    callback: 转换结果回调（true=成功, false=失败/超时）
    // ================================================================
    void send_change_state_async(uint8_t transition_id,
                                  const std::string& transition_name,
                                  std::function<void(bool)> callback = nullptr) {
        if (!change_state_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(),
                "服务 %s/change_state 不可用", managed_node_.c_str());
            if (callback) callback(false);
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        RCLCPP_INFO(this->get_logger(),
            "发送转换请求: %s (id=%d)", transition_name.c_str(), transition_id);

        change_state_client_->async_send_request(request,
            [this, transition_name, callback](
                rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(),
                        "✓ 转换 %s 成功", transition_name.c_str());
                    if (callback) callback(true);
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "✗ 转换 %s 失败", transition_name.c_str());
                    if (callback) callback(false);
                }
            });
    }

    // ================================================================
    // 自动管理步骤 —— 核心自动监控与恢复逻辑
    //
    // 设计原则：
    // - 每个定时周期只推进一个转换步骤
    // - 使用异步回调，不阻塞 executor
    // - 下次定时器触发时再次检查，确保收敛到期望状态
    //
    // 工作流程：
    // 1. 异步查询被管理节点当前状态
    // 2. 在回调中根据当前状态 vs 期望状态做决策
    // 3. 发送一个转换请求（异步），下次定时器再检查
    //
    // 设计参考：Nav2 的 lifecycle_manager
    // ================================================================
    void auto_manage_step() {
        if (!auto_manage_enabled_) {
            return;  // 自动管理未启用，跳过
        }

        // 异步查询状态 → 在回调中决策
        query_state_async([this](uint8_t current_state) {
            handle_auto_manage_result(current_state);
        });
    }

    // ================================================================
    // 自动管理决策逻辑（在状态查询回调中执行）
    // ================================================================
    void handle_auto_manage_result(uint8_t current_state) {
        RCLCPP_DEBUG(this->get_logger(),
            "[自动管理] %s 当前: %s, 期望: %s",
            managed_node_.c_str(),
            state_to_string(current_state).c_str(),
            state_to_string(desired_state_).c_str());

        switch (current_state) {
        // ────────────────────────────────────────
        // 情况A：节点服务不可达（可能还没启动或已崩溃）
        // ────────────────────────────────────────
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[自动管理] %s 状态未知，服务不可达，等待节点上线...",
                managed_node_.c_str());
            break;

        // ────────────────────────────────────────
        // 情况B：节点刚启动，处于 Unconfigured
        //   期望 Active/Inactive → configure
        //   期望 Finalized → 直接 unconfigured_shutdown
        // ────────────────────────────────────────
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
            if (desired_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
                RCLCPP_INFO(this->get_logger(),
                    "[自动管理] %s 处于 Unconfigured，执行 shutdown...",
                    managed_node_.c_str());
                send_change_state_async(
                    lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN,
                    "unconfigured_shutdown");
            } else if (desired_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
                       desired_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                RCLCPP_INFO(this->get_logger(),
                    "[自动管理] %s 处于 Unconfigured，发送 configure...",
                    managed_node_.c_str());
                send_change_state_async(
                    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                    "configure",
                    [this](bool success) {
                        if (success) {
                            restart_count_ = 0;
                            // configure 成功后如果期望 Active，
                            // 下次定时器会自动发 activate
                        } else {
                            handle_restart_failure();
                        }
                    });
            }
            break;

        // ────────────────────────────────────────
        // 情况C：节点已配置，处于 Inactive
        //   期望 Active → activate
        //   期望 Unconfigured → cleanup
        //   期望 Finalized → inactive_shutdown（一步到位）
        // ────────────────────────────────────────
        case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
            if (desired_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_INFO(this->get_logger(),
                    "[自动管理] %s 处于 Inactive，发送 activate...",
                    managed_node_.c_str());
                send_change_state_async(
                    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                    "activate",
                    [this](bool success) {
                        if (success) restart_count_ = 0;
                        else handle_restart_failure();
                    });
            } else if (desired_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
                RCLCPP_INFO(this->get_logger(),
                    "[自动管理] %s 处于 Inactive，执行 inactive_shutdown...",
                    managed_node_.c_str());
                send_change_state_async(
                    lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN,
                    "inactive_shutdown");
            } else if (desired_state_ == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                RCLCPP_INFO(this->get_logger(),
                    "[自动管理] %s 处于 Inactive，发送 cleanup...",
                    managed_node_.c_str());
                send_change_state_async(
                    lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                    "cleanup");
            }
            break;

        // ────────────────────────────────────────
        // 情况D：节点正常工作，处于 Active
        //   期望 Inactive/Unconfigured/Finalized → deactivate
        //   期望 Active → 健康，无需操作
        // ────────────────────────────────────────
        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
            if (desired_state_ != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_INFO(this->get_logger(),
                    "[自动管理] %s 处于 Active，发送 deactivate...",
                    managed_node_.c_str());
                send_change_state_async(
                    lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                    "deactivate");
            } else {
                // 期望 Active 且当前 Active → 健康，无需操作
                RCLCPP_DEBUG(this->get_logger(),
                    "[自动管理] %s 运行正常 (Active)", managed_node_.c_str());
            }
            break;

        // ────────────────────────────────────────
        // 情况E：节点已死亡，处于 Finalized
        //   → 无法通过服务调用恢复，发出告警
        // ────────────────────────────────────────
        case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
            RCLCPP_ERROR(this->get_logger(),
                "[自动管理] ⚠️ %s 已进入 Finalized 状态（节点死亡）!",
                managed_node_.c_str());
            handle_restart_failure();

            if (restart_count_ <= max_restart_attempts_) {
                RCLCPP_WARN(this->get_logger(),
                    "[自动管理] Finalized 节点无法通过服务调用恢复，"
                    "需要重启进程！");
            }
            break;

        // ────────────────────────────────────────
        // 情况F：过渡状态（Configuring/Activating 等）
        //   → 不做操作，等待下次检查
        // ────────────────────────────────────────
        default:
            // 过渡状态 (id >= 10)：Configuring/Activating/Deactivating 等
            RCLCPP_DEBUG(this->get_logger(),
                "[自动管理] %s 处于过渡状态 (id=%d)，等待...",
                managed_node_.c_str(), current_state);
            break;
        }
    }

    // ================================================================
    // 重启失败处理
    //    超过最大重试次数后禁用自动管理并发出告警
    // ================================================================
    void handle_restart_failure() {
        restart_count_++;
        if (restart_count_ > max_restart_attempts_) {
            RCLCPP_ERROR(this->get_logger(),
                "[自动管理] %s 重启失败次数超过上限 (%d 次)，"
                "禁用自动管理！请手动检查节点状态。",
                managed_node_.c_str(), max_restart_attempts_);
            auto_manage_enabled_ = false;
            // 发布告警消息
            auto alert = std_msgs::msg::String();
            alert.data = "ALERT: " + managed_node_ + " 重启失败，自动管理已禁用!";
            alert_pub_->publish(alert);
        }
    }

    // ================================================================
    // 状态ID → 可读字符串
    // ================================================================
    static std::string state_to_string(uint8_t state_id) {
        switch (state_id) {
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:     return "Unknown";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED: return "Unconfigured";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:     return "Inactive";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:       return "Active";
        case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:    return "Finalized";
        default: return "Transitioning(id=" + std::to_string(state_id) + ")";
        }
    }

    std::string managed_node_;

    // 期望状态：管理器会将节点自动带到此状态
    uint8_t desired_state_;
    // 重启计数与上限
    int restart_count_;
    int max_restart_attempts_;
    // 自动管理开关
    bool auto_manage_enabled_;

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetAvailableTransitions>::SharedPtr get_transitions_client_;
    rclcpp::TimerBase::SharedPtr auto_manage_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
