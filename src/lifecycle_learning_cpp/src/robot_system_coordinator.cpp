/**
 * @file robot_system_coordinator.cpp
 * @brief ⑩ 机器人多子系统协调 —— 多生命周期节点的有序启停
 *
 * 知识点：
 * - 多个生命周期节点的有序启动和关闭
 * - 子系统依赖关系：传感器 → 定位 → 规划 → 控制
 * - 启动顺序：先启动底层（传感器），再启动上层（控制）
 * - 关闭顺序：先关闭上层（控制），再关闭底层（传感器）
 * - 统一生命周期管理器的设计模式
 *
 * 启动顺序（依赖链）：
 *   传感器驱动 → 定位节点 → 路径规划 → 速度控制
 *
 * 关闭顺序（逆序）：
 *   速度控制 → 路径规划 → 定位节点 → 传感器驱动
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <map>

using namespace std::chrono_literals;

class RobotSystemCoordinator : public rclcpp::Node {
public:
    RobotSystemCoordinator()
        : Node("system_coordinator"), phase_(0)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 机器人系统协调器已创建 ===");

        // ================================================================
        // 定义子系统启动顺序（按依赖关系）
        //
        //   传感器驱动 (imu_driver)
        //       ↓ 依赖传感器数据
        //   定位节点 (localization)
        //       ↓ 依赖定位结果
        //   路径规划 (planner)
        //       ↓ 依赖规划路径
        //   速度控制 (controller)
        //
        // 关闭顺序为启动的逆序
        // ================================================================
        startup_sequence_ = {
            {"imu_driver",     "传感器驱动", 1},
            {"localization",   "定位节点",   2},
            {"planner",        "路径规划",   3},
            {"controller",     "速度控制",   4},
        };

        // 为每个子系统创建服务客户端
        for (const auto& subsystem : startup_sequence_) {
            auto change_client = this->create_client<lifecycle_msgs::srv::ChangeState>(
                "/" + subsystem.name + "/change_state");
            auto state_client = this->create_client<lifecycle_msgs::srv::GetState>(
                "/" + subsystem.name + "/get_state");
            change_clients_[subsystem.name] = change_client;
            state_clients_[subsystem.name] = state_client;
        }

        // ================================================================
        // 自动启动定时器
        //    每2秒执行一步，逐步将所有子系统带到 Active 状态
        // ================================================================
        startup_timer_ = this->create_wall_timer(
            3s,
            [this]() { startup_step(); });

        // 状态监控定时器
        monitor_timer_ = this->create_wall_timer(
            5s,
            [this]() { monitor_all_subsystems(); });

        // 命令订阅
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/coordinator_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(),
                    "收到命令: '%s'", msg->data.c_str());
                if (msg->data == "startup") {
                    phase_ = 0;
                    startup_timer_->reset();
                    RCLCPP_INFO(this->get_logger(), "开始启动序列");
                } else if (msg->data == "shutdown") {
                    shutdown_all();
                } else if (msg->data == "status") {
                    monitor_all_subsystems();
                }
            });

        RCLCPP_INFO(this->get_logger(), "协调器就绪，开始自动启动序列...");
    }

private:
    struct SubsystemInfo {
        std::string name;
        std::string description;
        int order;
    };

    // ================================================================
    // 启动步骤：逐步将每个子系统 configure → activate
    // ================================================================
    void startup_step() {
        if (phase_ >= static_cast<int>(startup_sequence_.size())) {
            RCLCPP_INFO(this->get_logger(),
                "═══ 所有子系统已启动！═══");
            startup_timer_->cancel();
            return;
        }

        const auto& subsystem = startup_sequence_[phase_];
        RCLCPP_INFO(this->get_logger(),
            ">>> 启动第%d个子系统: %s (%s)",
            phase_ + 1, subsystem.name.c_str(), subsystem.description.c_str());

        // configure
        send_change_state(subsystem.name,
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure");

        // activate
        send_change_state(subsystem.name,
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate");

        phase_++;
    }

    // ================================================================
    // 关闭所有子系统（逆序）
    // ================================================================
    void shutdown_all() {
        RCLCPP_INFO(this->get_logger(),
            "═══ 开始关闭序列（逆序）═══");

        for (int i = static_cast<int>(startup_sequence_.size()) - 1; i >= 0; --i) {
            const auto& subsystem = startup_sequence_[i];
            RCLCPP_INFO(this->get_logger(),
                ">>> 关闭第%d个子系统: %s (%s)",
                i + 1, subsystem.name.c_str(), subsystem.description.c_str());

            // deactivate
            send_change_state(subsystem.name,
                lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "deactivate");

            // cleanup
            send_change_state(subsystem.name,
                lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "cleanup");

            // shutdown — 注意：没有通用的 TRANSITION_SHUTDOWN
            // 需根据当前状态选择对应转换，这里简化处理
            // cleanup 后节点回到 Unconfigured → 用 TRANSITION_UNCONFIGURED_SHUTDOWN
            send_change_state(subsystem.name,
                lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, "shutdown");
        }

        RCLCPP_INFO(this->get_logger(), "═══ 所有子系统已关闭 ═══");
    }

    // ================================================================
    // 发送状态转换请求
    // ================================================================
    void send_change_state(const std::string& node_name,
                           uint8_t transition_id,
                           const std::string& transition_name)
    {
        auto it = change_clients_.find(node_name);
        if (it == change_clients_.end()) {
            RCLCPP_WARN(this->get_logger(),
                "未找到 %s 的 change_state 客户端", node_name.c_str());
            return;
        }

        if (!it->second->wait_for_service(500ms)) {
            RCLCPP_WARN(this->get_logger(),
                "子系统 %s 不可用 (服务未就绪)", node_name.c_str());
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        it->second->async_send_request(request,
            [node_name, transition_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(rclcpp::get_logger("coordinator"),
                        "  %s: %s 成功", node_name.c_str(), transition_name.c_str());
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("coordinator"),
                        "  %s: %s 失败!", node_name.c_str(), transition_name.c_str());
                }
            });
    }

    // ================================================================
    // 监控所有子系统状态
    // ================================================================
    void monitor_all_subsystems() {
        RCLCPP_INFO(this->get_logger(), "─── 子系统状态监控 ───");
        for (const auto& subsystem : startup_sequence_) {
            auto it = state_clients_.find(subsystem.name);
            if (it != state_clients_.end() && it->second->service_is_ready()) {
                auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
                it->second->async_send_request(request,
                    [name = subsystem.name, desc = subsystem.description](
                        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                        auto response = future.get();
                        RCLCPP_INFO(rclcpp::get_logger("coordinator"),
                            "  %s (%s): %s",
                            name.c_str(), desc.c_str(),
                            response->current_state.label.c_str());
                    });
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "  %s (%s): 离线",
                    subsystem.name.c_str(), subsystem.description.c_str());
            }
        }
    }

    std::vector<SubsystemInfo> startup_sequence_;
    int phase_;

    std::map<std::string,
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_clients_;
    std::map<std::string,
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> state_clients_;

    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSystemCoordinator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
