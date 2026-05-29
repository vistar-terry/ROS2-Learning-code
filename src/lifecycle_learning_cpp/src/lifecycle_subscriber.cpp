/**
 * @file lifecycle_subscriber.cpp
 * @brief ④ 生命周期订阅器 —— 按状态控制订阅行为
 *
 * 知识点：
 * - 生命周期节点中订阅的启用/禁用策略
 * - Inactive 状态下订阅回调仍会执行（与LifecyclePublisher不同！）
 * - 手动控制订阅：通过条件变量或开关过滤
 * - 推荐设计模式：在回调中检查当前状态
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class LifecycleSubscriberNode : public rclcpp_lifecycle::LifecycleNode {
public:
    LifecycleSubscriberNode()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_subscriber"),
          msg_count_(0), processed_count_(0)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 生命周期订阅器节点已创建 ===");

        // ================================================================
        // 创建订阅 —— 生命周期节点中的订阅行为
        //
        // ⚠️ 重要区别：
        //    LifecyclePublisher: Inactive 状态下自动不发布
        //    普通订阅:          Inactive 状态下回调仍会执行！
        //
        // 这意味着即使节点处于 Inactive 状态，
        // 订阅回调依然会被触发。需要在回调中手动检查状态。
        // ================================================================
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr /*msg*/) {
                msg_count_++;
                // ⚠️ 即使在 Inactive 状态，此回调也会执行！
                //    必须手动检查节点状态
                auto state = this->get_current_state();
                if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    processed_count_++;
                    // 仅在 Active 状态下处理数据
                    if (processed_count_ % 10 == 0) {
                        int proc = processed_count_;
                        RCLCPP_INFO(this->get_logger(),
                            "[Active] 处理IMU数据 #%d", proc);
                    }
                } else {
                    // Inactive/Unconfigured 状态：丢弃数据
                    if (msg_count_ % 50 == 0) {
                        int total = msg_count_;
                        RCLCPP_WARN(this->get_logger(),
                            "[%s] 丢弃IMU数据 (总接收=%d, 已处理=%d)",
                            state.label().c_str(), total, processed_count_);
                    }
                }
            });

        // 命令订阅 —— 用于演示手动启用/禁用
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/lifecycle_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(),
                    "收到命令: '%s' (当前状态: %s)",
                    msg->data.c_str(),
                    this->get_current_state().label().c_str());
            });

        // 状态报告定时器
        report_timer_ = this->create_wall_timer(
            2s,
            [this]() {
                int total = msg_count_;
                int proc = processed_count_;
                RCLCPP_INFO(this->get_logger(),
                    "[报告] 总接收=%d, 已处理=%d, 状态=%s",
                    total, proc,
                    this->get_current_state().label().c_str());
            });
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_configure ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  订阅已存在，但节点未激活");
        RCLCPP_INFO(this->get_logger(),
            "  ⚠️ 订阅回调仍会执行，但在回调中检查状态后丢弃数据");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_activate ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  节点已激活，订阅回调将处理数据");
        rclcpp_lifecycle::LifecycleNode::on_activate(state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_deactivate ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  节点已停用，订阅回调将丢弃数据");
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_cleanup ━━━");
        msg_count_ = 0;
        processed_count_ = 0;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        return CallbackReturn::SUCCESS;
    }

private:
    int msg_count_;
    int processed_count_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleSubscriberNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
