/**
 * @file lifecycle_publisher.cpp
 * @brief ③ 生命周期发布器 —— LifecyclePublisher 按状态自动启用/禁用
 *
 * 知识点：
 * - LifecyclePublisher vs 普通 Publisher
 * - Inactive 状态下 publish() 静默失败（消息不发出）
 * - Active 状态下 publish() 正常发布
 * - is_activated() 检查发布器状态
 * - on_activate/on_deactivate 中自动管理发布器
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>

using namespace std::chrono_literals;

class LifecyclePublisherNode : public rclcpp_lifecycle::LifecycleNode {
public:
    LifecyclePublisherNode()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_publisher"),
          pub_count_(0)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 生命周期发布器节点已创建 ===");

        // ================================================================
        // 创建 LifecyclePublisher
        //    与普通 Publisher 不同：
        //    1. 在 Inactive 状态下，publish() 不会实际发送消息
        //    2. 调用 on_activate() 后自动启用
        //    3. 调用 on_deactivate() 后自动禁用
        //    4. 通过 is_activated() 检查状态
        // ================================================================
        lifecycle_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);

        RCLCPP_INFO(this->get_logger(),
            "LifecyclePublisher 已创建 (状态: Inactive, is_activated=%d)",
            lifecycle_pub_->is_activated() ? 1 : 0);

        // ================================================================
        // 创建定时器 —— 无论什么状态都尝试发布
        //    但只有在 Active 状态下消息才会实际发出
        // ================================================================
        pub_timer_ = this->create_wall_timer(
            100ms,  // 10Hz
            [this]() {
                pub_count_++;
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = this->now();
                msg.header.frame_id = "imu_link";

                // ⚠️ 关键：LifecyclePublisher 的行为取决于节点状态
                //
                // Inactive 状态:
                //   - publish() 被调用但消息不发出
                //   - is_activated() = false
                //   - 订阅者不会收到任何消息
                //
                // Active 状态:
                //   - publish() 正常发布
                //   - is_activated() = true
                //   - 订阅者正常接收消息
                lifecycle_pub_->publish(msg);

                // 每10次报告一次状态
                if (pub_count_ % 10 == 0) {
                    int count = pub_count_;
                    RCLCPP_INFO(this->get_logger(),
                        "[发布] #%d, is_activated=%d, %s",
                        count,
                        lifecycle_pub_->is_activated() ? 1 : 0,
                        lifecycle_pub_->is_activated() ? "✓ 消息已发出" : "✗ 消息被丢弃(Inactive)");
                }
            });
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_configure ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  创建发布器完成，但发布器仍为 Inactive");
        RCLCPP_INFO(this->get_logger(),
            "  is_activated = %d", lifecycle_pub_->is_activated() ? 1 : 0);
        RCLCPP_INFO(this->get_logger(),
            "  此时 publish() 不会实际发送消息");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_activate ━━━");

        // ⚠️ 必须调用父类 on_activate！
        //    它会激活所有 LifecyclePublisher
        //    调用后 is_activated() 返回 true
        rclcpp_lifecycle::LifecycleNode::on_activate(state);

        RCLCPP_INFO(this->get_logger(),
            "  发布器已激活！is_activated = %d",
            lifecycle_pub_->is_activated() ? 1 : 0);
        RCLCPP_INFO(this->get_logger(),
            "  现在 publish() 会实际发送消息");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_deactivate ━━━");

        // ⚠️ 必须调用父类 on_deactivate！
        //    它会停用所有 LifecyclePublisher
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

        RCLCPP_INFO(this->get_logger(),
            "  发布器已停用！is_activated = %d",
            lifecycle_pub_->is_activated() ? 1 : 0);
        RCLCPP_INFO(this->get_logger(),
            "  现在 publish() 不会发送消息");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_cleanup ━━━");
        pub_count_ = 0;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_shutdown ━━━");
        return CallbackReturn::SUCCESS;
    }

private:
    int pub_count_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr lifecycle_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecyclePublisherNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

/*
 * ══════════════════════════════════════════════════════════
 *  演示步骤
 * ══════════════════════════════════════════════════════════
 *
 * 终端1: 运行节点
 *   ros2 run lifecycle_learning_cpp lifecycle_publisher
 *
 * 终端2: 监听话题
 *   ros2 topic echo /imu/data
 *
 * 终端3: 控制生命周期
 *   ros2 lifecycle set /lifecycle_publisher configure   # 发布器仍不发出消息
 *   ros2 lifecycle set /lifecycle_publisher activate    # 开始发出消息
 *   ros2 lifecycle set /lifecycle_publisher deactivate  # 停止发出消息
 *   ros2 lifecycle set /lifecycle_publisher activate    # 再次发出消息
 *
 * 观察：
 *   - configure 后，echo 看不到消息
 *   - activate 后，echo 开始收到消息
 *   - deactivate 后，echo 停止收到消息
 * ══════════════════════════════════════════════════════════
 */
