/**
 * @file lifecycle_launch_node.cpp
 * @brief ⑥ Launch 编排生命周期 —— LifecycleNode + Launch 系统集成
 *
 * 知识点：
 * - Launch 文件中声明生命周期节点
 * - lifecycle_msgs 事件在 Launch 中的用法
 * - ros2_lifecycle 适配器节点
 * - Launch 中控制状态转换顺序
 * - 多个生命周期节点的编排启动
 *
 * 此节点本身是一个简单的生命周期节点，
 * 配合 06_lifecycle_launch.launch.py 演示 Launch 编排
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

// ================================================================
// 传感器驱动节点 —— 模拟生命周期控制的传感器
// ================================================================
class LifecycleLaunchNode : public rclcpp_lifecycle::LifecycleNode {
public:
    LifecycleLaunchNode()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_launch_node"),
          data_count_(0)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== Launch 编排生命周期节点已创建 ===");

        lifecycle_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/lifecycle_data", 10);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_configure ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  初始化资源：打开设备、分配缓冲区");

        // 模拟硬件初始化
        data_count_ = 0;

        RCLCPP_INFO(this->get_logger(),
            "  配置成功");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_activate ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  启动数据采集");

        // 启动数据发布定时器
        data_timer_ = this->create_wall_timer(
            200ms,
            [this]() {
                data_count_++;
                auto msg = std_msgs::msg::String();
                int count = data_count_;
                msg.data = "数据帧 #" + std::to_string(count);

                // LifecyclePublisher：仅在 Active 状态下发布
                lifecycle_pub_->publish(msg);
            });

        rclcpp_lifecycle::LifecycleNode::on_activate(state);

        RCLCPP_INFO(this->get_logger(),
            "  激活成功，开始发布数据");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_deactivate ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  暂停数据采集");

        // 停止数据发布
        if (data_timer_) {
            data_timer_->cancel();
        }

        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

        RCLCPP_INFO(this->get_logger(),
            "  停用成功，暂停发布");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_cleanup ━━━");
        data_timer_.reset();
        data_count_ = 0;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_shutdown ━━━");
        if (data_timer_) {
            data_timer_->cancel();
            data_timer_.reset();
        }
        return CallbackReturn::SUCCESS;
    }

private:
    int data_count_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr lifecycle_pub_;
    rclcpp::TimerBase::SharedPtr data_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleLaunchNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

/*
 * ══════════════════════════════════════════════════════════
 *  Launch 编排生命周期节点
 * ══════════════════════════════════════════════════════════
 *
 * 配套 Launch 文件 06_lifecycle_launch.launch.py 中使用：
 *
 * from launch_ros.actions import LifecycleNode
 * from launch_ros.events.lifecycle import ChangeState
 * from lifecycle_msgs.msg import Transition
 *
 * # 1. 声明生命周期节点
 * lifecycle_node = LifecycleNode(
 *     package='lifecycle_learning_cpp',
 *     executable='lifecycle_launch_node',
 *     name='lifecycle_launch_node',
 * )
 *
 * # 2. Launch 启动后自动 configure + activate
 * #    使用 EmitEvent + RegisterEventHandler 编排
 *
 * # 3. 或使用 ros2 launch 的 lifecycle_manager
 * #    Nav2 使用 nav2_lifecycle_manager 统一管理
 * ══════════════════════════════════════════════════════════
 */
