/**
 * @file lifecycle_callback_groups.cpp
 * @brief ⑦ 生命周期回调组 —— LifecycleNode + CallbackGroup 配合
 *
 * 知识点：
 * - LifecycleNode 中使用回调组
 * - 生命周期转换回调与普通回调的回调组
 * - 生命周期服务自动使用的回调组
 * - MultiThreadedExecutor 驱动 LifecycleNode
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class LifecycleCallbackGroupNode : public rclcpp_lifecycle::LifecycleNode {
public:
    LifecycleCallbackGroupNode()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_cb_groups"),
          imu_count_(0), process_count_(0)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 生命周期回调组节点已创建 ===");

        // ================================================================
        // 1. 创建回调组
        //    与普通 Node 一样，LifecycleNode 也支持回调组
        // ================================================================
        sensor_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        processing_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // ================================================================
        // 2. 订阅绑定到回调组
        //    ⚠️ 注意：必须使用 rclcpp::SubscriptionOptions
        // ================================================================
        rclcpp::SubscriptionOptions sensor_opts;
        sensor_opts.callback_group = sensor_group_;

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr /*msg*/) {
                imu_count_++;
                if (imu_count_ % 50 == 0) {
                    int count = imu_count_;
                    RCLCPP_INFO(this->get_logger(),
                        "[IMU回调] #%d, 组=sensor_group", count);
                }
            }, sensor_opts);

        // 处理定时器绑定到 processing_group_
        process_timer_ = this->create_wall_timer(
            500ms,
            [this]() {
                process_count_++;
                int count = process_count_;
                RCLCPP_INFO(this->get_logger(),
                    "[处理回调] #%d, 组=processing_group, 状态=%s",
                    count, this->get_current_state().label().c_str());
            }, processing_group_);

        RCLCPP_INFO(this->get_logger(),
            "  sensor_group: IMU订阅（高频）");
        RCLCPP_INFO(this->get_logger(),
            "  processing_group: 处理定时器（低频）");
        RCLCPP_INFO(this->get_logger(),
            "  默认组: 生命周期转换服务");

        // ================================================================
        // ⚠️ 生命周期节点的特殊回调组行为：
        //
        // 生命周期转换服务（change_state/get_state）使用默认回调组
        // 因此在 MultiThreadedExecutor 下：
        //   - 传感器回调和处理回调可以并发执行
        //   - 生命周期状态转换不会阻塞传感器/处理回调
        //   - 但状态转换之间是串行的（同一个默认组）
        // ================================================================
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_configure (默认回调组) ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  配置资源");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_activate (默认回调组) ━━━");
        rclcpp_lifecycle::LifecycleNode::on_activate(state);
        RCLCPP_INFO(this->get_logger(),
            "  传感器数据将开始被处理");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_deactivate (默认回调组) ━━━");
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        imu_count_ = 0;
        process_count_ = 0;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        return CallbackReturn::SUCCESS;
    }

private:
    int imu_count_;
    int process_count_;

    rclcpp::CallbackGroup::SharedPtr sensor_group_;
    rclcpp::CallbackGroup::SharedPtr processing_group_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr process_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LifecycleCallbackGroupNode>();

    // 使用 MultiThreadedExecutor 让不同回调组并发执行
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3);
    executor.add_node(node->get_node_base_interface());

    RCLCPP_INFO(node->get_logger(),
        "使用 MultiThreadedExecutor (3线程)，回调组可并发执行");
    RCLCPP_INFO(node->get_logger(),
        "生命周期转换服务在默认回调组中，不阻塞传感器/处理回调");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
