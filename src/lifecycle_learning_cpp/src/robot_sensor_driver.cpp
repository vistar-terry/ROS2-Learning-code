/**
 * @file robot_sensor_driver.cpp
 * @brief ⑧ 机器人传感器驱动 —— 生命周期控制的传感器节点
 *
 * 知识点：
 * - 生命周期节点在传感器驱动中的实际应用
 * - configure: 初始化硬件连接
 * - activate: 开始采集数据
 * - deactivate: 暂停采集（不断开硬件）
 * - cleanup: 断开硬件连接
 * - shutdown: 安全关闭
 * - 硬件故障处理（on_error）
 *
 * 典型场景：IMU驱动、激光雷达驱动、相机驱动
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class RobotSensorDriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
    RobotSensorDriverNode()
        : rclcpp_lifecycle::LifecycleNode("imu_driver"),
          hw_connected_(false), hw_error_(false),
          imu_count_(0), temp_count_(0)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 机器人IMU传感器驱动（生命周期版）已创建 ===");

        // 声明参数
        this->declare_parameter("device_path", std::string("/dev/imu0"));
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("publish_rate", 100);  // Hz
        this->declare_parameter("frame_id", std::string("imu_link"));

        // 创建 LifecyclePublisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
            "/imu/temperature", 10);
    }

    // ================================================================
    // on_configure: Unconfigured → Inactive
    //    传感器驱动中：
    //    - 打开设备文件 /dev/imu0
    //    - 设置波特率
    //    - 验证硬件连接
    //    - 配置采样参数
    // ================================================================
    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_configure ━━━");

        // 读取参数
        auto device = this->get_parameter("device_path").as_string();
        auto baudrate = this->get_parameter("baudrate").as_int();
        auto rate = this->get_parameter("publish_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        RCLCPP_INFO(this->get_logger(),
            "  设备: %s, 波特率: %ld, 采样率: %ldHz",
            device.c_str(), static_cast<long>(baudrate), static_cast<long>(rate));

        // 模拟硬件连接
        RCLCPP_INFO(this->get_logger(), "  正在连接硬件...");
        std::this_thread::sleep_for(500ms);  // 模拟连接耗时

        // 模拟硬件连接失败（第3次configure时失败，测试错误恢复）
        configure_attempt_++;
        if (configure_attempt_ == 3) {
            RCLCPP_ERROR(this->get_logger(),
                "  硬件连接失败！设备 %s 不可用", device.c_str());
            hw_error_ = true;
            return CallbackReturn::FAILURE;
        }

        hw_connected_ = true;
        RCLCPP_INFO(this->get_logger(), "  硬件连接成功");

        // 创建采样定时器（但不启动）
        auto period = std::chrono::milliseconds(1000 / rate);
        sample_timer_ = this->create_wall_timer(
            period,
            [this]() { sample_and_publish(); });
        sample_timer_->cancel();  // 不立即启动

        RCLCPP_INFO(this->get_logger(),
            "  配置成功 → Inactive (设备已连接但未采集)");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_activate: Inactive → Active
    //    传感器驱动中：
    //    - 启动数据采集
    //    - 使能数据流
    //    - 开始发布传感器数据
    // ================================================================
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_activate ━━━");

        if (!hw_connected_) {
            RCLCPP_ERROR(this->get_logger(),
                "  硬件未连接，无法激活！");
            return CallbackReturn::FAILURE;
        }

        // 启动采样定时器
        sample_timer_->reset();

        // 激活 LifecyclePublisher
        rclcpp_lifecycle::LifecycleNode::on_activate(state);

        RCLCPP_INFO(this->get_logger(),
            "  传感器已激活，开始采集和发布数据");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_deactivate: Active → Inactive
    //    传感器驱动中：
    //    - 暂停数据采集
    //    - 停止发布数据
    //    - 保持硬件连接（不断开！）
    // ================================================================
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_deactivate ━━━");

        // 暂停采样
        sample_timer_->cancel();

        // 停用 LifecyclePublisher
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

        RCLCPP_INFO(this->get_logger(),
            "  传感器已停用，暂停采集（硬件保持连接）");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_cleanup: Inactive → Unconfigured
    //    传感器驱动中：
    //    - 断开硬件连接
    //    - 释放设备资源
    //    - 重置内部状态
    // ================================================================
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_cleanup ━━━");

        // 断开硬件
        if (hw_connected_) {
            RCLCPP_INFO(this->get_logger(), "  正在断开硬件...");
            std::this_thread::sleep_for(200ms);
            hw_connected_ = false;
            RCLCPP_INFO(this->get_logger(), "  硬件已断开");
        }

        // 释放定时器
        sample_timer_.reset();

        // 重置计数器
        imu_count_ = 0;
        temp_count_ = 0;

        RCLCPP_INFO(this->get_logger(),
            "  清理完成 → Unconfigured (硬件已断开)");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_error: 错误处理
    //    传感器驱动中：
    //    - 尝试复位硬件
    //    - 释放可能损坏的资源
    //    - 如果恢复成功 → 回到 Unconfigured
    //    - 如果恢复失败 → Finalized
    // ================================================================
    CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_ERROR(this->get_logger(),
            "━━━ on_error ━━━");
        RCLCPP_ERROR(this->get_logger(),
            "  尝试错误恢复...");

        // 模拟硬件复位
        if (hw_connected_) {
            RCLCPP_INFO(this->get_logger(), "  复位硬件...");
            std::this_thread::sleep_for(1000ms);
            hw_connected_ = false;
            hw_error_ = false;
            sample_timer_.reset();
        }

        RCLCPP_INFO(this->get_logger(),
            "  错误恢复成功 → 将回到 Unconfigured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_shutdown ━━━");
        if (sample_timer_) {
            sample_timer_->cancel();
            sample_timer_.reset();
        }
        hw_connected_ = false;
        return CallbackReturn::SUCCESS;
    }

private:
    // 模拟采样和发布
    void sample_and_publish() {
        imu_count_++;

        // 模拟硬件故障（每500帧一次）
        if (imu_count_ == 500) {
            RCLCPP_ERROR(this->get_logger(),
                "  ⚠️ 硬件故障！IMU通信超时");
            hw_error_ = true;
            // 在实际应用中，这里应该触发错误处理
        }

        // 发布 IMU 数据
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = frame_id_;
        imu_pub_->publish(imu_msg);

        // 每100帧发布一次温度
        if (imu_count_ % 100 == 0) {
            auto temp_msg = sensor_msgs::msg::Temperature();
            temp_msg.header.stamp = this->now();
            temp_msg.header.frame_id = frame_id_;
            temp_msg.temperature = 25.0 + static_cast<double>(imu_count_ % 10) * 0.1;
            temp_pub_->publish(temp_msg);

            int count = imu_count_;
            RCLCPP_INFO(this->get_logger(),
                "[IMU驱动] #%d, 温度=%.1f°C, 发布器状态=%s",
                count, temp_msg.temperature,
                imu_pub_->is_activated() ? "激活" : "未激活");
        }
    }

    bool hw_connected_;
    bool hw_error_;
    int imu_count_;
    int temp_count_;
    int configure_attempt_ = 0;
    std::string frame_id_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::TimerBase::SharedPtr sample_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSensorDriverNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
