/**
 * @file robot_nav_lifecycle.cpp
 * @brief ⑨ 机器人导航子系统 —— 生命周期控制的安全关键系统
 *
 * 知识点：
 * - 导航栈中生命周期节点的应用
 * - 多个生命周期节点的协同启停
 * - 安全关键系统：急停时 deactivate 所有执行器
 * - 配置参数验证（on_configure 中检查参数合法性）
 * - 资源申请失败的处理
 *
 * 架构：
 *   LifecycleController（生命周期节点）
 *     ├── 速度控制（activate时启动，deactivate时停止）
 *     ├── 路径规划（activate时启动，deactivate时停止）
 *     └── 安全监控（始终运行）
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class RobotNavLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    RobotNavLifecycleNode()
        : rclcpp_lifecycle::LifecycleNode("nav_controller"),
          cmd_count_(0), plan_count_(0), odom_count_(0),
          scan_count_(0), emergency_stop_(false),
          current_speed_(0.0), target_speed_(0.5)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 机器人导航控制器（生命周期版）已创建 ===");

        // 声明参数
        this->declare_parameter("control_rate", 50);    // Hz
        this->declare_parameter("planning_rate", 0.5);  // Hz
        this->declare_parameter("max_speed", 1.0);      // m/s
        this->declare_parameter("safety_distance", 0.3); // m

        // 回调组
        control_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        sensor_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        planning_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 创建 LifecyclePublisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planned_path", 10);

        // 创建订阅（始终接收数据，但只在 Active 状态处理）
        rclcpp::SubscriptionOptions sensor_opts;
        sensor_opts.callback_group = sensor_group_;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_count_++;
                // 安全监控：始终检查障碍物（无论什么状态！）
                float min_range = std::numeric_limits<float>::infinity();
                for (const auto& r : msg->ranges) {
                    if (r < min_range && r > msg->range_min) {
                        min_range = r;
                    }
                }
                auto safety_dist = this->get_parameter("safety_distance").as_double();
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    emergency_stop_ = (min_range < static_cast<float>(safety_dist));
                }

                // 仅在 Active 状态下执行额外处理
                auto state = this->get_current_state();
                if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    if (emergency_stop_ && scan_count_ % 5 == 0) {
                        int count = scan_count_;
                        RCLCPP_WARN(this->get_logger(),
                            "[安全] 障碍物过近(%.2fm<%.2fm)! 急停! 扫描=%d",
                            min_range, safety_dist, count);
                    }
                }
            }, sensor_opts);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
                odom_count_++;
            }, sensor_opts);

        // 仿真数据发布
        sim_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", 10);
        sim_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", 10);

        sim_scan_timer_ = this->create_wall_timer(100ms, [this]() {
            auto msg = sensor_msgs::msg::LaserScan();
            msg.header.stamp = this->now();
            msg.range_min = 0.1f;
            msg.range_max = 10.0f;
            msg.ranges.resize(360, 2.0f);
            sim_scan_pub_->publish(msg);
        });

        sim_odom_timer_ = this->create_wall_timer(20ms, [this]() {
            auto msg = nav_msgs::msg::Odometry();
            msg.header.stamp = this->now();
            sim_odom_pub_->publish(msg);
        });
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(), "━━━ on_configure ━━━");

        // ================================================================
        // 参数验证 —— 在 configure 阶段验证所有参数
        //    如果参数不合法，返回 FAILURE
        // ================================================================
        auto max_speed = this->get_parameter("max_speed").as_double();
        auto control_rate = this->get_parameter("control_rate").as_int();
        auto safety_dist = this->get_parameter("safety_distance").as_double();

        if (max_speed <= 0.0) {
            RCLCPP_ERROR(this->get_logger(),
                "  参数错误: max_speed=%.2f 必须>0", max_speed);
            return CallbackReturn::FAILURE;
        }
        if (control_rate <= 0 || control_rate > 1000) {
            RCLCPP_ERROR(this->get_logger(),
                "  参数错误: control_rate=%ld 必须在(0,1000]范围内", static_cast<long>(control_rate));
            return CallbackReturn::FAILURE;
        }
        if (safety_dist <= 0.0) {
            RCLCPP_ERROR(this->get_logger(),
                "  参数错误: safety_distance=%.2f 必须>0", safety_dist);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(),
            "  参数验证通过: max_speed=%.2f, control_rate=%ldHz, safety_dist=%.2fm",
            max_speed, static_cast<long>(control_rate), safety_dist);

        // 创建定时器（但不启动）
        int rate = this->get_parameter("control_rate").as_int();
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / rate),
            [this]() { control_callback(); },
            control_group_);
        control_timer_->cancel();

        planning_timer_ = this->create_wall_timer(
            2s,
            [this]() { planning_callback(); },
            planning_group_);
        planning_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "  配置成功 → Inactive");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(), "━━━ on_activate ━━━");

        // 启动控制循环
        control_timer_->reset();
        planning_timer_->reset();

        // 激活发布器
        rclcpp_lifecycle::LifecycleNode::on_activate(state);

        RCLCPP_INFO(this->get_logger(),
            "  导航控制器已激活，开始控制循环");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(), "━━━ on_deactivate ━━━");

        // ================================================================
        // 安全停机：deactivate 时必须发送零速度！
        //    这是最关键的安全措施
        // ================================================================
        auto stop_cmd = geometry_msgs::msg::Twist();
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;

        // ⚠️ 在 deactivate 时，LifecyclePublisher 还未完全停用
        //    可以在此发布最后的停止命令
        cmd_pub_->publish(stop_cmd);
        RCLCPP_INFO(this->get_logger(), "  已发送零速度指令（安全停机）");

        // 停止定时器
        control_timer_->cancel();
        planning_timer_->cancel();

        // 重置速度
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_speed_ = 0.0;
        }

        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

        RCLCPP_INFO(this->get_logger(), "  导航控制器已停用");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(), "━━━ on_cleanup ━━━");
        control_timer_.reset();
        planning_timer_.reset();
        cmd_count_ = 0;
        plan_count_ = 0;
        odom_count_ = 0;
        scan_count_ = 0;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(), "━━━ on_shutdown ━━━");
        if (control_timer_) control_timer_->cancel();
        if (planning_timer_) planning_timer_->cancel();
        return CallbackReturn::SUCCESS;
    }

private:
    void control_callback() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 急停检查
        if (emergency_stop_) {
            current_speed_ = 0.0;
        } else {
            // 速度控制
            double alpha = 0.1;
            current_speed_ += alpha * (target_speed_ - current_speed_);
        }

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = current_speed_;
        cmd_pub_->publish(cmd);

        cmd_count_++;
        if (cmd_count_ % 100 == 0) {
            int count = cmd_count_;
            RCLCPP_INFO(this->get_logger(),
                "[控制 50Hz] #%d, 速度=%.2f, 急停=%d",
                count, current_speed_, emergency_stop_ ? 1 : 0);
        }
    }

    void planning_callback() {
        plan_count_++;
        int count = plan_count_;
        RCLCPP_INFO(this->get_logger(),
            "[规划 0.5Hz] #%d, 开始规划...", count);

        std::this_thread::sleep_for(500ms);

        auto path = nav_msgs::msg::Path();
        path.header.stamp = this->now();
        path.header.frame_id = "map";
        path_pub_->publish(path);

        RCLCPP_INFO(this->get_logger(),
            "[规划 0.5Hz] #%d 完成", count);
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr control_group_;
    rclcpp::CallbackGroup::SharedPtr sensor_group_;
    rclcpp::CallbackGroup::SharedPtr planning_group_;

    // 共享数据
    std::mutex data_mutex_;
    int cmd_count_;
    int plan_count_;
    int odom_count_;
    int scan_count_;
    bool emergency_stop_;
    double current_speed_;
    double target_speed_;

    // LifecyclePublisher
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // 订阅
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr planning_timer_;

    // 仿真
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sim_scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sim_odom_pub_;
    rclcpp::TimerBase::SharedPtr sim_scan_timer_;
    rclcpp::TimerBase::SharedPtr sim_odom_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotNavLifecycleNode>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3);
    executor.add_node(node->get_node_base_interface());

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
