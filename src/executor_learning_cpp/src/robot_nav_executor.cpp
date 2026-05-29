/**
 * @file robot_nav_executor.cpp
 * @brief ⑨ 机器人导航栈 —— 执行器驱动的多优先级控制架构
 *
 * 知识点：
 * - 执行器架构在导航栈中的实际应用
 * - 速度控制（50Hz）绝不被路径规划（0.5Hz）阻塞
 * - 多回调组 + MultiThreadedExecutor 的实战组合
 * - 跨回调组数据共享与线程安全
 * - 仿真多传感器输入（IMU、激光、里程计）
 *
 * 架构设计：
 *
 *   ┌───────────────────────────────────────────────────┐
 *   │          MultiThreadedExecutor (3线程)             │
 *   │                                                    │
 *   │  ┌─────────────┐  ┌─────────────┐  ┌────────────┐│
 *   │  │ 控制回调组   │  │ 规划回调组   │  │ 传感器回调组││
 *   │  │ (50Hz 速度) │  │ (0.5Hz 路径) │  │ (100Hz IMU)││
 *   │  └─────────────┘  └─────────────┘  └────────────┘│
 *   │                                                    │
 *   │  ┌─────────────┐  ┌─────────────┐                │
 *   │  │ 激光回调组   │  │ 里程计回调组 │                │
 *   │  │ (10Hz 扫描) │  │ (50Hz 里程) │                │
 *   │  └─────────────┘  └─────────────┘                │
 *   └───────────────────────────────────────────────────┘
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <mutex>
#include <atomic>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class RobotNavExecutorNode : public rclcpp::Node {
public:
    RobotNavExecutorNode()
        : Node("robot_nav_executor"),
          cmd_count_(0), plan_count_(0), imu_count_(0),
          scan_count_(0), odom_count_(0),
          current_speed_(0.0), target_speed_(0.0),
          obstacle_detected_(false)
    {
        // ================================================================
        // 创建5个独立回调组 —— 每个优先级不同
        // ================================================================

        // 最高优先级：速度控制 50Hz
        control_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 高优先级：传感器数据
        imu_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        odom_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 中优先级：激光扫描
        scan_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 低优先级：路径规划（耗时）
        planning_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // ================================================================
        // 1. 速度控制 50Hz —— 绑定到 control_group_
        //    绝不能被其他回调阻塞！
        // ================================================================
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            20ms,  // 50Hz
            [this]() {
                std::lock_guard<std::mutex> lock(data_mutex_);

                // 简单速度控制：趋近目标速度
                double alpha = 0.1;
                current_speed_ += alpha * (target_speed_ - current_speed_);

                // 障碍物检测 → 急停
                if (obstacle_detected_) {
                    current_speed_ = 0.0;
                }

                // 发布速度指令
                auto cmd = geometry_msgs::msg::Twist();
                cmd.linear.x = current_speed_;
                cmd_pub_->publish(cmd);

                cmd_count_++;
                if (cmd_count_ % 100 == 0) {
                    // 注意：std::atomic 不可拷贝，必须用 .load() 先转为普通类型
                    int count = cmd_count_.load();
                    RCLCPP_INFO(this->get_logger(),
                        "[速度控制 50Hz] #%d, 速度=%.2f m/s, 目标=%.2f, 线程=%zu",
                        count, current_speed_, target_speed_, get_thread_id());
                }
            }, control_group_);

        // ================================================================
        // 2. IMU 数据订阅 100Hz —— 绑定到 imu_group_
        // ================================================================
        rclcpp::SubscriptionOptions imu_opts;
        imu_opts.callback_group = imu_group_;

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr /*msg*/) {
                imu_count_++;
                // IMU 数据处理（轻量）
                if (imu_count_ % 100 == 0) {
                    int count = imu_count_.load();
                    RCLCPP_DEBUG(this->get_logger(),
                        "[IMU 100Hz] #%d", count);
                }
            }, imu_opts);

        // ================================================================
        // 3. 里程计订阅 50Hz —— 绑定到 odom_group_
        // ================================================================
        rclcpp::SubscriptionOptions odom_opts;
        odom_opts.callback_group = odom_group_;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
                odom_count_++;
                std::lock_guard<std::mutex> lock(data_mutex_);
                // 更新机器人位置（轻量操作）
                if (odom_count_ % 50 == 0) {
                    int count = odom_count_.load();
                    RCLCPP_DEBUG(this->get_logger(),
                        "[里程计 50Hz] #%d", count);
                }
            }, odom_opts);

        // ================================================================
        // 4. 激光扫描 10Hz —— 绑定到 scan_group_
        // ================================================================
        rclcpp::SubscriptionOptions scan_opts;
        scan_opts.callback_group = scan_group_;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_count_++;
                // 障碍物检测（轻量）
                float min_range = std::numeric_limits<float>::infinity();
                for (const auto& r : msg->ranges) {
                    if (r < min_range && r > msg->range_min) {
                        min_range = r;
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    obstacle_detected_ = (min_range < 0.5);
                }
                if (scan_count_ % 10 == 0) {
                    int count = scan_count_.load();
                    RCLCPP_INFO(this->get_logger(),
                        "[激光 10Hz] #%d, 最近=%.2fm, 障碍=%s, 线程=%zu",
                        count, min_range,
                        obstacle_detected_ ? "是" : "否", get_thread_id());
                }
            }, scan_opts);

        // ================================================================
        // 5. 路径规划 0.5Hz —— 绑定到 planning_group_（耗时操作）
        //    规划回调可能耗时 1-2 秒，但绝不影响 50Hz 控制回调
        // ================================================================
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planned_path", 10);

        planning_timer_ = this->create_wall_timer(
            2s,  // 0.5Hz
            [this]() {
                plan_count_++;
                int pcount = plan_count_.load();
                RCLCPP_INFO(this->get_logger(),
                    "[路径规划 0.5Hz] #%d 开始规划...", pcount);

                // 模拟耗时规划（1秒）
                std::this_thread::sleep_for(1000ms);

                // 发布规划路径
                auto path = nav_msgs::msg::Path();
                path.header.stamp = this->now();
                path.header.frame_id = "map";
                path_pub_->publish(path);

                // 更新目标速度
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    target_speed_ = obstacle_detected_ ? 0.0 : 0.5;
                }

                RCLCPP_INFO(this->get_logger(),
                    "[路径规划 0.5Hz] #%d 完成, 目标速度=%.2f, 线程=%zu",
                    pcount, target_speed_, get_thread_id());
            }, planning_group_);

        // ================================================================
        // 6. 仿真传感器发布器 —— 模拟传感器输入
        // ================================================================
        sim_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);
        sim_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", 10);
        sim_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", 10);

        sim_imu_timer_ = this->create_wall_timer(10ms, [this]() {
            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp = this->now();
            sim_imu_pub_->publish(msg);
        });

        sim_scan_timer_ = this->create_wall_timer(100ms, [this]() {
            auto msg = sensor_msgs::msg::LaserScan();
            msg.header.stamp = this->now();
            msg.range_min = 0.1f;
            msg.range_max = 10.0f;
            msg.ranges.resize(360, 2.0f);  // 模拟2m外无障碍
            sim_scan_pub_->publish(msg);
        });

        sim_odom_timer_ = this->create_wall_timer(20ms, [this]() {
            auto msg = nav_msgs::msg::Odometry();
            msg.header.stamp = this->now();
            sim_odom_pub_->publish(msg);
        });

        RCLCPP_INFO(this->get_logger(),
            "=== 机器人导航栈执行器节点已启动 ===");
        RCLCPP_INFO(this->get_logger(),
            "  控制组(50Hz) | IMU组(100Hz) | 里程计组(50Hz) | 激光组(10Hz) | 规划组(0.5Hz)");
        RCLCPP_INFO(this->get_logger(),
            "  路径规划1秒耗时不影响速度控制50Hz输出");
    }

private:
    std::size_t get_thread_id() {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr control_group_;
    rclcpp::CallbackGroup::SharedPtr imu_group_;
    rclcpp::CallbackGroup::SharedPtr odom_group_;
    rclcpp::CallbackGroup::SharedPtr scan_group_;
    rclcpp::CallbackGroup::SharedPtr planning_group_;

    // 共享数据（跨回调组访问，需要互斥锁）
    std::mutex data_mutex_;
    std::atomic<int> cmd_count_;
    std::atomic<int> plan_count_;
    std::atomic<int> imu_count_;
    std::atomic<int> scan_count_;
    std::atomic<int> odom_count_;
    double current_speed_;
    double target_speed_;
    bool obstacle_detected_;

    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // 仿真发布器
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr sim_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sim_scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sim_odom_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr planning_timer_;
    rclcpp::TimerBase::SharedPtr sim_imu_timer_;
    rclcpp::TimerBase::SharedPtr sim_scan_timer_;
    rclcpp::TimerBase::SharedPtr sim_odom_timer_;

    // 订阅
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotNavExecutorNode>();

    // ================================================================
    // MultiThreadedExecutor + 5个回调组
    //    线程数建议：回调组数量（5）或更多
    //    确保最高优先级的控制回调有足够线程可用
    // ================================================================
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 5);
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(),
        "使用 MultiThreadedExecutor (5线程) 驱动导航栈");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
