/**
 * ===========================================================================
 *  ⑦ 机器人导航栈 —— 实际应用案例 (C++版)
 * ===========================================================================
 *
 * 回调组设计策略（优先级驱动）：
 *   ┌──────────────────┬─────────────┬──────────┬──────────────────────────┐
 *   │ 功能模块         │ 频率        │ 回调组   │ 原因                     │
 *   ├──────────────────┼─────────────┼──────────┼──────────────────────────┤
 *   │ 速度控制/里程计  │ 50Hz        │ 独立ME   │ 最高实时性，不能被阻塞   │
 *   │ 避障检测         │ 10Hz        │ 独立ME   │ 安全关键，不能被阻塞     │
 *   │ 路径规划         │ 1Hz         │ Reentrant│ 耗时，允许并发           │
 *   │ 地图更新         │ 0.5Hz       │ 独立ME   │ 耗时但低频，保护地图数据 │
 *   └──────────────────┴─────────────┴──────────┴──────────────────────────┘
 *
 * C++ 导航相关消息类型：
 *   - geometry_msgs::msg::Twist      → 速度指令
 *   - geometry_msgs::msg::PoseStamped → 目标位姿
 *   - nav_msgs::msg::Odometry         → 里程计
 *   - nav_msgs::msg::OccupancyGrid    → 栅格地图
 *   - nav_msgs::msg::Path             → 路径
 *   - sensor_msgs::msg::LaserScan     → 激光扫描
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class RobotNavStackNode : public rclcpp::Node
{
public:
    RobotNavStackNode()
        : Node("robot_nav_stack"),
          current_speed_(0.0), target_speed_(0.3), angular_speed_(0.0),
          min_obstacle_dist_(std::numeric_limits<double>::infinity()),
          emergency_stop_(false),
          has_goal_(false), goal_x_(0.0), goal_y_(0.0), planning_in_progress_(false),
          odom_x_(0.0), odom_y_(0.0), odom_theta_(0.0),
          control_count_(0), safety_count_(0), planning_count_(0), mapping_count_(0)
    {
        // ================================================================
        // 1. 回调组设计（按优先级）
        // ================================================================

        // 最高优先级：速度控制 + 里程计 (50Hz)
        control_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 高优先级：避障检测 (10Hz)
        safety_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 低优先级：路径规划 (1Hz) —— Reentrant 允许并发
        planning_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // 后台优先级：地图更新 (0.5Hz)
        mapping_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // ================================================================
        // 2. 发布者
        // ================================================================
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // ================================================================
        // 3. 订阅者
        // ================================================================

        // 激光雷达 —— safety_group
        rclcpp::SubscriptionOptions scan_opts;
        scan_opts.callback_group = safety_group_;
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&RobotNavStackNode::scan_callback, this, std::placeholders::_1),
            scan_opts);

        // 目标位姿 —— planning_group
        rclcpp::SubscriptionOptions goal_opts;
        goal_opts.callback_group = planning_group_;
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&RobotNavStackNode::goal_callback, this, std::placeholders::_1),
            goal_opts);

        // ================================================================
        // 4. 定时器
        // ================================================================

        // 速度控制 —— 50Hz，最高优先级
        control_timer_ = this->create_wall_timer(
            20ms, std::bind(&RobotNavStackNode::control_callback, this),
            control_group_);

        // 避障 —— 10Hz
        safety_timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotNavStackNode::safety_callback, this),
            safety_group_);

        // 路径规划 —— 1Hz
        planning_timer_ = this->create_wall_timer(
            1s, std::bind(&RobotNavStackNode::planning_callback, this),
            planning_group_);

        // 地图更新 —— 0.5Hz
        mapping_timer_ = this->create_wall_timer(
            2s, std::bind(&RobotNavStackNode::mapping_callback, this),
            mapping_group_);

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "机器人导航栈启动 (C++)");
        RCLCPP_INFO(this->get_logger(), "  速度控制 → control_group (50Hz, MutuallyExclusive)");
        RCLCPP_INFO(this->get_logger(), "  避障检测 → safety_group (10Hz, MutuallyExclusive)");
        RCLCPP_INFO(this->get_logger(), "  路径规划 → planning_group (1Hz, Reentrant)");
        RCLCPP_INFO(this->get_logger(), "  地图更新 → mapping_group (0.5Hz, MutuallyExclusive)");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    // ================================================================
    // 最高优先级：速度控制 + 里程计
    // ================================================================
    void control_callback()
    {
        control_count_++;

        // 更新里程计（简单积分）
        double dt = 0.02;  // 50Hz
        odom_x_ += current_speed_ * std::cos(odom_theta_) * dt;
        odom_y_ += current_speed_ * std::sin(odom_theta_) * dt;
        odom_theta_ += angular_speed_ * dt;

        // 发布里程计
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = odom_x_;
        odom.pose.pose.position.y = odom_y_;
        odom_pub_->publish(odom);

        // 速度控制：平滑加减速
        if (emergency_stop_) {
            current_speed_ = 0.0;
            angular_speed_ = 0.0;
        } else {
            double speed_diff = target_speed_ - current_speed_;
            double accel = std::max(-0.1, std::min(0.1, speed_diff));
            current_speed_ += accel;
        }

        // 发布速度指令
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = current_speed_;
        cmd.angular.z = angular_speed_;
        cmd_vel_pub_->publish(cmd);
    }

    // ================================================================
    // 高优先级：避障检测
    // ================================================================
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& r : msg->ranges) {
            if (r > msg->range_min && r < msg->range_max && r < min_dist) {
                min_dist = r;
            }
        }
        std::lock_guard<std::mutex> lock(mutex_);
        min_obstacle_dist_ = min_dist;
    }

    void safety_callback()
    {
        safety_count_++;

        double min_dist;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            min_dist = min_obstacle_dist_;
        }

        if (min_dist < 0.3) {
            emergency_stop_ = true;
            RCLCPP_ERROR(this->get_logger(),
                "[避障] 紧急停止！障碍物距离: %.2fm", min_dist);
        } else if (min_dist < 0.5) {
            target_speed_ = 0.1;
            emergency_stop_ = false;
            RCLCPP_WARN(this->get_logger(),
                "[避障] 减速！障碍物距离: %.2fm", min_dist);
        } else {
            emergency_stop_ = false;
            target_speed_ = 0.3;
        }
    }

    // ================================================================
    // 低优先级：路径规划
    // ================================================================
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        has_goal_ = true;
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        RCLCPP_INFO(this->get_logger(),
            "[规划] 收到新目标: (%.1f, %.1f)", goal_x_, goal_y_);
    }

    void planning_callback()
    {
        planning_count_++;
        if (!has_goal_) return;

        planning_in_progress_ = true;
        RCLCPP_INFO(this->get_logger(), "[规划] 开始路径规划...");

        // 模拟耗时规划算法
        std::this_thread::sleep_for(500ms);

        // 发布规划路径
        auto path = nav_msgs::msg::Path();
        path.header.stamp = this->now();
        path.header.frame_id = "map";
        for (int i = 0; i <= 10; ++i) {
            auto pose = geometry_msgs::msg::PoseStamped();
            pose.header = path.header;
            double t = i / 10.0;
            pose.pose.position.x = odom_x_ + t * (goal_x_ - odom_x_);
            pose.pose.position.y = odom_y_ + t * (goal_y_ - odom_y_);
            path.poses.push_back(pose);
        }
        path_pub_->publish(path);

        planning_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(),
            "[规划] 完成，路径 %zu 个点", path.poses.size());
    }

    // ================================================================
    // 后台优先级：地图更新
    // ================================================================
    void mapping_callback()
    {
        mapping_count_++;

        // 模拟地图更新
        std::this_thread::sleep_for(200ms);

        auto map_msg = nav_msgs::msg::OccupancyGrid();
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";
        map_msg.info.resolution = 0.05f;
        map_msg.info.width = 100;
        map_msg.info.height = 100;
        map_msg.info.origin.position.x = -2.5;
        map_msg.info.origin.position.y = -2.5;
        map_msg.data.resize(100 * 100, 0);  // 空地图
        map_pub_->publish(map_msg);

        if (mapping_count_ % 5 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[地图] 已更新 %d 次", mapping_count_);
        }
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr control_group_;
    rclcpp::CallbackGroup::SharedPtr safety_group_;
    rclcpp::CallbackGroup::SharedPtr planning_group_;
    rclcpp::CallbackGroup::SharedPtr mapping_group_;

    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    rclcpp::TimerBase::SharedPtr planning_timer_;
    rclcpp::TimerBase::SharedPtr mapping_timer_;

    // 线程安全
    std::mutex mutex_;

    // 速度控制状态
    double current_speed_;
    double target_speed_;
    double angular_speed_;

    // 避障状态
    double min_obstacle_dist_;
    bool emergency_stop_;

    // 路径规划状态
    bool has_goal_;
    double goal_x_, goal_y_;
    bool planning_in_progress_;

    // 里程计
    double odom_x_, odom_y_, odom_theta_;

    // 统计
    int control_count_;
    int safety_count_;
    int planning_count_;
    int mapping_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotNavStackNode>();

    // 使用 MultiThreadedExecutor，4个回调组充分并发
    // 速度控制(50Hz) 绝不会被路径规划(1Hz) 阻塞
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);

    executor.add_node(node);

    try {
        executor.spin();
    } catch (...) {}

    rclcpp::shutdown();
    return 0;
}
