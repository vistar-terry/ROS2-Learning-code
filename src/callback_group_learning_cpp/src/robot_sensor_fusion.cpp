/**
 * ===========================================================================
 *  ⑥ 机器人传感器融合 —— 实际应用案例 (C++版)
 * ===========================================================================
 *
 * 场景描述：
 *   移动机器人配备多种传感器：
 *   - 激光雷达 (10Hz)：用于避障
 *   - IMU (100Hz)：用于姿态估计
 *   - 相机 (30Hz)：用于目标检测
 *
 * 回调组设计（与Python版相同策略）：
 *   1. 激光雷达回调 → 独立 MutuallyExclusive（保护障碍物列表）
 *   2. IMU 回调    → 独立 MutuallyExclusive（最高频，不能被阻塞）
 *   3. 相机回调    → Reentrant（允许并发处理多帧）
 *   4. 融合回调    → 独立 MutuallyExclusive（读取所有数据，原子性）
 *
 * C++ 线程安全：
 *   - std::mutex + std::lock_guard<std::mutex>
 *   - 跨回调组读取共享数据时需要加锁
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <sstream>

using namespace std::chrono_literals;

class RobotSensorFusionNode : public rclcpp::Node
{
public:
    RobotSensorFusionNode()
        : Node("robot_sensor_fusion"),
          lidar_count_(0), imu_count_(0), camera_count_(0), fusion_count_(0),
          min_distance_(std::numeric_limits<double>::infinity()),
          roll_(0.0), pitch_(0.0), yaw_(0.0)
    {
        // ================================================================
        // 1. 回调组设计（核心！）
        //    每种传感器的回调放入独立的回调组
        //    避免高频传感器被低频传感器阻塞
        // ================================================================

        // 激光雷达回调组 —— MutuallyExclusive（保护障碍物数据）
        lidar_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // IMU 回调组 —— 独立 MutuallyExclusive（100Hz，不能被阻塞）
        imu_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 相机回调组 —— Reentrant（允许并发处理）
        camera_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // 融合回调组 —— 独立 MutuallyExclusive
        fusion_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // ================================================================
        // 2. 订阅传感器数据
        // ================================================================

        // 激光雷达订阅
        rclcpp::SubscriptionOptions lidar_opts;
        lidar_opts.callback_group = lidar_group_;
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&RobotSensorFusionNode::lidar_callback, this, std::placeholders::_1),
            lidar_opts);

        // IMU 订阅
        rclcpp::SubscriptionOptions imu_opts;
        imu_opts.callback_group = imu_group_;
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&RobotSensorFusionNode::imu_callback, this, std::placeholders::_1),
            imu_opts);

        // 相机订阅
        rclcpp::SubscriptionOptions camera_opts;
        camera_opts.callback_group = camera_group_;
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&RobotSensorFusionNode::camera_callback, this, std::placeholders::_1),
            camera_opts);

        // ================================================================
        // 3. 融合定时器 —— 周期性融合所有传感器数据
        // ================================================================
        fusion_timer_ = this->create_wall_timer(
            100ms,  // 10Hz 融合
            std::bind(&RobotSensorFusionNode::fusion_callback, this),
            fusion_group_);

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "机器人传感器融合节点启动 (C++)");
        RCLCPP_INFO(this->get_logger(), "  激光雷达 → lidar_group (MutuallyExclusive)");
        RCLCPP_INFO(this->get_logger(), "  IMU      → imu_group (MutuallyExclusive, 独立)");
        RCLCPP_INFO(this->get_logger(), "  相机     → camera_group (Reentrant)");
        RCLCPP_INFO(this->get_logger(), "  融合     → fusion_group (MutuallyExclusive, 独立)");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    /**
     * 激光雷达回调 —— 在 lidar_group 中
     * MutuallyExclusive 保护 min_distance_ 和 obstacle_angles_
     */
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        lidar_count_++;

        // 提取最近障碍物距离
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& r : msg->ranges) {
            if (r > msg->range_min && r < msg->range_max && r < min_dist) {
                min_dist = r;
            }
        }
        min_distance_ = min_dist;

        // 提取障碍物角度
        obstacle_angles_.clear();
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < 1.0 && msg->ranges[i] > msg->range_min) {
                double angle = msg->angle_min + i * msg->angle_increment;
                obstacle_angles_.push_back(angle);
            }
        }
    }

    /**
     * IMU 回调 —— 在 imu_group 中（独立回调组）
     * 由于是独立的 MutuallyExclusive 组，
     * 这个回调不会被激光雷达或相机回调阻塞
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_count_++;

        // 从四元数计算欧拉角
        const auto& q = msg->orientation;

        // Roll
        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll_ = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        sinp = std::max(-1.0, std::min(1.0, sinp));
        pitch_ = std::asin(sinp);

        // Yaw
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    /**
     * 相机回调 —— 在 camera_group (Reentrant) 中
     * 使用 Reentrant 允许多帧并发处理
     * 共享数据需要锁保护！
     */
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr /*msg*/)
    {
        camera_count_++;

        // 模拟图像处理耗时
        std::this_thread::sleep_for(50ms);

        // 模拟目标检测结果（需要锁保护，因为 Reentrant 允许并发）
        std::lock_guard<std::mutex> lock(mutex_);
        detected_objects_.clear();
        detected_objects_.push_back({"person", 0.95, 2.5});
        detected_objects_.push_back({"chair", 0.87, 1.8});
    }

    /**
     * 融合回调 —— 在 fusion_group 中
     * 读取所有传感器的最新数据，做综合判断
     */
    void fusion_callback()
    {
        fusion_count_++;

        // 读取各传感器数据（跨回调组读取，需要锁保护）
        double min_dist;
        int num_obstacles;
        int num_detected;
        double r, p, y;
        bool imu_ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            min_dist = min_distance_;
            num_obstacles = static_cast<int>(obstacle_angles_.size());
            num_detected = static_cast<int>(detected_objects_.size());
            r = roll_;
            p = pitch_;
            y = yaw_;
            imu_ok = (imu_count_ > 0);
        }

        // 如果 IMU 无数据，提示警告
        if (!imu_ok) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "IMU 数据尚未收到，姿态信息可能不准确");
        }

        // 综合判断
        std::string danger_level = "安全";
        if (min_dist < 0.3) {
            danger_level = "危险！紧急停止";
        } else if (min_dist < 0.5) {
            danger_level = "警告！减速避障";
        } else if (min_dist < 1.0) {
            danger_level = "注意";
        }

        // 每10次融合输出一次日志
        if (fusion_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[融合 #%d] 最近障碍: %.2fm, 障碍数: %d, "
                "姿态: R=%.1f° P=%.1f° Y=%.1f°, "
                "检测目标: %d, 状态: %s",
                fusion_count_, min_dist, num_obstacles,
                r * 180.0 / M_PI, p * 180.0 / M_PI, y * 180.0 / M_PI,
                num_detected, danger_level.c_str());
        }
    }

    // ================================================================
    // 回调组
    // ================================================================
    rclcpp::CallbackGroup::SharedPtr lidar_group_;
    rclcpp::CallbackGroup::SharedPtr imu_group_;
    rclcpp::CallbackGroup::SharedPtr camera_group_;
    rclcpp::CallbackGroup::SharedPtr fusion_group_;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::TimerBase::SharedPtr fusion_timer_;

    // 线程安全
    std::mutex mutex_;

    // 统计（声明在前，与初始化列表顺序一致）
    int lidar_count_;
    int imu_count_;
    int camera_count_;
    int fusion_count_;

    // 激光数据
    double min_distance_;
    std::vector<double> obstacle_angles_;

    // IMU 数据
    double roll_, pitch_, yaw_;

    // 相机数据
    struct DetectedObject {
        std::string cls;
        double confidence;
        double distance;
    };
    std::vector<DetectedObject> detected_objects_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotSensorFusionNode>();

    // 使用 MultiThreadedExecutor，4个独立回调组充分并发
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);

    executor.add_node(node);

    try {
        executor.spin();
    } catch (...) {}

    rclcpp::shutdown();
    return 0;
}
