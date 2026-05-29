/**
 * ===========================================================================
 *  ⑤ 多节点单执行器 (C++版)
 * ===========================================================================
 *
 * 核心知识点：
 *   - 一个 Executor 可以管理多个节点
 *   - executor.spin_once() vs executor.spin() 的区别
 *   - 手动执行器循环：可以插入自定义逻辑
 *   - 多节点不同回调组的交互
 *
 * C++ 多节点执行器 API:
 *   - executor.add_node(node)
 *   - executor.remove_node(node)
 *   - executor.spin() / executor.spin_once(timeout)
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>

using namespace std::chrono_literals;

/**
 * 模拟传感器节点 —— 发布激光雷达数据
 */
class SensorNode : public rclcpp::Node
{
public:
    SensorNode() : Node("sensor_node"), scan_count_(0)
    {
        // 使用 Reentrant 回调组，允许发布和定时器并发
        reentrant_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/sensor_status", 10);

        // 模拟传感器数据发布（10Hz）
        scan_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&SensorNode::publish_scan, this),
            reentrant_group_);

        // 状态发布（1Hz）
        status_timer_ = this->create_wall_timer(
            1s,
            std::bind(&SensorNode::publish_status, this),
            reentrant_group_);

        RCLCPP_INFO(this->get_logger(), "[传感器节点] 启动，10Hz发布激光数据");
    }

private:
    void publish_scan()
    {
        scan_count_++;
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser_frame";
        msg.angle_min = -3.14159f;
        msg.angle_max = 3.14159f;
        msg.angle_increment = 0.0174533f;
        msg.range_min = 0.1f;
        msg.range_max = 10.0f;
        // 模拟360个射线
        msg.ranges.resize(360);
        for (size_t i = 0; i < 360; ++i) {
            msg.ranges[i] = 1.0f + 0.1f * static_cast<float>(i % 10);
        }
        scan_pub_->publish(msg);
    }

    void publish_status()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "在线, 已发布 " + std::to_string(scan_count_) + " 帧扫描";
        status_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[传感器] 状态: %s", msg.data.c_str());
    }

    rclcpp::CallbackGroup::SharedPtr reentrant_group_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    int scan_count_;
};

/**
 * 数据处理节点 —— 订阅传感器数据并处理
 */
class ProcessingNode : public rclcpp::Node
{
public:
    ProcessingNode() : Node("processing_node"), obstacle_count_(0), sensor_online_(false)
    {
        // ================================================================
        // 回调组设计（与Python版相同策略）
        // ================================================================
        // 订阅回调使用 MutuallyExclusive：保护共享的障碍物列表
        subscriber_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        // 处理定时器使用 Reentrant：可以与订阅并发
        processing_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // 订阅激光数据
        rclcpp::SubscriptionOptions scan_opts;
        scan_opts.callback_group = subscriber_group_;
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ProcessingNode::scan_callback, this, std::placeholders::_1),
            scan_opts);

        // 订阅传感器状态
        rclcpp::SubscriptionOptions status_opts;
        status_opts.callback_group = subscriber_group_;
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sensor_status", 10,
            std::bind(&ProcessingNode::status_callback, this, std::placeholders::_1),
            status_opts);

        // 处理定时器
        process_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&ProcessingNode::process_callback, this),
            processing_group_);

        RCLCPP_INFO(this->get_logger(), "[处理节点] 启动");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 统计障碍物数量（距离小于阈值的点）
        float threshold = 0.5f;
        int count = 0;
        for (const auto& r : msg->ranges) {
            if (r < threshold && r > msg->range_min) {
                count++;
            }
        }
        // subscriber_group_ 是 MutuallyExclusive，保护 obstacle_count_
        obstacle_count_ = count;
    }

    void status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 与 scan_callback 在同一个互斥回调组，
        // 所以不会与 scan_callback 并发执行，共享数据安全
        sensor_online_ = msg->data.find("在线") != std::string::npos;
        RCLCPP_INFO(this->get_logger(), "[处理] 传感器状态更新: %s", msg->data.c_str());
    }

    void process_callback()
    {
        if (sensor_online_) {
            RCLCPP_INFO(this->get_logger(),
                "[处理] 障碍物点数: %d", obstacle_count_);
        } else {
            RCLCPP_WARN(this->get_logger(), "[处理] 传感器离线，无法处理");
        }
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr subscriber_group_;
    rclcpp::CallbackGroup::SharedPtr processing_group_;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr process_timer_;

    // 共享数据（受 subscriber_group_ 的 MutuallyExclusive 保护）
    int obstacle_count_;
    bool sensor_online_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto sensor_node = std::make_shared<SensorNode>();
    auto processing_node = std::make_shared<ProcessingNode>();

    // ================================================================
    // 一个 MultiThreadedExecutor 管理两个节点
    // 两个节点的回调可以交叉并发
    // 但同一 MutuallyExclusive 回调组内的回调仍然串行
    // ================================================================
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);
    executor.add_node(sensor_node);
    executor.add_node(processing_node);

    try {
        // 手动执行器循环（替代 executor.spin()）
        // 可以在循环中插入自定义逻辑
        while (rclcpp::ok()) {
            // spin_once 执行一批就绪的回调
            executor.spin_once(100ms);

            // 可以在这里插入自定义逻辑
            // 例如：检查节点状态、执行非ROS任务等
        }
    } catch (...) {}

    rclcpp::shutdown();
    return 0;
}
