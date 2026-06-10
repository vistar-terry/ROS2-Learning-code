/**
 * @file multi_executor_arch.cpp
 * @brief ⑦ 多执行器架构 —— 每个节点独立执行器 + 多线程
 *
 * 知识点：
 * - 多执行器架构的设计模式
 * - 实时关键节点 vs 非实时节点的隔离
 * - std::thread 驱动多个执行器
 * - 执行器间通过 topic/service 通信
 * - 优雅关闭（信号处理 + executor.cancel()）
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

// ================================================================
// 实时传感器节点 —— 需要 100Hz 稳定输出
//    放在独立的 SingleThreadedExecutor 中，不受其他节点影响
// ================================================================
class RealtimeSensorNode : public rclcpp::Node
{
public:
    RealtimeSensorNode()
        : Node("realtime_sensor"), pub_count_(0)
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);

        // 100Hz 严格定时
        imu_timer_ = this->create_wall_timer(
            10ms,
            [this]()
            {
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = this->now();
                msg.header.frame_id = "imu_link";
                pub_count_++;

                imu_pub_->publish(msg);

                if (pub_count_ % 200 == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[Realtime sensor] published %d IMU frames, thread=%zu",
                                pub_count_, get_thread_id());
                }
            });

        RCLCPP_INFO(this->get_logger(),
                    "[Realtime sensor node] started (100Hz, independent executor)");
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int pub_count_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
};

// ================================================================
// 数据处理节点 —— 耗时计算，不影响传感器
//    放在独立的 MultiThreadedExecutor 中
// ================================================================
class DataProcessingNode : public rclcpp::Node
{
public:
    DataProcessingNode()
        : Node("data_processing"), process_count_(0)
    {
        // 创建独立回调组 —— 订阅和定时器不互相阻塞
        sub_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 订阅 IMU
        rclcpp::SubscriptionOptions opts;
        opts.callback_group = sub_group_;
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                process_count_++;
                // 每50帧做一次耗时处理
                if (process_count_ % 50 == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "[Data processing] frame #%d, start heavy computation...", process_count_);

                    std::this_thread::sleep_for(200ms); // 模拟耗时
                    RCLCPP_INFO(this->get_logger(), "[Data processing] msg linear acceleration x: %lf, y: %lf, z: %lf",
                                msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                    RCLCPP_INFO(this->get_logger(), "[Data processing] msg angular velocity x: %lf, y: %lf, z: %lf",
                                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                    RCLCPP_INFO(this->get_logger(), "[Data processing] heavy computation completed");
                }
            },
            opts);

        // 状态报告
        status_timer_ = this->create_wall_timer(
            2s,
            [this]()
            {
                RCLCPP_INFO(this->get_logger(), "[Data processing] processed %d IMU frames", process_count_);
            },
            timer_group_);

        RCLCPP_INFO(this->get_logger(), "[Data processing node] started (independent executor)");
    }

private:
    int process_count_;
    rclcpp::CallbackGroup::SharedPtr sub_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

// ================================================================
// 监控节点 —— 低频监控
//    放在独立的 SingleThreadedExecutor 中
// ================================================================
class MonitorNode : public rclcpp::Node
{
public:
    MonitorNode()
        : Node("monitor"), check_count_(0)
    {
        monitor_timer_ = this->create_wall_timer(
            3s,
            [this]()
            {
                check_count_++;
                RCLCPP_INFO(this->get_logger(),
                            "[Monitor node] system check #%d, thread=%zu",
                            check_count_, get_thread_id());
            });

        RCLCPP_INFO(this->get_logger(),
                    "[Monitor node] started (0.33Hz, independent executor)");
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int check_count_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2

    // ================================================================
    // 多执行器架构
    //
    //   ┌─────────────────────┐   ┌──────────────────────┐   ┌──────────────────┐
    //   │  Executor 1         │   │  Executor 2          │   │  Executor 3      │
    //   │  (SingleThreaded)   │   │  (MultiThreaded)     │   │  (SingleThreaded)│
    //   │  ┌───────────────┐  │   │  ┌────────────────┐  │   │  ┌────────────┐  │
    //   │  │RealtimeSensor │  │   │  │DataProcessing  │  │   │  │  Monitor   │  │
    //   │  │  (100Hz)      │  │   │  │  (耗时计算)     │  │   │  │  (低频)    │  │
    //   │  └───────────────┘  │   │  └────────────────┘  │   │  └────────────┘  │
    //   └─────────┬───────────┘   └──────────┬───────────┘   └──────────────────┘
    //             │ /imu/data                │
    //             └──────────────────────────┘
    //
    // 优点：
    //   1. 实时性隔离 —— 耗时处理绝不影响传感器频率
    //   2. 故障隔离 —— 一个执行器崩溃不影响其他
    //   3. 灵活配置 —— 不同节点用不同类型执行器
    //
    // 缺点：
    //   1. 更多线程 → 更多内存和上下文切换
    //   2. 代码复杂度增加
    //   3. 节点间仅通过 topic/service 通信
    // ================================================================

    auto sensor_node = std::make_shared<RealtimeSensorNode>();
    auto process_node = std::make_shared<DataProcessingNode>();
    auto monitor_node = std::make_shared<MonitorNode>();

    // 创建三个独立的执行器（使用 shared_ptr 以便在线程间共享）
    auto sensor_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto process_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 2); // 数据处理用多线程，订阅和定时器不互相阻塞
    auto monitor_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    sensor_executor->add_node(sensor_node);   // 传感器独享执行器，保证 100Hz 稳定
    process_executor->add_node(process_node); // 处理独享执行器，耗时计算不影响其他
    monitor_executor->add_node(monitor_node); // 监控独享执行器

    RCLCPP_INFO(rclcpp::get_logger("main"),
                "=== Multi-executor architecture started ===");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  Executor 1 (SingleThreaded): realtime sensor 100Hz");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  Executor 2 (MultiThreaded):  data processing (2 threads)");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  Executor 3 (SingleThreaded): monitor (0.33Hz)");

    // ================================================================
    // 启动三个线程，分别驱动三个执行器
    // ================================================================
    std::vector<std::thread> threads; // 存放各执行器线程

    threads.emplace_back([sensor_executor]()
                         { sensor_executor->spin(); }); // 线程1：驱动传感器执行器

    threads.emplace_back([process_executor]()
                         { process_executor->spin(); }); // 线程2：驱动数据处理执行器

    threads.emplace_back([monitor_executor]()
                         { monitor_executor->spin(); }); // 线程3：驱动监控执行器

    RCLCPP_INFO(rclcpp::get_logger("main"),
                "Three executor threads started, observe sensor 100Hz is not blocked");

    // 等待所有线程结束
    for (auto &t : threads)
    {
        if (t.joinable()) // 检查线程是否可 join
        {
            t.join(); // 阻塞等待线程结束
        }
    }

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
