/**
 * @file multi_node_single_executor.cpp
 * @brief ⑥ 多节点单执行器 —— 一个执行器管理多个节点
 *
 * 知识点：
 * - 单执行器多节点架构的优势与局限
 * - add_node() / remove_node() 动态管理
 * - 节点间的回调调度顺序
 * - 单线程 vs 多线程执行器下的多节点行为
 * - 何时应该使用单执行器 vs 多执行器
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>

using namespace std::chrono_literals;

// ================================================================
// 传感器节点 —— 模拟高频传感器数据发布
// ================================================================
class SensorNode : public rclcpp::Node
{
public:
    SensorNode()
        : Node("sensor_node"), imu_count_(0)
    {
        // 发布 IMU 数据
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);

        // 100Hz 定时器发布模拟 IMU 数据
        imu_timer_ = this->create_wall_timer(
            10ms,
            [this]()
            {
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = this->now();
                msg.header.frame_id = "imu_link";
                imu_count_++;

                imu_pub_->publish(msg);

                if (imu_count_ % 100 == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[Sensor node] published %d IMU frames, thread=%zu",
                                imu_count_, get_thread_id());
                }
            });

        RCLCPP_INFO(this->get_logger(), "[Sensor node] started (100Hz IMU)");
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int imu_count_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
};

// ================================================================
// 处理节点 —— 订阅传感器数据并处理
// ================================================================
class ProcessingNode : public rclcpp::Node
{
public:
    ProcessingNode()
        : Node("processing_node"), process_count_(0)
    {
        // 订阅 IMU 数据
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                process_count_++;
                // 每收到 10 帧处理一次
                if (process_count_ % 10 == 0)
                {
                    // 模拟轻量处理
                    double dt = (this->now() - msg->header.stamp).seconds();
                    RCLCPP_INFO(this->get_logger(),
                                "[Processing node] frame #%d, latency=%.3fms, thread=%zu",
                                process_count_, dt * 1000.0, get_thread_id());
                }
            });

        // 低频状态报告
        status_timer_ = this->create_wall_timer(
            1s,
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Processing node] status: processed %d frames", process_count_);
            });

        RCLCPP_INFO(this->get_logger(), "[Processing node] started");
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int process_count_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

// ================================================================
// 监控节点 —— 低频监控所有子系统
// ================================================================
class MonitorNode : public rclcpp::Node
{
public:
    MonitorNode()
        : Node("monitor_node"), check_count_(0)
    {
        // 0.5Hz 监控定时器
        monitor_timer_ = this->create_wall_timer(
            2s,
            [this]()
            {
                check_count_++;
                RCLCPP_INFO(this->get_logger(),
                            "[Monitor node] system check #%d, thread=%zu",
                            check_count_, get_thread_id());
            });

        RCLCPP_INFO(this->get_logger(), "[Monitor node] started (0.5Hz)");
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

    int mode = 1; // 默认：多线程执行器
    if (argc > 1)
    {
        mode = std::atoi(argv[1]); // 从命令行参数读取模式
    }

    auto sensor_node = std::make_shared<SensorNode>();      // 传感器节点
    auto process_node = std::make_shared<ProcessingNode>(); // 处理节点
    auto monitor_node = std::make_shared<MonitorNode>();    // 监控节点

    switch (mode)
    {
    // ============================================================
    // 模式一：MultiThreadedExecutor —— 多节点并发执行
    //
    // 优点：
    //   - 不同节点的回调可以并发
    //   - 高频传感器回调不被低频监控阻塞
    //   - 充分利用多核 CPU
    //
    // 缺点：
    //   - 线程切换开销
    //   - 需要考虑线程安全
    // ============================================================
    case 1:
    {
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "=== Mode 1: MultiThreadedExecutor (3 threads) ===");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Three nodes share the executor, callbacks from different nodes can execute concurrently");

        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 3); // 3个线程，对应3个节点
        executor.add_node(sensor_node);    // 不同节点的回调可并发执行
        executor.add_node(process_node);
        executor.add_node(monitor_node);
        executor.spin();
        break;
    }

    // ============================================================
    // 模式二：SingleThreadedExecutor —— 多节点串行执行
    //
    // 优点：
    //   - 无线程安全顾虑
    //   - 确定性执行顺序
    //
    // 缺点：
    //   - 一个节点的耗时回调会阻塞所有节点
    //   - 高频回调被低频回调拖慢
    // ============================================================
    case 2:
    {
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "=== Mode 2: SingleThreadedExecutor (single thread) ===");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Three nodes execute serially, observe sensor node being blocked");

        rclcpp::executors::SingleThreadedExecutor executor; // 单线程：所有节点串行
        executor.add_node(sensor_node);
        executor.add_node(process_node);
        executor.add_node(monitor_node);
        executor.spin();
        break;
    }

    // ============================================================
    // 模式三：动态添加/移除节点
    //    演示 add_node / remove_node 的运行时操作
    //    注意：StaticSingleThreadedExecutor 不支持此功能！
    // ============================================================
    case 3:
    {
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "=== Mode 3: Dynamic add/remove node ===");

        rclcpp::executors::SingleThreadedExecutor executor; // 支持运行时 add/remove_node

        // 先只添加传感器节点和处理节点
        executor.add_node(sensor_node);
        executor.add_node(process_node);
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Added sensor node and processing node (2 nodes)");

        // 使用 spin_once 实现动态管理
        int iteration = 0;
        bool monitor_added = false;

        while (rclcpp::ok())
        {
            executor.spin_once(100ms);
            iteration++;

            // 5秒后添加监控节点
            if (!monitor_added && iteration > 50)
            {
                executor.add_node(monitor_node); // 运行时动态添加节点
                monitor_added = true;
                RCLCPP_INFO(rclcpp::get_logger("main"),
                            ">>> Dynamically added monitor node!");
            }

            // 15秒后移除监控节点
            if (monitor_added && iteration > 150)
            {
                executor.remove_node(monitor_node); // 运行时动态移除节点
                RCLCPP_INFO(rclcpp::get_logger("main"),
                            ">>> Dynamically removed monitor node!");
                break; // 退出演示
            }
        }
        break;
    }

    default:
    {
        RCLCPP_WARN(rclcpp::get_logger("main"), "Unknown mode %d", mode);
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 3); // 未知模式默认使用多线程执行器
        executor.add_node(sensor_node);
        executor.add_node(process_node);
        executor.add_node(monitor_node);
        executor.spin();
        break;
    }
    }

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
