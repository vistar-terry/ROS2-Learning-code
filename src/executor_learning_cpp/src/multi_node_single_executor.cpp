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
class SensorNode : public rclcpp::Node {
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
            [this]() {
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = this->now();
                msg.header.frame_id = "imu_link";
                imu_count_++;

                imu_pub_->publish(msg);

                if (imu_count_ % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(),
                        "[传感器节点] 已发布 %d 帧 IMU 数据, 线程=%zu",
                        imu_count_, get_thread_id());
                }
            });

        RCLCPP_INFO(this->get_logger(), "[传感器节点] 已启动 (100Hz IMU)");
    }

private:
    std::size_t get_thread_id() {
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
class ProcessingNode : public rclcpp::Node {
public:
    ProcessingNode()
        : Node("processing_node"), process_count_(0)
    {
        // 订阅 IMU 数据
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                process_count_++;
                // 每收到 10 帧处理一次
                if (process_count_ % 10 == 0) {
                    // 模拟轻量处理
                    double dt = (this->now() - msg->header.stamp).seconds();
                    RCLCPP_INFO(this->get_logger(),
                        "[处理节点] 第%d帧, 延迟=%.3fms, 线程=%zu",
                        process_count_, dt * 1000.0, get_thread_id());
                }
            });

        // 低频状态报告
        status_timer_ = this->create_wall_timer(
            1s,
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                    "[处理节点] 状态: 已处理%d帧", process_count_);
            });

        RCLCPP_INFO(this->get_logger(), "[处理节点] 已启动");
    }

private:
    std::size_t get_thread_id() {
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
class MonitorNode : public rclcpp::Node {
public:
    MonitorNode()
        : Node("monitor_node"), check_count_(0)
    {
        // 0.5Hz 监控定时器
        monitor_timer_ = this->create_wall_timer(
            2s,
            [this]() {
                check_count_++;
                RCLCPP_INFO(this->get_logger(),
                    "[监控节点] 系统检查 #%d, 线程=%zu",
                    check_count_, get_thread_id());
            });

        RCLCPP_INFO(this->get_logger(), "[监控节点] 已启动 (0.5Hz)");
    }

private:
    std::size_t get_thread_id() {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int check_count_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    int mode = 1;  // 默认：多线程执行器
    if (argc > 1) {
        mode = std::atoi(argv[1]);
    }

    auto sensor_node = std::make_shared<SensorNode>();
    auto process_node = std::make_shared<ProcessingNode>();
    auto monitor_node = std::make_shared<MonitorNode>();

    switch (mode) {
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
        case 1: {
            RCLCPP_INFO(rclcpp::get_logger("main"),
                "=== 模式1: MultiThreadedExecutor (3线程) ===");
            RCLCPP_INFO(rclcpp::get_logger("main"),
                "三个节点共享执行器，不同节点的回调可并发执行");

            rclcpp::executors::MultiThreadedExecutor executor(
                rclcpp::ExecutorOptions(), 3);
            executor.add_node(sensor_node);
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
        case 2: {
            RCLCPP_INFO(rclcpp::get_logger("main"),
                "=== 模式2: SingleThreadedExecutor (单线程) ===");
            RCLCPP_INFO(rclcpp::get_logger("main"),
                "三个节点串行执行，观察传感器节点被阻塞的情况");

            rclcpp::executors::SingleThreadedExecutor executor;
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
        case 3: {
            RCLCPP_INFO(rclcpp::get_logger("main"),
                "=== 模式3: 动态添加/移除节点 ===");

            rclcpp::executors::SingleThreadedExecutor executor;

            // 先只添加传感器节点和处理节点
            executor.add_node(sensor_node);
            executor.add_node(process_node);
            RCLCPP_INFO(rclcpp::get_logger("main"),
                "已添加传感器节点和处理节点（2个节点）");

            // 使用 spin_once 实现动态管理
            int iteration = 0;
            bool monitor_added = false;

            while (rclcpp::ok()) {
                executor.spin_once(100ms);
                iteration++;

                // 5秒后添加监控节点
                if (!monitor_added && iteration > 50) {
                    executor.add_node(monitor_node);
                    monitor_added = true;
                    RCLCPP_INFO(rclcpp::get_logger("main"),
                        ">>> 动态添加监控节点！");
                }

                // 15秒后移除监控节点
                if (monitor_added && iteration > 150) {
                    executor.remove_node(monitor_node);
                    RCLCPP_INFO(rclcpp::get_logger("main"),
                        ">>> 动态移除监控节点！");
                    break;  // 退出演示
                }
            }
            break;
        }

        default: {
            RCLCPP_WARN(rclcpp::get_logger("main"), "未知模式 %d", mode);
            rclcpp::executors::MultiThreadedExecutor executor(
                rclcpp::ExecutorOptions(), 3);
            executor.add_node(sensor_node);
            executor.add_node(process_node);
            executor.add_node(monitor_node);
            executor.spin();
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}
