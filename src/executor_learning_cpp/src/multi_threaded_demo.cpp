/**
 * @file multi_threaded_demo.cpp
 * @brief ② 多线程执行器 —— 并发回调与线程安全
 *
 * 知识点：
 * - MultiThreadedExecutor 的创建与配置
 * - 线程数配置（num_threads 参数）
 * - 不同回调组下的并发行为
 * - std::mutex / std::atomic 保护共享数据
 * - 回调组互斥 vs 线程锁互斥的对比
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

class MultiThreadedDemoNode : public rclcpp::Node
{
public:
    MultiThreadedDemoNode()
        : Node("multi_threaded_demo")
        , counter_(0)
        , imu_count_(0)
    {
        // ================================================================
        // 1. 创建回调组
        //    - 互斥组：同组回调不会并发执行（保护共享数据）
        //    - 可重入组：同组回调可以并发执行（需自行加锁）
        // ================================================================
        imu_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        mutually_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 可重入组 —— 允许同组回调并发（演示线程安全）
        reentrant_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // ================================================================
        // 2. IMU 订阅 —— 绑定到 imu_group_
        //    高频传感器数据（100Hz），不能被低频任务阻塞
        // ================================================================
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = imu_group_;

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                // 互斥组保证此回调不会被同组其他回调打断
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_orientation_ = msg->orientation;
                imu_count_++;
                if (imu_count_ % 50 == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                        "[IMU callback][thread %zu] frame: %d, group: imu_group",
                        get_thread_id(), imu_count_.load());
                }
            },
            sub_opts);

        // ================================================================
        // 3. 处理定时器 —— 绑定到 mutually_group_
        //    低频处理任务（2Hz），耗时操作不会阻塞 IMU
        // ================================================================
        light_timer_ = this->create_wall_timer(
            500ms,
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                    "[Light callback][thread %zu] start ‌light computation, group: reentrant_group_",
                    get_thread_id());
                // 模拟 100ms 的计算
                std::this_thread::sleep_for(100ms);
                RCLCPP_INFO(this->get_logger(),
                    "[Light callback][thread %zu] ‌light computation completed",
                    get_thread_id());
            },
            mutually_group_);

        heavy_timer_ = this->create_wall_timer(
            1500ms,
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                    "[Heavy callback][thread %zu] start heavy computation, group: reentrant_group_",
                    get_thread_id());
                // 模拟 1000ms 的计算
                std::this_thread::sleep_for(1000ms);
                RCLCPP_INFO(this->get_logger(),
                    "[Heavy callback][thread %zu] heavy computation completed",
                    get_thread_id());
            },
            mutually_group_);

        // ================================================================
        // 4. 状态报告定时器 —— 绑定到 reentrant_group_
        //    可重入组允许与其他可重入回调并发执行
        // ================================================================
        report_timer_ = this->create_wall_timer(
            200ms,
            [this]()
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                RCLCPP_INFO(this->get_logger(),
                    "[Status report][thread %zu] IMU frames: %d, count: %d, group: reentrant",
                    get_thread_id(), imu_count_.load(), counter_.load());
            },
            reentrant_group_);

        // ================================================================
        // 5. 另一个可重入回调 —— 演示同组并发
        // ================================================================
        reentrant_timer_ = this->create_wall_timer(
            300ms,
            [this]()
            {
                RCLCPP_INFO(this->get_logger(),
                    "[Reentrant callback A][thread %zu] start", get_thread_id());
                std::this_thread::sleep_for(100ms);
                counter_++;
                RCLCPP_INFO(this->get_logger(),
                    "[Reentrant callback A][thread %zu] end, count: %d", get_thread_id(), counter_.load());
            },
            reentrant_group_);

        RCLCPP_INFO(this->get_logger(), "=== MultiThreadedExecutor demo node started ===");
        RCLCPP_INFO(this->get_logger(), "Observe: callbacks in different groups execute concurrently in different threads");
        RCLCPP_INFO(this->get_logger(), "Callbacks in the same mutually exclusive group still execute serially");
        RCLCPP_INFO(this->get_logger(), "Callbacks in the same reentrant group can execute concurrently");
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr imu_group_;
    rclcpp::CallbackGroup::SharedPtr mutually_group_;
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;

    // 共享数据（跨回调组访问，需加锁）
    std::mutex data_mutex_;      // 互斥锁，保护跨回调组的共享数据
    std::atomic<int> counter_;   // 原子变量，可重入回调中安全自增
    std::atomic<int> imu_count_; // 原子变量，IMU 帧计数
    geometry_msgs::msg::Quaternion latest_orientation_;

    // 订阅和定时器
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr light_timer_;
    rclcpp::TimerBase::SharedPtr heavy_timer_;
    rclcpp::TimerBase::SharedPtr report_timer_;
    rclcpp::TimerBase::SharedPtr reentrant_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2

    auto node = std::make_shared<MultiThreadedDemoNode>();

    // ================================================================
    // 创建 MultiThreadedExecutor
    //    num_threads: 线程池大小
    //    - 默认值 = std::thread::hardware_concurrency()
    //    - 建议设置为回调组数量 + 1
    //    - 线程数 >= 回调组数量才能实现最大并发
    // ================================================================
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4); // 使用 4 个线程

    executor.add_node(node); // 将节点注册到执行器

    RCLCPP_INFO(node->get_logger(), "Using MultiThreadedExecutor with 4 threads");
    executor.spin(); // 阻塞当前线程，多线程并发执行回调

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
