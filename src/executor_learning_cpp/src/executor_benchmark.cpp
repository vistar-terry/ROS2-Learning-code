/**
 * @file executor_benchmark.cpp
 * @brief ④ 执行器对比基准 —— 三种执行器性能测试
 *
 * 知识点：
 * - 三种执行器的吞吐量与延迟对比
 * - 如何测量回调执行时间与调度延迟
 * - 不同负载下的执行器选择策略
 * - 执行器性能分析工具与方法
 *
 * 用法：
 *   ros2 run executor_learning_cpp executor_benchmark [模式]
 *   模式 1: SingleThreadedExecutor（默认）
 *   模式 2: MultiThreadedExecutor
 *   模式 3: StaticSingleThreadedExecutor
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class BenchmarkNode : public rclcpp::Node
{
public:
    explicit BenchmarkNode(const std::string &executor_name)
        : Node("benchmark_node")
        , executor_name_(executor_name)
        , total_callbacks_(0)
        , heavy_count_(0)
    {
        // ================================================================
        // 1. 高频轻量回调 —— 测试吞吐量
        //    1ms 间隔触发，模拟高频传感器数据
        // ================================================================
        high_freq_timer_ = this->create_wall_timer(
            1ms,
            [this]()
            {
                auto start = std::chrono::steady_clock::now();  // 记录回调开始时间
                // 极轻量操作
                total_callbacks_++;
                auto end = std::chrono::steady_clock::now();    // 记录回调结束时间
                auto duration = std::chrono::duration<double, std::micro>(end - start).count();  // 计算执行耗时(微秒)
                high_freq_latencies_.push_back(duration);  // 记录延迟用于统计
            });

        // ================================================================
        // 2. 低频重量回调 —— 测试调度延迟
        //    100ms 间隔触发，模拟 50ms 的计算负载
        // ================================================================
        heavy_timer_ = this->create_wall_timer(
            100ms,
            [this]()
            {
                heavy_count_++;
                RCLCPP_INFO(this->get_logger(),
                            "[%s] heavy callback #%d started, total high-freq callbacks=%d",
                            executor_name_.c_str(), heavy_count_, total_callbacks_.load());
                // 模拟 50ms 计算负载
                std::this_thread::sleep_for(50ms);
                RCLCPP_INFO(this->get_logger(),
                            "[%s] heavy callback #%d completed",
                            executor_name_.c_str(), heavy_count_);
            });

        // ================================================================
        // 3. 统计报告定时器 —— 每2秒输出性能指标
        // ================================================================
        stats_timer_ = this->create_wall_timer(
            2s,
            [this]()
            {
                print_stats();
            });

        RCLCPP_INFO(this->get_logger(), "=== Executor benchmark node [%s] started ===",
                    executor_name_.c_str());
    }

    // 打印性能统计
    void print_stats()
    {
        if (high_freq_latencies_.empty())
            return;

        // 计算延迟统计：排序后取平均值、中位数(p50)、p99 和最大值
        std::sort(high_freq_latencies_.begin(), high_freq_latencies_.end());
        double avg = std::accumulate(high_freq_latencies_.begin(),
                                     high_freq_latencies_.end(), 0.0) /
                     high_freq_latencies_.size();
        double p50 = high_freq_latencies_[high_freq_latencies_.size() / 2];  // 中位数
        double p99 = high_freq_latencies_[static_cast<size_t>(
            high_freq_latencies_.size() * 0.99)];  // 99 百分位
        double max_lat = high_freq_latencies_.back();  // 最大延迟

        RCLCPP_INFO(this->get_logger(),
                    "\n╔═════════════════════════════════════════════════════════════╗\n"
                    "║  [%s] Performance Report\n"
                    "║  Total high-freq callbacks: %d\n"
                    "║  Total heavy callbacks: %d\n"
                    "║  Callback latency(us): avg=%.1f  p50=%.1f  p99=%.1f  max=%.1f\n"
                    "╚═════════════════════════════════════════════════════════════╝",
                    executor_name_.c_str(),
                    total_callbacks_.load(), heavy_count_,
                    avg, p50, p99, max_lat);

        // 清空延迟数据，避免内存持续增长
        high_freq_latencies_.clear();
    }

private:
    std::string executor_name_;
    std::atomic<int> total_callbacks_;
    int heavy_count_;
    std::vector<double> high_freq_latencies_;

    rclcpp::TimerBase::SharedPtr high_freq_timer_;
    rclcpp::TimerBase::SharedPtr heavy_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // 初始化 ROS2

    int mode = 1;  // 默认使用 SingleThreadedExecutor
    if (argc > 1)
    {
        mode = std::atoi(argv[1]);  // 从命令行参数读取模式
    }

    std::string executor_name;
    switch (mode)
    {
    case 1:
        executor_name = "SingleThreaded";
        break;
    case 2:
        executor_name = "MultiThreaded";
        break;
    case 3:
        executor_name = "StaticSingleThreaded";
        break;
    default:
        executor_name = "SingleThreaded";
        mode = 1;
        break;
    }

    auto node = std::make_shared<BenchmarkNode>(executor_name);  // 传入执行器名称用于日志区分

    // 根据模式选择执行器
    switch (mode)
    {
    case 1:
    {
        RCLCPP_INFO(node->get_logger(), "Using SingleThreadedExecutor");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        break;
    }
    case 2:
    {
        RCLCPP_INFO(node->get_logger(), "Using MultiThreadedExecutor (2 threads)");
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
        break;
    }
    case 3:
    {
        RCLCPP_INFO(node->get_logger(), "Using StaticSingleThreadedExecutor");
        rclcpp::executors::StaticSingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        break;
    }
    }

    rclcpp::shutdown();  // 清理 ROS2 资源
    return 0;
}
