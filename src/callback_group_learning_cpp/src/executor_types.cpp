/**
 * ===========================================================================
 *  ④ 执行器类型对比 (C++版)
 * ===========================================================================
 *
 * 核心知识点：
 *   - SingleThreadedExecutor：单线程依次执行所有就绪回调
 *   - MultiThreadedExecutor：多线程并发执行不同回调组的回调
 *   - 执行器与回调组的配合决定了回调的执行方式
 *
 * C++ Executor API:
 *   - rclcpp::spin(node)                                    → SingleThreaded
 *   - rclcpp::executors::SingleThreadedExecutor             → 单线程
 *   - rclcpp::executors::MultiThreadedExecutor(num_threads) → 多线程
 *   - executor.add_node(node)                               → 注册节点
 *   - executor.spin() / executor.spin_once(timeout)          → 运行
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <sstream>
#include <atomic>

using namespace std::chrono_literals;

/**
 * 执行器测试节点 —— 包含两种回调组
 */
class ExecutorTestNode : public rclcpp::Node
{
public:
    ExecutorTestNode() : Node("executor_test_node"), count_(0)
    {
        // ================================================================
        // 1. 创建两种回调组
        // ================================================================
        group_a_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        group_b_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // ================================================================
        // 2. 创建多个定时器，分布在不同回调组
        // ================================================================

        // 定时器A1 —— 在 MutuallyExclusive 回调组 A 中
        timer_a1_ = this->create_wall_timer(
            1s, std::bind(&ExecutorTestNode::callback_a1, this), group_a_);

        // 定时器A2 —— 也在回调组 A 中（与 A1 互斥）
        timer_a2_ = this->create_wall_timer(
            1s, std::bind(&ExecutorTestNode::callback_a2, this), group_a_);

        // 定时器B1 —— 在 Reentrant 回调组 B 中
        timer_b1_ = this->create_wall_timer(
            1s, std::bind(&ExecutorTestNode::callback_b1, this), group_b_);

        // 定时器B2 —— 也在回调组 B 中（与 B1 可并发）
        timer_b2_ = this->create_wall_timer(
            1s, std::bind(&ExecutorTestNode::callback_b2, this), group_b_);

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "执行器类型对比演示启动 (C++)");
        RCLCPP_INFO(this->get_logger(), "  A1, A2 在 MutuallyExclusiveCallbackGroup → 同组串行");
        RCLCPP_INFO(this->get_logger(), "  B1, B2 在 ReentrantCallbackGroup → 同组可并发");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    void callback_a1()
    {
        count_++;
        auto tid = get_thread_id();
        RCLCPP_INFO(this->get_logger(), "[A1] 开始 #%d, 线程: %s", count_.load(), tid.c_str());
        std::this_thread::sleep_for(300ms);
        RCLCPP_INFO(this->get_logger(), "[A1] 完成 #%d", count_.load());
    }

    void callback_a2()
    {
        count_++;
        auto tid = get_thread_id();
        RCLCPP_INFO(this->get_logger(), "[A2] 开始 #%d, 线程: %s", count_.load(), tid.c_str());
        std::this_thread::sleep_for(200ms);
        RCLCPP_INFO(this->get_logger(), "[A2] 完成 #%d", count_.load());
    }

    void callback_b1()
    {
        count_++;
        auto tid = get_thread_id();
        RCLCPP_INFO(this->get_logger(), "[B1] 开始 #%d, 线程: %s", count_.load(), tid.c_str());
        std::this_thread::sleep_for(300ms);
        RCLCPP_INFO(this->get_logger(), "[B1] 完成 #%d", count_.load());
    }

    void callback_b2()
    {
        count_++;
        auto tid = get_thread_id();
        RCLCPP_INFO(this->get_logger(), "[B2] 开始 #%d, 线程: %s", count_.load(), tid.c_str());
        std::this_thread::sleep_for(200ms);
        RCLCPP_INFO(this->get_logger(), "[B2] 完成 #%d", count_.load());
    }

    std::string get_thread_id()
    {
        std::ostringstream tid;
        tid << std::this_thread::get_id();
        return tid.str();
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr group_a_;
    rclcpp::CallbackGroup::SharedPtr group_b_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_a1_;
    rclcpp::TimerBase::SharedPtr timer_a2_;
    rclcpp::TimerBase::SharedPtr timer_b1_;
    rclcpp::TimerBase::SharedPtr timer_b2_;

    // 原子计数器
    std::atomic<int> count_;
};

/**
 * 使用指定执行器运行节点5秒
 */
void run_with_executor(rclcpp::Node::SharedPtr node,
                       rclcpp::Executor& executor,
                       const std::string& executor_name)
{
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "\n>>> 使用 %s <<<", executor_name.c_str());

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 5s) {
        executor.spin_once(100ms);
    }

    executor.remove_node(node);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ================================================================
    // 对比1：SingleThreadedExecutor
    // 所有回调串行执行，无论回调组类型
    // ================================================================
    {
        auto node1 = std::make_shared<ExecutorTestNode>();
        rclcpp::executors::SingleThreadedExecutor single_executor;
        run_with_executor(node1, single_executor, "SingleThreadedExecutor");
    }

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "切换到 MultiThreadedExecutor，等待3秒..." << std::endl;
    std::cout << std::string(60, '=') << "\n" << std::endl;
    std::this_thread::sleep_for(3s);

    // ================================================================
    // 对比2：MultiThreadedExecutor
    // 不同回调组的回调可以并发
    // 同一 MutuallyExclusive 组的回调仍然串行
    // 同一 Reentrant 组的回调可以并发
    // ================================================================
    {
        auto node2 = std::make_shared<ExecutorTestNode>();
        rclcpp::executors::MultiThreadedExecutor multi_executor(
            rclcpp::ExecutorOptions(), 4);
        run_with_executor(node2, multi_executor, "MultiThreadedExecutor (4线程)");
    }

    rclcpp::shutdown();
    return 0;
}
