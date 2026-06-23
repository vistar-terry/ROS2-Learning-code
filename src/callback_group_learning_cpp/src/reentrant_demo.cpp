/**
 * ===========================================================================
 *  ③ Reentrant 回调组演示 (C++版)
 * ===========================================================================
 *
 * 核心知识点：
 *   - ReentrantCallbackGroup：同一组内的回调可以并发执行
 *   - 必须搭配 MultiThreadedExecutor 才能真正并发
 *   - 使用 Reentrant 时必须自行保证线程安全！
 *   - C++ 中使用 std::mutex 替代 Python 的 threading.Lock()
 *
 * C++ API:
 *   - 创建: node->create_callback_group(CallbackGroupType::Reentrant)
 *   - 定时器绑定: create_wall_timer(period, callback, callback_group)
 *   - 线程安全: std::mutex + std::lock_guard<std::mutex>
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <sstream>

using namespace std::chrono_literals;

/**
 * Reentrant 回调组演示节点
 */
class ReentrantDemoNode : public rclcpp::Node
{
public:
    ReentrantDemoNode()
        : Node("reentrant_demo_node"),
          counter_(0), safe_counter_(0)
    {
        // ================================================================
        // 1. 创建两种回调组
        // ================================================================
        // Reentrant：同组回调可并发
        reentrant_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        // MutuallyExclusive：同组回调串行（作为对照）
        exclusive_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // ================================================================
        // 2. 使用 ReentrantCallbackGroup 的定时器
        //    C++ 中 create_wall_timer 的最后一个参数是 callback_group
        // ================================================================

        // 定时器1 —— 在 reentrant_group_ 中
        timer1_ = this->create_wall_timer(
            1s,
            std::bind(&ReentrantDemoNode::timer1_callback, this),
            reentrant_group_);

        // 定时器2 —— 也在 reentrant_group_ 中
        timer2_ = this->create_wall_timer(
            1s,
            std::bind(&ReentrantDemoNode::timer2_callback, this),
            reentrant_group_);

        // 定时器3 —— 在 exclusive_group_ 中（对照）
        timer3_ = this->create_wall_timer(
            1s,
            std::bind(&ReentrantDemoNode::timer3_callback, this),
            exclusive_group_);

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "ReentrantCallbackGroup 演示启动 (C++)");
        RCLCPP_INFO(this->get_logger(), "  timer1 和 timer2 在 ReentrantCallbackGroup 中 → 可并发");
        RCLCPP_INFO(this->get_logger(), "  timer3 在 MutuallyExclusiveCallbackGroup 中 → 串行");
        RCLCPP_INFO(this->get_logger(), "  使用 MultiThreadedExecutor");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    /**
     * 定时器1回调 —— Reentrant，可与其他回调并发
     * 演示不安全的计数操作和安全的计数操作
     */
    void timer1_callback()
    {
        std::ostringstream tid;
        tid << std::this_thread::get_id();
        RCLCPP_INFO(this->get_logger(), "[timer1] 开始, 线程: %s", tid.str().c_str());

        std::this_thread::sleep_for(500ms);

        // --- 不安全的计数操作（演示竞态条件）---
        // 读取 → 修改 → 写回，不是原子操作
        int temp = counter_;
        std::this_thread::sleep_for(10ms); // 增加竞态窗口
        temp += 1;
        counter_ = temp;

        // --- 安全的计数操作（使用 std::mutex 保护）---
        {
            std::lock_guard<std::mutex> lock(mutex_);
            safe_counter_ += 1;
        }

        RCLCPP_INFO(this->get_logger(),
                    "[timer1] 完成, 不安全计数=%d, 安全计数=%d", counter_, safe_counter_);
    }

    /**
     * 定时器2回调 —— 与 timer1 在同一个 Reentrant 组，可并发
     */
    void timer2_callback()
    {
        std::ostringstream tid;
        tid << std::this_thread::get_id();
        RCLCPP_INFO(this->get_logger(), "[timer2] 开始, 线程: %s", tid.str().c_str());

        std::this_thread::sleep_for(300ms);

        // 同样的不安全操作
        int temp = counter_;
        std::this_thread::sleep_for(10ms);
        temp += 1;
        counter_ = temp;

        // 安全操作
        {
            std::lock_guard<std::mutex> lock(mutex_);
            safe_counter_ += 1;
        }

        RCLCPP_INFO(this->get_logger(),
                    "[timer2] 完成, 不安全计数=%d, 安全计数=%d", counter_, safe_counter_);
    }

    /**
     * 定时器3回调 —— 在 exclusive_group_ 中
     */
    void timer3_callback()
    {
        std::ostringstream tid;
        tid << std::this_thread::get_id();
        RCLCPP_INFO(this->get_logger(), "[timer3] 开始, 线程: %s", tid.str().c_str());
        std::this_thread::sleep_for(100ms);
        RCLCPP_INFO(this->get_logger(), "[timer3] 完成");
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;
    rclcpp::CallbackGroup::SharedPtr exclusive_group_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;

    // 线程安全
    std::mutex mutex_; // C++ 互斥锁

    // 计数器
    int counter_;      // 不安全的计数器（演示竞态）
    int safe_counter_; // 安全的计数器（使用 mutex 保护）
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReentrantDemoNode>();

    // ================================================================
    // 必须使用 MultiThreadedExecutor 才能让 Reentrant 组真正并发
    // ================================================================
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);

    executor.add_node(node);

    try
    {
        executor.spin();
    }
    catch (...)
    {
    }

    rclcpp::shutdown();
    return 0;
}
