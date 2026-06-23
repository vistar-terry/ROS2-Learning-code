/**
 * ===========================================================================
 *  ② MutuallyExclusive 回调组演示 (C++版)
 * ===========================================================================
 *
 * 核心知识点：
 *   - MutuallyExclusiveCallbackGroup：同一组内的回调不能并发执行
 *   - 即使使用 MultiThreadedExecutor，同一组内仍然串行执行
 *   - 不同组之间可以并发执行
 *   - 用途：保护共享资源，避免竞态条件
 *
 * C++ API 关键差异（相比 Python）：
 *   - 创建回调组: node->create_callback_group(CallbackGroupType::MutuallyExclusive)
 *   - 绑定到订阅: rclcpp::SubscriptionOptions().callback_group(group)
 *   - 绑定到定时器: create_wall_timer(..., group) 作为最后一个参数
 *   - 绑定到服务: create_service<...>(..., group) 作为最后一个参数
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <sstream>
#include <mutex>

using namespace std::chrono_literals;

/**
 * MutuallyExclusive 回调组演示节点
 */
class MutuallyExclusiveDemoNode : public rclcpp::Node
{
public:
    MutuallyExclusiveDemoNode() : Node("mutually_exclusive_demo_node"), count_(0), shared_data_(0)
    {
        // ================================================================
        // 1. 创建两个 MutuallyExclusive 回调组
        //    C++ 中使用 create_callback_group() 方法
        // ================================================================
        exclusive_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        another_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // ================================================================
        // 2. 将两个订阅放入同一个互斥回调组
        //    C++ 中通过 SubscriptionOptions 指定回调组
        // ================================================================

        // 订阅话题A —— 在 exclusive_group_ 中
        rclcpp::SubscriptionOptions sub_a_opts;
        sub_a_opts.callback_group = exclusive_group_;
        sub_a_ = this->create_subscription<std_msgs::msg::String>(
            "/topic_a", 10,
            std::bind(&MutuallyExclusiveDemoNode::callback_a, this, std::placeholders::_1),
            sub_a_opts);

        // 订阅话题B —— 也在 exclusive_group_ 中
        rclcpp::SubscriptionOptions sub_b_opts;
        sub_b_opts.callback_group = exclusive_group_;
        sub_b_ = this->create_subscription<std_msgs::msg::String>(
            "/topic_b", 10,
            std::bind(&MutuallyExclusiveDemoNode::callback_b, this, std::placeholders::_1),
            sub_b_opts);

        // 订阅话题C —— 在另一个独立回调组中
        rclcpp::SubscriptionOptions sub_c_opts;
        sub_c_opts.callback_group = another_group_;
        sub_c_ = this->create_subscription<std_msgs::msg::String>(
            "/topic_c", 10,
            std::bind(&MutuallyExclusiveDemoNode::callback_c, this, std::placeholders::_1),
            sub_c_opts);

        // ================================================================
        // 3. 创建发布者和定时器
        // ================================================================
        pub_a_ = this->create_publisher<std_msgs::msg::String>("/topic_a", 10);
        pub_b_ = this->create_publisher<std_msgs::msg::String>("/topic_b", 10);
        pub_c_ = this->create_publisher<std_msgs::msg::String>("/topic_c", 10);

        // 发布定时器 —— 使用独立回调组避免阻塞订阅
        pub_timer_ = this->create_wall_timer(
            1s,
            std::bind(&MutuallyExclusiveDemoNode::publish_messages, this),
            another_group_); // 发布定时器放在另一个组中

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "MutuallyExclusiveCallbackGroup 演示启动");
        RCLCPP_INFO(this->get_logger(), "  /topic_a 和 /topic_b 在同一互斥回调组");
        RCLCPP_INFO(this->get_logger(), "  /topic_c 在另一个独立回调组");
        RCLCPP_INFO(this->get_logger(), "  使用 MultiThreadedExecutor");
        RCLCPP_INFO(this->get_logger(), "  预期: A和B不会并发，A/B和C可以并发");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    void publish_messages()
    {
        count_++;
        auto msg_a = std_msgs::msg::String();
        msg_a.data = "msg_A_" + std::to_string(count_);
        auto msg_b = std_msgs::msg::String();
        msg_b.data = "msg_B_" + std::to_string(count_);
        auto msg_c = std_msgs::msg::String();
        msg_c.data = "msg_C_" + std::to_string(count_);
        pub_a_->publish(msg_a);
        pub_b_->publish(msg_b);
        pub_c_->publish(msg_c);
    }

    /**
     * 话题A回调 —— 在 exclusive_group_ 中
     * 修改 shared_data_，受 MutuallyExclusive 保护
     */
    void callback_a(const std_msgs::msg::String::SharedPtr msg)
    {
        // 获取当前线程ID（C++11）
        std::ostringstream thread_id;
        thread_id << std::this_thread::get_id();
        RCLCPP_INFO(this->get_logger(),
                    "[callback_a] 收到: %s, 线程: %s, 共享数据: %d",
                    msg->data.c_str(), thread_id.str().c_str(), shared_data_);

        // 修改共享数据（MutuallyExclusive 保护，无需加锁）
        shared_data_ += 1;
        std::this_thread::sleep_for(500ms);

        RCLCPP_INFO(this->get_logger(),
                    "[callback_a] 处理完成, 共享数据更新为: %d", shared_data_);
    }

    /**
     * 话题B回调 —— 也在 exclusive_group_ 中
     * 由于和 callback_a 在同一个 MutuallyExclusive 回调组，
     * 所以 callback_a 执行期间，callback_b 必须等待
     */
    void callback_b(const std_msgs::msg::String::SharedPtr msg)
    {
        std::ostringstream thread_id;
        thread_id << std::this_thread::get_id();
        RCLCPP_INFO(this->get_logger(),
                    "[callback_b] 收到: %s, 线程: %s, 共享数据: %d",
                    msg->data.c_str(), thread_id.str().c_str(), shared_data_);

        shared_data_ += 10;
        std::this_thread::sleep_for(300ms);

        RCLCPP_INFO(this->get_logger(),
                    "[callback_b] 处理完成, 共享数据更新为: %d", shared_data_);
    }

    /**
     * 话题C回调 —— 在 another_group_ 中
     * 在另一个回调组，可以和 callback_a / callback_b 并发执行
     */
    void callback_c(const std_msgs::msg::String::SharedPtr msg)
    {
        std::ostringstream thread_id;
        thread_id << std::this_thread::get_id();
        RCLCPP_INFO(this->get_logger(),
                    "[callback_c] 收到: %s, 线程: %s",
                    msg->data.c_str(), thread_id.str().c_str());
        std::this_thread::sleep_for(100ms);
    }

    // 回调组
    rclcpp::CallbackGroup::SharedPtr exclusive_group_;
    rclcpp::CallbackGroup::SharedPtr another_group_;

    // 订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_a_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_b_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_c_;

    // 发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_a_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_b_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_c_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    // 共享数据（受 MutuallyExclusiveCallbackGroup 保护）
    int count_;
    int shared_data_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MutuallyExclusiveDemoNode>();

    // ================================================================
    // 使用 MultiThreadedExecutor
    // 多个线程可以同时执行不同回调组的回调
    // 但同一 MutuallyExclusiveCallbackGroup 内的回调仍然串行
    // ================================================================
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // 4个线程

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
