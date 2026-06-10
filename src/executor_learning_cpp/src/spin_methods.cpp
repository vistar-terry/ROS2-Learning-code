/**
 * @file spin_methods.cpp
 * @brief ⑤ spin / spin_once / spin_some —— 三种驱动方式详解
 *
 * 知识点：
 * - spin(): 阻塞式无限循环
 * - spin_once(): 执行一轮等待 + 分发
 * - spin_some(): 非阻塞式执行可用回调
 * - 自定义执行循环：结合 spin_some 实现优先级调度
 * - spin_once 超时参数的意义
 * - 在自定义循环中集成其他逻辑
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class SpinMethodsNode : public rclcpp::Node
{
public:
    SpinMethodsNode()
        : Node("spin_methods"), mode_("unknown"),
          callback_count_(0), custom_loop_count_(0)
    {
        // ================================================================
        // 创建订阅和定时器
        // ================================================================
        timer_ = this->create_wall_timer(
            100ms,
            [this]()
            {
                callback_count_++;
                RCLCPP_INFO(this->get_logger(),
                            "[Timer callback] #%d", callback_count_);
            });

        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/test_topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Subscription callback] received: '%s'", msg->data.c_str());
            });

        RCLCPP_INFO(this->get_logger(), "=== Spin methods demo node started ===");
    }

    void set_mode(const std::string &mode)
    {
        mode_ = mode;
        RCLCPP_INFO(this->get_logger(), "Running mode: %s", mode_.c_str());
    }

    int get_callback_count() const { return callback_count_; }
    int get_custom_loop_count() const { return custom_loop_count_; }
    void increment_custom_loop() { custom_loop_count_++; }

private:
    std::string mode_;
    int callback_count_;
    int custom_loop_count_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2

    int mode = 1; // 默认 spin()
    if (argc > 1)
    {
        mode = std::atoi(argv[1]); // 从命令行参数读取模式
    }

    auto node = std::make_shared<SpinMethodsNode>();    // 创建节点
    rclcpp::executors::SingleThreadedExecutor executor; // 所有模式共用一个执行器
    executor.add_node(node);                            // 将节点注册到执行器

    switch (mode)
    {
    // ================================================================
    // 模式一：spin() —— 阻塞式无限循环
    //
    //   while(rclcpp::ok()) {
    //       wait_for_ready_callbacks();  // 等待任一回调就绪
    //       execute_callback();           // 执行就绪的回调
    //   }
    //
    // 特点：最简单、最常用、完全由执行器控制
    // 适用：大多数 ROS2 节点
    // ================================================================
    case 1:
    {
        node->set_mode("spin() - blocking infinite loop");
        RCLCPP_INFO(node->get_logger(),
                    "spin() blocks the current thread, executing callbacks until rclcpp::shutdown()");
        executor.spin();
        break;
    }

    // ================================================================
    // 模式二：spin_once() —— 执行一轮
    //
    //   wait_for_ready_callbacks(timeout);  // 等待（有超时）
    //   execute_next_callback();              // 执行一个就绪回调
    //
    // 特点：每次调用只执行一个回调
    // 适用：需要在回调之间插入自定义逻辑
    // 注意：需要自己写循环！否则只执行一次
    // ================================================================
    case 2:
    {
        node->set_mode("spin_once() - single step execution");
        RCLCPP_INFO(node->get_logger(),
                    "spin_once() executes one callback per call, must be placed in a loop");

        while (rclcpp::ok())
        {
            // spin_once 执行一轮，超时 100ms
            executor.spin_once(100ms);

            // 可以在这里插入自定义逻辑
            node->increment_custom_loop();
            if (node->get_custom_loop_count() % 20 == 0)
            {
                RCLCPP_INFO(node->get_logger(),
                            "[Custom logic] loop=%d, callbacks=%d",
                            node->get_custom_loop_count(),
                            node->get_callback_count());
            }
        }
        break;
    }

    // ================================================================
    // 模式三：spin_some() —— 非阻塞执行所有可用回调
    //
    //   while(has_ready_callbacks()) {
    //       execute_next_callback();  // 执行一个就绪回调
    //       if (exceeded_max_duration()) break;
    //   }
    //
    // 特点：执行所有当前就绪的回调（有最大时长限制）
    // 适用：与其他事件循环集成（Qt/GTK/OpenCV GUI 等）
    // ================================================================
    case 3:
    {
        node->set_mode("spin_some() - non-blocking batch execution");
        RCLCPP_INFO(node->get_logger(),
                    "spin_some() executes all currently ready callbacks without blocking for new ones");

        while (rclcpp::ok())
        {
            // spin_some: 执行所有就绪回调，最多耗时 50ms
            executor.spin_some(50ms);

            // 模拟 GUI 或其他主循环逻辑
            node->increment_custom_loop();
            if (node->get_custom_loop_count() % 20 == 0)
            {
                RCLCPP_INFO(node->get_logger(),
                            "[Main loop] loop=%d, callbacks=%d",
                            node->get_custom_loop_count(),
                            node->get_callback_count());
            }

            // 防止 CPU 空转 —— 短暂休眠
            std::this_thread::sleep_for(10ms);
        }
        break;
    }

    // ================================================================
    // 模式四：自定义执行循环 —— 基于spin_some的优先级调度
    //
    //   while(rclcpp::ok()) {
    //       check_high_priority_callbacks();  // 先检查高优先级
    //       check_low_priority_callbacks();   // 再检查低优先级
    //       do_custom_work();                 // 自定义工作
    //   }
    //
    // 适用：需要精确控制回调执行顺序的实时系统
    // ================================================================
    case 4:
    {
        node->set_mode("Custom loop - priority scheduling");
        RCLCPP_INFO(node->get_logger(),
                    "Custom execution loop: inserting priority logic between spin_some calls");

        while (rclcpp::ok())
        {
            // 执行所有就绪回调
            executor.spin_some(10ms);

            // 自定义逻辑：例如检查系统状态、处理非 ROS 任务
            node->increment_custom_loop();
            if (node->get_custom_loop_count() % 50 == 0)
            {
                RCLCPP_INFO(node->get_logger(),
                            "[Priority scheduling] loop=%d, callbacks=%d",
                            node->get_custom_loop_count(),
                            node->get_callback_count());
            }

            // 控制循环频率
            std::this_thread::sleep_for(5ms);
        }
        break;
    }

    default:
    {
        RCLCPP_WARN(node->get_logger(), "Unknown mode %d, using default spin()", mode);
        executor.spin();
        break;
    }
    }

    rclcpp::shutdown(); // 清理 ROS2 资源
    return 0;
}
