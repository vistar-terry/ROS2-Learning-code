/**
 * @file robot_multi_arm.cpp
 * @brief ⑩ 机器人多臂协调 —— 多执行器隔离不同子系统的实时性
 *
 * 知识点：
 * - 多执行器架构在复杂机器人系统中的应用
 * - 执行器隔离实现子系统独立实时性保证
 * - 左臂/右臂/底盘三个子系统的独立执行器
 * - 全局协调器通过 topic 通信
 * - 优雅关闭：信号处理 + executor.cancel()
 *
 * 架构设计：
 *
 *   ┌────────────────────┐  ┌────────────────────┐  ┌──────────────────┐
 *   │  左臂执行器         │  │  右臂执行器         │  │  底盘执行器       │
 *   │  (StaticSingle)    │  │  (StaticSingle)    │  │  (StaticSingle)  │
 *   │  ┌──────────────┐  │  │  ┌──────────────┐  │  │  ┌────────────┐  │
 *   │  │ 100Hz 控制   │  │  │  │ 100Hz 控制   │  │  │  │ 50Hz 控制  │  │
 *   │  │ 10Hz 状态    │  │  │  │ 10Hz 状态    │  │  │  │ 20Hz 里程  │  │
 *   │  └──────────────┘  │  │  └──────────────┘  │  │  └────────────┘  │
 *   └────────┬───────────┘  └────────┬───────────┘  └────────┬─────────┘
 *            │ /left_arm_cmd         │ /right_arm_cmd        │ /base_cmd
 *            └───────────┬───────────┘                      │
 *                   ┌────┴────┐                              │
 *                   │协调器    │◄─────────────────────────────┘
 *                   │(Multi)  │
 *                   └─────────┘
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <mutex>
#include <atomic>
#include <csignal>

using namespace std::chrono_literals;

// 全局原子变量，用于信号处理中的优雅关闭
static std::atomic<bool> g_shutdown_requested{false}; // 原子变量，线程安全

void signal_handler(int signum)
{
    (void)signum;                // 忽略未使用参数警告
    g_shutdown_requested = true; // 设置关闭标志
}

// ================================================================
// 机械臂控制节点 —— 每个臂一个独立实例
// ================================================================
class ArmControlNode : public rclcpp::Node
{
public:
    explicit ArmControlNode(const std::string &arm_name)
        : Node(arm_name + "_arm_controller"),
          arm_name_(arm_name), control_count_(0), state_count_(0)
    {
        // 命令订阅
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/" + arm_name + "_arm_cmd", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[%s arm] received command: '%s'", arm_name_.c_str(), msg->data.c_str());
            });

        // 关节状态发布
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/" + arm_name + "_arm/joint_states", 10);

        // 100Hz 控制循环
        control_timer_ = this->create_wall_timer(
            10ms,
            [this]()
            {
                control_count_++;
                // 模拟控制计算
                auto msg = sensor_msgs::msg::JointState();
                msg.header.stamp = this->now();
                msg.name = {arm_name_ + "_joint1", arm_name_ + "_joint2", arm_name_ + "_joint3"};
                msg.position = {0.0, 0.0, 0.0};
                joint_pub_->publish(msg);

                if (control_count_ % 200 == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[%s arm control 100Hz] #%d, thread=%zu",
                                arm_name_.c_str(), control_count_, get_thread_id());
                }
            });

        // 10Hz 状态报告
        state_timer_ = this->create_wall_timer(
            100ms,
            [this]()
            {
                state_count_++;
                if (state_count_ % 10 == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[%s arm status 10Hz] control=%d, thread=%zu",
                                arm_name_.c_str(), control_count_, get_thread_id());
                }
            });

        RCLCPP_INFO(this->get_logger(),
                    "[%s arm controller] started (100Hz control + 10Hz status)", arm_name_.c_str());
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    std::string arm_name_;
    int control_count_;
    int state_count_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
};

// ================================================================
// 底盘控制节点 —— 独立执行器
// ================================================================
class BaseControlNode : public rclcpp::Node
{
public:
    BaseControlNode()
        : Node("base_controller"), control_count_(0)
    {
        // 速度命令订阅
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/base_cmd", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(),
                            "[Base] velocity command: vx=%.2f, omega=%.2f",
                            msg->linear.x, msg->angular.z);
            });

        // 50Hz 控制循环
        control_timer_ = this->create_wall_timer(
            20ms,
            [this]()
            {
                control_count_++;
                if (control_count_ % 100 == 0)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "[Base control 50Hz] #%d, thread=%zu",
                                control_count_, get_thread_id());
                }
            });

        // 20Hz 里程计发布
        odom_timer_ = this->create_wall_timer(
            50ms,
            [this]()
            {
                // 模拟里程计发布
            });

        RCLCPP_INFO(this->get_logger(),
                    "[Base controller] started (50Hz control + 20Hz odometry)");
    }

private:
    std::size_t get_thread_id()
    {
        std::ostringstream oss;
        oss << std::this_thread::get_id();
        return std::hash<std::string>{}(oss.str()) % 10000;
    }

    int control_count_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
};

// ================================================================
// 协调器节点 —— 通过 topic 协调各子系统
// ================================================================
class CoordinatorNode : public rclcpp::Node
{
public:
    CoordinatorNode()
        : Node("coordinator"), coordination_count_(0)
    {
        // 左臂命令发布
        left_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/left_arm_cmd", 10);

        // 右臂命令发布
        right_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/right_arm_cmd", 10);

        // 底盘命令发布
        base_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/base_cmd", 10);

        // 1Hz 协调循环
        coord_timer_ = this->create_wall_timer(
            1s,
            [this]()
            {
                coordination_count_++;
                RCLCPP_INFO(this->get_logger(),
                            "[Coordinator] sending coordination command #%d", coordination_count_);

                // 左臂命令
                auto left_msg = std_msgs::msg::String();
                left_msg.data = "move_to_pick";
                left_cmd_pub_->publish(left_msg);

                // 右臂命令
                auto right_msg = std_msgs::msg::String();
                right_msg.data = "move_to_place";
                right_cmd_pub_->publish(right_msg);

                // 底盘命令
                auto base_msg = geometry_msgs::msg::Twist();
                base_msg.linear.x = 0.2;
                base_cmd_pub_->publish(base_msg);
            });

        RCLCPP_INFO(this->get_logger(),
                    "[Coordinator] started (1Hz coordination loop)");
    }

private:
    int coordination_count_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr left_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr right_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_pub_;
    rclcpp::TimerBase::SharedPtr coord_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化 ROS2

    // 注册信号处理（优雅关闭）
    std::signal(SIGINT, signal_handler); // 捕获 Ctrl+C 信号

    // ================================================================
    // 创建4个节点
    // ================================================================
    auto left_arm = std::make_shared<ArmControlNode>("left");
    auto right_arm = std::make_shared<ArmControlNode>("right");
    auto base = std::make_shared<BaseControlNode>();
    auto coordinator = std::make_shared<CoordinatorNode>();

    // ================================================================
    // 创建3个独立执行器
    //
    //   执行器1 (StaticSingleThreaded): 左臂 —— 确定性100Hz控制
    //   执行器2 (StaticSingleThreaded): 右臂 —— 确定性100Hz控制
    //   执行器3 (SingleThreaded): 底盘 + 协调器 —— 非关键实时
    //
    // 为什么用 StaticSingleThreaded？
    //   - 机械臂控制需要确定性延迟
    //   - 不需要动态添加/移除节点
    //   - 零分配 → 最低抖动
    // ================================================================
    auto left_executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();  // 左臂：零分配确定性控制
    auto right_executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>(); // 右臂：零分配确定性控制
    auto base_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();        // 底盘+协调器：非关键实时

    left_executor->add_node(left_arm);    // 左臂节点 → 独立执行器
    right_executor->add_node(right_arm);  // 右臂节点 → 独立执行器
    base_executor->add_node(base);        // 底盘节点
    base_executor->add_node(coordinator); // 协调器与底盘共享执行器

    RCLCPP_INFO(rclcpp::get_logger("main"),
                "=== Multi-arm coordination robot system started ===");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  Executor 1 (StaticSingleThreaded): left arm 100Hz");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  Executor 2 (StaticSingleThreaded): right arm 100Hz");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  Executor 3 (SingleThreaded):       base 50Hz + coordinator 1Hz");
    RCLCPP_INFO(rclcpp::get_logger("main"),
                "  All independent, no mutual interference");

    // ================================================================
    // 启动三个线程驱动三个执行器
    // ================================================================
    std::vector<std::thread> threads; // 存放各执行器线程

    threads.emplace_back([left_executor]()
                         {
                             left_executor->spin(); // 线程1：驱动左臂执行器
                         });

    threads.emplace_back([right_executor]()
                         {
                             right_executor->spin(); // 线程2：驱动右臂执行器
                         });

    threads.emplace_back([base_executor]()
                         {
                             base_executor->spin(); // 线程3：驱动底盘+协调器执行器
                         });

    // ================================================================
    // 主线程监控 + 优雅关闭
    // ================================================================
    while (rclcpp::ok() && !g_shutdown_requested)
    {                                       // 主线程轮询检查关闭标志
        std::this_thread::sleep_for(500ms); // 每500ms检查一次
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Graceful shutdown in progress...");

    // 取消所有执行器（使 spin() 退出阻塞）
    left_executor->cancel();
    right_executor->cancel();
    base_executor->cancel();

    // 等待所有线程结束
    for (auto &t : threads)
    {
        if (t.joinable())
        {
            t.join(); // 阻塞等待线程安全退出
        }
    }

    rclcpp::shutdown(); // 清理 ROS2 资源
    RCLCPP_INFO(rclcpp::get_logger("main"), "System safely shut down");
    return 0;
}
