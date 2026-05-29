/**
 * @file lifecycle_basic.cpp
 * @brief ① 基础生命周期节点 —— 4个转换回调 + 状态查询
 *
 * 知识点：
 * - LifecycleNode 的4个主要状态
 * - 4个转换回调（on_configure/on_activate/on_deactivate/on_cleanup）
 * - on_shutdown 回调
 * - 状态转换成功/失败的返回值
 * - 通过命令行工具查看和切换状态
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>

using namespace std::chrono_literals;

// 继承 rclcpp_lifecycle::LifecycleNode 而非 rclcpp::Node
class LifecycleBasicNode : public rclcpp_lifecycle::LifecycleNode {
public:
    // ================================================================
    // 构造函数 —— 使用 LifecycleNode 基类
    //    注意：LifecycleNode 和普通 Node 的基类不同
    //    它自带状态机和转换服务
    // ================================================================
    LifecycleBasicNode()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_basic")
    {
        // 声明参数（生命周期节点同样支持参数）
        this->declare_parameter("demo_param", 42);

        RCLCPP_INFO(this->get_logger(),
            "=== 基础生命周期节点已创建 ===");
        RCLCPP_INFO(this->get_logger(),
            "初始状态: Unconfigured [%s]",
            this->get_current_state().label().c_str());

        // ================================================================
        // 注册状态转换事件回调（可选，用于监控所有状态变化）
        //    每次状态转换都会触发此回调
        // ================================================================
        transition_event_sub_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
            "/lifecycle_basic/transition_event", 10,
            [this](const lifecycle_msgs::msg::TransitionEvent::SharedPtr event) {
                RCLCPP_INFO(this->get_logger(),
                    "[转换事件] %s → %s (转换ID=%d)",
                    event->start_state.label.c_str(),
                    event->goal_state.label.c_str(),
                    event->transition.id);
            });
    }

    // ================================================================
    // on_configure: Unconfigured → Inactive
    //
    // 含义：初始化资源、打开文件、连接硬件、分配内存
    // 典型操作：
    //   - 读取配置文件
    //   - 初始化硬件连接
    //   - 创建发布器/订阅器（但不激活）
    //   - 分配缓冲区
    // ================================================================
    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_configure ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  Unconfigured → Inactive");
        RCLCPP_INFO(this->get_logger(),
            "  在此执行：初始化资源、读取配置、创建发布器");

        // 模拟初始化操作
        config_value_ = this->get_parameter("demo_param").as_int();
        RCLCPP_INFO(this->get_logger(),
            "  读取参数 demo_param = %d", config_value_);

        // 模拟偶尔失败的情况（演示错误处理）
        // return CallbackReturn::FAILURE;  // 取消注释可测试失败

        RCLCPP_INFO(this->get_logger(),
            "  on_configure 成功 → 进入 Inactive 状态");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_activate: Inactive → Active
    //
    // 含义：激活节点功能，开始发布/订阅数据
    // 典型操作：
    //   - 激活 LifecyclePublisher（调用 on_activate()）
    //   - 启动定时器
    //   - 使能硬件输出
    //   - 启用数据流
    // ================================================================
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_activate ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  Inactive → Active");
        RCLCPP_INFO(this->get_logger(),
            "  在此执行：激活发布器、启动定时器、使能硬件");

        // ⚠️ 必须调用父类的 on_activate！
        //    它负责激活所有 LifecyclePublisher
        rclcpp_lifecycle::LifecycleNode::on_activate(state);

        RCLCPP_INFO(this->get_logger(),
            "  on_activate 成功 → 进入 Active 状态");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_deactivate: Active → Inactive
    //
    // 含义：暂停节点功能，停止发布数据（但保留资源）
    // 典型操作：
    //   - 停用 LifecyclePublisher（调用 on_deactivate()）
    //   - 停止定时器
    //   - 禁用硬件输出
    //   - 保留内存和连接
    // ================================================================
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_deactivate ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  Active → Inactive");
        RCLCPP_INFO(this->get_logger(),
            "  在此执行：停用发布器、停止定时器、禁用输出");

        // ⚠️ 必须调用父类的 on_deactivate！
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

        RCLCPP_INFO(this->get_logger(),
            "  on_deactivate 成功 → 进入 Inactive 状态");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_cleanup: Inactive → Unconfigured
    //
    // 含义：释放所有资源，回到初始状态
    // 典型操作：
    //   - 释放内存
    //   - 关闭文件/连接
    //   - 重置内部状态
    //   - 销毁发布器/订阅器
    // ================================================================
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_cleanup ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  Inactive → Unconfigured");
        RCLCPP_INFO(this->get_logger(),
            "  在此执行：释放资源、关闭连接、重置状态");

        config_value_ = 0;

        RCLCPP_INFO(this->get_logger(),
            "  on_cleanup 成功 → 进入 Unconfigured 状态");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_shutdown: 任意主状态 → Finalized
    //
    // 含义：节点关闭，释放所有资源
    // 从任何主状态都可以转换到 Finalized
    // ================================================================
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "━━━ on_shutdown ━━━");
        RCLCPP_INFO(this->get_logger(),
            "  %s → Finalized", previous_state.label().c_str());
        RCLCPP_INFO(this->get_logger(),
            "  在此执行：安全关闭、释放所有资源");

        return CallbackReturn::SUCCESS;
    }

private:
    int config_value_ = 0;
    rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr transition_event_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LifecycleBasicNode>();

    // ⚠️ 生命周期节点不能用 rclcpp::spin(node)
    //    必须通过 executor 显式驱动，add_node 也需要 get_node_base_interface()
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

/*
 * ══════════════════════════════════════════════════════════
 *  命令行操作指南
 * ══════════════════════════════════════════════════════════
 *
 * # 查看当前状态
 * ros2 lifecycle list /lifecycle_basic
 * ros2 lifecycle get /lifecycle_basic
 *
 * # 手动触发状态转换
 * ros2 lifecycle set /lifecycle_basic configure    # → Inactive
 * ros2 lifecycle set /lifecycle_basic activate     # → Active
 * ros2 lifecycle set /lifecycle_basic deactivate   # → Inactive
 * ros2 lifecycle set /lifecycle_basic cleanup      # → Unconfigured
 * ros2 lifecycle set /lifecycle_basic shutdown     # → Finalized
 *
 * # 查看可用转换
 * ros2 lifecycle list /lifecycle_basic
 *
 * # 状态转换图：
 *
 *   Unconfigured ──configure──► Inactive ──activate──► Active
 *        ▲                        │  ▲                    │
 *        │                        │  │                    │
 *        └────cleanup─────────────┘  └───deactivate──────┘
 *
 *   任何状态 ──shutdown──► Finalized
 * ══════════════════════════════════════════════════════════
 */
