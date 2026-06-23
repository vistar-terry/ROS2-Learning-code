/**
 * ===========================================================================
 *  ① 默认行为演示 —— 单线程执行器下的回调阻塞问题 (C++版)
 * ===========================================================================
 *
 * 核心知识点：
 *   - ROS2 默认使用 SingleThreadedExecutor（单线程执行器）
 *   - 节点的所有回调默认属于同一个 MutuallyExclusiveCallbackGroup
 *   - 同一 MutuallyExclusiveCallbackGroup 中的回调不能并发执行
 *   - 慢回调会阻塞同组的快回调
 *
 * C++ 回调组 API：
 *   - node->create_callback_group(CallbackGroupType::MutuallyExclusive)
 *   - node->create_callback_group(CallbackGroupType::Reentrant)
 *   - 不指定时，回调默认加入 node 的默认回调组（MutuallyExclusive）
 * ===========================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

/**
 * 默认行为演示节点
 * 演示：两个定时器都在默认回调组中，慢回调阻塞快回调
 */
class DefaultBehaviorNode : public rclcpp::Node
{
public:
    DefaultBehaviorNode() : Node("default_behavior_node"), slow_count_(0), fast_count_(0)
    {
        // ================================================================
        // 1. 默认回调组说明
        //    每个节点都有一个默认回调组，类型为 MutuallyExclusiveCallbackGroup
        //    不指定 callback_group 时，回调自动加入默认回调组
        //    注意：ROS2 Jazzy 中没有 get_default_callback_group() 方法，
        //    但默认回调组的 MutuallyExclusive 行为不变
        // ================================================================
        RCLCPP_INFO(this->get_logger(), "默认回调组类型: MutuallyExclusiveCallbackGroup");

        // ================================================================
        // 2. 创建两个定时器 —— 都使用默认回调组（不指定 callback_group）
        //    由于默认回调组是 MutuallyExclusive 类型，
        //    所以 slow_callback 执行时，fast_callback 会被阻塞
        // ================================================================

        // 慢回调：模拟耗时2秒的处理
        slow_timer_ = this->create_wall_timer(
            2s, // 每2秒触发一次
            std::bind(&DefaultBehaviorNode::slow_callback, this)
            // 注意：没有指定 callback_group，默认使用节点的 default_callback_group
        );

        // 快回调：轻量处理
        fast_timer_ = this->create_wall_timer(
            500ms, // 每0.5秒触发一次
            std::bind(&DefaultBehaviorNode::fast_callback, this)
            // 同样使用默认回调组
        );

        RCLCPP_INFO(this->get_logger(), "================================================================");
        RCLCPP_INFO(this->get_logger(), "默认行为演示启动");
        RCLCPP_INFO(this->get_logger(), "  慢回调: 每2秒触发，执行耗时2秒");
        RCLCPP_INFO(this->get_logger(), "  快回调: 每0.5秒触发，执行耗时<1ms");
        RCLCPP_INFO(this->get_logger(), "  回调组: 两个回调都在默认 MutuallyExclusiveCallbackGroup 中");
        RCLCPP_INFO(this->get_logger(), "  预期: 慢回调执行期间，快回调被阻塞，无法按时执行");
        RCLCPP_INFO(this->get_logger(), "================================================================");
    }

private:
    /**
     * 慢回调 —— 模拟耗时2秒的处理
     * 在默认回调组中，执行期间其他回调必须等待
     */
    void slow_callback()
    {
        slow_count_++;
        auto start = std::chrono::steady_clock::now();
        auto start_sec = std::chrono::duration<double>(
                             start.time_since_epoch())
                             .count();
        RCLCPP_WARN(this->get_logger(),
                    "[慢回调 #%d] 开始执行 (t=%.2f)", slow_count_, start_sec);

        // 模拟耗时处理（例如：图像处理、复杂计算、阻塞IO）
        std::this_thread::sleep_for(2s);

        auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
        RCLCPP_WARN(this->get_logger(),
                    "[慢回调 #%d] 执行完成，耗时 %.2fs", slow_count_, elapsed);
    }

    /**
     * 快回调 —— 轻量处理
     * 由于和慢回调在同一个 MutuallyExclusive 回调组中，
     * 慢回调执行期间，快回调必须排队等待
     */
    void fast_callback()
    {
        fast_count_++;
        auto now_sec = std::chrono::duration<double>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
        RCLCPP_INFO(this->get_logger(),
                    "[快回调 #%d] 执行 (t=%.2f)", fast_count_, now_sec);
    }

    // 定时器
    rclcpp::TimerBase::SharedPtr slow_timer_;
    rclcpp::TimerBase::SharedPtr fast_timer_;

    // 计数器
    int slow_count_;
    int fast_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DefaultBehaviorNode>();

    // ================================================================
    // 3. 使用默认的 SingleThreadedExecutor
    //    rclcpp::spin() 内部创建 SingleThreadedExecutor
    //    单线程依次执行回调 —— 同一 MutuallyExclusive 回调组中的回调互斥
    // ================================================================
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(),
                "\n统计: 慢回调 %d 次, 快回调 %d 次\n"
                "注意：快回调的触发次数远少于预期（0.5s间隔），因为被慢回调阻塞了",
                0, 0); // 注意：spin返回后无法获取成员变量，仅做演示

    rclcpp::shutdown();
    return 0;
}
