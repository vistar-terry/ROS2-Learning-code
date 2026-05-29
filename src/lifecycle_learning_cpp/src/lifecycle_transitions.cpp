/**
 * @file lifecycle_transitions.cpp
 * @brief ② 状态转换详解 —— 每个转换的含义、触发条件、返回值
 *
 * 知识点：
 * - 4个主状态和7个过渡状态
 * - 合法转换路径（不能跳状态！）
 * - CallbackReturn::SUCCESS / FAILURE / ERROR
 * - 错误恢复机制
 * - 过渡状态（Transitioning/Configuring/Activating...）
 */

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>

using namespace std::chrono_literals;

class LifecycleTransitionsNode : public rclcpp_lifecycle::LifecycleNode {
public:
    LifecycleTransitionsNode()
        : rclcpp_lifecycle::LifecycleNode("lifecycle_transitions"),
          configure_count_(0), activate_count_(0)
    {
        RCLCPP_INFO(this->get_logger(),
            "=== 状态转换详解节点已创建 ===");
        RCLCPP_INFO(this->get_logger(),
            "当前状态: %s", this->get_current_state().label().c_str());

        // ================================================================
        // 定时器：周期报告当前状态
        //    在 Active 状态才启动
        // ================================================================
        status_timer_ = this->create_wall_timer(
            1s,
            [this]() {
                auto state = this->get_current_state();
                RCLCPP_INFO(this->get_logger(),
                    "[状态报告] 当前状态: %s (id=%d)",
                    state.label().c_str(), state.id());
            });
        // 初始时不启动定时器，等 on_activate 中再启动
        status_timer_->cancel();
    }

    // ================================================================
    // on_configure: Unconfigured → Inactive
    //    只能从 Unconfigured 状态调用！
    //    如果从其他状态调用会返回错误
    // ================================================================
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
    {
        configure_count_++;
        RCLCPP_INFO(this->get_logger(),
            "┌─── on_configure #%d ───┐", configure_count_);
        RCLCPP_INFO(this->get_logger(),
            "│  从状态: %s (id=%d)",
            previous_state.label().c_str(), previous_state.id());
        RCLCPP_INFO(this->get_logger(),
            "│  目标状态: Inactive");

        // ================================================================
        // 返回值说明：
        //
        // CallbackReturn::SUCCESS → 转换成功，进入目标状态
        //   Unconfigured → Inactive ✓
        //
        // CallbackReturn::FAILURE → 转换失败，回到之前的状态
        //   Unconfigured → Inactive 失败 → 仍为 Unconfigured
        //
        // CallbackReturn::ERROR → 严重错误，进入 ErrorProcessing
        //   然后自动尝试 on_error 回调
        //   如果 on_error 返回 SUCCESS → 回到 Unconfigured
        //   如果 on_error 返回 FAILURE → 进入 Finalized（节点死亡）
        // ================================================================

        // 模拟：第2次 configure 失败（演示错误恢复）
        if (configure_count_ == 2) {
            RCLCPP_WARN(this->get_logger(),
                "│  模拟配置失败！(第2次)");
            RCLCPP_INFO(this->get_logger(),
                "└─── on_configure FAILURE ───┘");
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(this->get_logger(),
            "│  配置成功");
        RCLCPP_INFO(this->get_logger(),
            "└─── on_configure SUCCESS ───┘");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
    {
        activate_count_++;
        RCLCPP_INFO(this->get_logger(),
            "┌─── on_activate #%d ───┐", activate_count_);
        RCLCPP_INFO(this->get_logger(),
            "│  从状态: %s", previous_state.label().c_str());

        // 启动状态报告定时器
        status_timer_->reset();

        // ⚠️ 必须调用父类的 on_activate
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

        RCLCPP_INFO(this->get_logger(),
            "│  激活成功");
        RCLCPP_INFO(this->get_logger(),
            "└─── on_activate SUCCESS ───┘");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "┌─── on_deactivate ───┐");
        RCLCPP_INFO(this->get_logger(),
            "│  从状态: %s", previous_state.label().c_str());

        // 停止定时器
        status_timer_->cancel();

        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);

        RCLCPP_INFO(this->get_logger(),
            "│  停用成功");
        RCLCPP_INFO(this->get_logger(),
            "└─── on_deactivate SUCCESS ───┘");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "┌─── on_cleanup ───┐");
        RCLCPP_INFO(this->get_logger(),
            "│  从状态: %s", previous_state.label().c_str());
        RCLCPP_INFO(this->get_logger(),
            "│  清理所有资源");
        RCLCPP_INFO(this->get_logger(),
            "└─── on_cleanup SUCCESS ───┘");
        return CallbackReturn::SUCCESS;
    }

    // ================================================================
    // on_error: 错误处理回调
    //    当任何转换回调返回 ERROR 时自动触发
    //    必须返回 SUCCESS（回到 Unconfigured）或 FAILURE（进入 Finalized）
    // ================================================================
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override
    {
        RCLCPP_ERROR(this->get_logger(),
            "┌─── on_error ───┐");
        RCLCPP_ERROR(this->get_logger(),
            "│  从状态: %s", previous_state.label().c_str());
        RCLCPP_ERROR(this->get_logger(),
            "│  发生严重错误，尝试恢复...");

        // 尝试恢复：释放资源，回到 Unconfigured
        // 如果恢复成功 → 返回 SUCCESS → 状态变为 Unconfigured
        // 如果恢复失败 → 返回 FAILURE → 状态变为 Finalized（节点不可用）

        RCLCPP_ERROR(this->get_logger(),
            "│  错误恢复成功");
        RCLCPP_ERROR(this->get_logger(),
            "└─── on_error SUCCESS → 回到 Unconfigured ───┘");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
    {
        RCLCPP_INFO(this->get_logger(),
            "┌─── on_shutdown ───┐");
        RCLCPP_INFO(this->get_logger(),
            "│  从状态: %s", previous_state.label().c_str());
        RCLCPP_INFO(this->get_logger(),
            "└─── on_shutdown SUCCESS ───┘");
        return CallbackReturn::SUCCESS;
    }

private:
    int configure_count_;
    int activate_count_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleTransitionsNode>();
    // 生命周期节点必须用 executor 驱动
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

/*
 * ══════════════════════════════════════════════════════════
 *  完整状态转换图
 * ══════════════════════════════════════════════════════════
 *
 * 4 个主状态:
 *   [1] Unconfigured  — 刚创建，未配置
 *   [2] Inactive      — 已配置，但未激活
 *   [3] Active        — 正在工作
 *   [4] Finalized     — 已关闭
 *
 * 7 个过渡状态（转换过程中短暂存在）:
 *   [10] Configuring       — 正在执行 on_configure
 *   [11] CleaningUp        — 正在执行 on_cleanup
 *   [12] ShuttingDown      — 正在执行 on_shutdown
 *   [13] Activating        — 正在执行 on_activate
 *   [14] Deactivating      — 正在执行 on_deactivate
 *   [15] ErrorProcessing   — 正在执行 on_error
 *
 * 合法转换路径:
 *
 *   Unconfigured ──[configure]──► Configuring ──[on_configure SUCCESS]──► Inactive
 *   Unconfigured ──[shutdown]──► ShuttingDown ──[on_shutdown SUCCESS]──► Finalized
 *
 *   Inactive ──[activate]──► Activating ──[on_activate SUCCESS]──► Active
 *   Inactive ──[cleanup]──► CleaningUp ──[on_cleanup SUCCESS]──► Unconfigured
 *   Inactive ──[shutdown]──► ShuttingDown ──[on_shutdown SUCCESS]──► Finalized
 *
 *   Active ──[deactivate]──► Deactivating ──[on_deactivate SUCCESS]──► Inactive
 *   Active ──[shutdown]──► ShuttingDown ──[on_shutdown SUCCESS]──► Finalized
 *
 * 失败路径:
 *   任何 ──[FAILURE]──► 回到原状态
 *   任何 ──[ERROR]──► ErrorProcessing ──[on_error SUCCESS]──► Unconfigured
 *                                           ──[on_error FAILURE]──► Finalized
 * ══════════════════════════════════════════════════════════
 */
