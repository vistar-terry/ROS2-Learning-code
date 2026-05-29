/**
 * @file diff_drive_hardware.cpp
 * @brief 两轮差速机器人硬件接口实现
 * 
 * 功能：实现 ros2_control 的 SystemInterface，作为 mock 硬件或真实硬件驱动
 * 
 * 知识点：
 * - SystemInterface 的生命周期：init → configure → activate → (read/write循环) → deactivate
 * - command_interface：控制器→硬件的方向（如速度命令）
 * - state_interface：硬件→控制器的方向（如位置/速度反馈）
 * - 插件注册：PLUGINLIB_EXPORT_CLASS 宏将类注册为 ros2_control 插件
 * 
 * 架构图：
 *   ┌─────────────────────────────────────────────────────┐
 *   │               controller_manager                      │
 *   │  ┌──────────────┐    ┌──────────────────────────┐   │
 *   │  │ DiffDrive     │    │ JointStateBroadcaster     │   │
 *   │  │ Controller    │    │ (读取状态→发布话题)        │   │
 *   │  └──────┬───────┘    └───────────┬──────────────┘   │
 *   │         │  command_interface      │ state_interface   │
 *   │         ▼                        ▼                    │
 *   │  ┌─────────────────────────────────────────────┐    │
 *   │  │        DiffDriveHardware (本文件)             │    │
 *   │  │  read(): 获取编码器/IMU数据 → state_interface │    │
 *   │  │  write(): command_interface → 发送电机命令    │    │
 *   │  └─────────────────────────────────────────────┘    │
 *   └─────────────────────────────────────────────────────┘
 *                      │
 *                      ▼
 *              ┌───────────────┐
 *              │  实际硬件      │  ← 电机驱动器、编码器、IMU
 *              │  (串口/CAN)   │
 *              └───────────────┘
 */

#include "diff_robot_control/diff_drive_hardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace diff_robot_control
{

// ============================================================
// on_init：初始化硬件接口
// ============================================================
hardware_interface::CallbackReturn DiffDriveHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    // ============================================================
    // 1. 调用父类的 on_init
    //    这会解析 URDF 中的 <ros2_control> 标签
    //    提取关节信息和硬件参数
    // ============================================================
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ============================================================
    // 2. 读取 URDF 中定义的参数
    //    通过 params.hardware_info.hardware_parameters 可以访问 <param> 标签
    // ============================================================
    // 示例：读取串口设备路径
    // std::string serial_port = params.hardware_info.hardware_parameters["serial_port"];

    // ============================================================
    // 3. 验证关节数量
    //    确保与 URDF 中定义的关节数一致
    // ============================================================
    if (params.hardware_info.joints.size() != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"),
            "期望2个关节（左右轮），实际有 %zu 个",
            params.hardware_info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 验证每个关节的命令接口
    for (const auto & joint : params.hardware_info.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveHardware"),
                "关节 '%s' 应有1个命令接口，实际有 %zu 个",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"),
        "硬件接口初始化完成，共 %zu 个关节", params.hardware_info.joints.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure：配置硬件
// ============================================================
hardware_interface::CallbackReturn DiffDriveHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // ============================================================
    // 在这里进行硬件的配置操作，例如：
    // - 打开串口连接
    // - 初始化驱动器
    // - 校准编码器
    // - 初始化 IMU
    // ============================================================

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"),
        "硬件接口已配置");

    // 重置状态变量
    hw_left_wheel_position_ = 0.0;
    hw_left_wheel_velocity_ = 0.0;
    hw_right_wheel_position_ = 0.0;
    hw_right_wheel_velocity_ = 0.0;
    hw_left_wheel_cmd_ = 0.0;
    hw_right_wheel_cmd_ = 0.0;

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate：激活硬件
// ============================================================
hardware_interface::CallbackReturn DiffDriveHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // ============================================================
    // 在这里激活硬件，例如：
    // - 使能电机驱动器
    // - 开始编码器计数
    // - 发送启动命令
    // ============================================================

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"),
        "硬件接口已激活，开始控制循环");

    hw_is_ready_ = true;

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate：停用硬件
// ============================================================
hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // ============================================================
    // 在这里安全停止硬件，例如：
    // - 禁用电机驱动器
    // - 发送停止命令
    // - 关闭串口
    // ============================================================

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"),
        "硬件接口已停用");

    hw_is_ready_ = false;

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// read：从硬件读取状态
// 在每个控制循环中被调用（频率由 update_rate 决定）
// ============================================================
hardware_interface::return_type DiffDriveHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    if (!hw_is_ready_)
    {
        return hardware_interface::return_type::OK;
    }

    // ============================================================
    // 1. 从实际硬件读取编码器值
    //    在真实机器人中，这里会：
    //    - 通过串口/CAN 读取编码器计数
    //    - 将计数转换为弧度
    //    - 计算速度（差分法）
    //
    //    在 mock 硬件中，我们模拟这个过程：
    //    使用 write() 中设置的命令来模拟硬件响应
    // ============================================================

    // 模拟：位置 = 上次位置 + 速度 × 时间步长
    double dt = period.seconds();
    hw_left_wheel_position_ += hw_left_wheel_cmd_ * dt;
    hw_right_wheel_position_ += hw_right_wheel_cmd_ * dt;

    // 速度 = 当前命令（假设电机响应足够快）
    hw_left_wheel_velocity_ = hw_left_wheel_cmd_;
    hw_right_wheel_velocity_ = hw_right_wheel_cmd_;

    // ============================================================
    // 2. 状态已经通过 export_state_interfaces() 暴露
    //    这里只需更新成员变量，框架会自动将变量值
    //    同步到对应的 state_interface
    // ============================================================

    return hardware_interface::return_type::OK;
}

// ============================================================
// write：向硬件写入命令
// 在每个控制循环中被调用
// ============================================================
hardware_interface::return_type DiffDriveHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!hw_is_ready_)
    {
        return hardware_interface::return_type::OK;
    }

    // ============================================================
    // 1. 从 command_interface 读取控制器计算出的速度命令
    //    hw_left_wheel_cmd_ 和 hw_right_wheel_cmd_ 已经通过
    //    export_command_interfaces() 与 command_interface 绑定
    //    控制器写入 command_interface 时，自动更新这些变量
    // ============================================================

    // ============================================================
    // 2. 向实际硬件发送命令
    //    在真实机器人中，这里会：
    //    - 将速度命令转换为电机驱动器协议
    //    - 通过串口/CAN 发送给驱动器
    //    - 加入安全检查（速度限制、急停等）
    //
    //    在 mock 硬件中，命令已经通过 command_interface 绑定
    //    不需要额外操作
    // ============================================================

    // 安全检查：限制最大速度
    double max_wheel_speed = 10.0;  // rad/s
    hw_left_wheel_cmd_ = std::max(-max_wheel_speed,
        std::min(max_wheel_speed, hw_left_wheel_cmd_));
    hw_right_wheel_cmd_ = std::max(-max_wheel_speed,
        std::min(max_wheel_speed, hw_right_wheel_cmd_));

    return hardware_interface::return_type::OK;
}

}  // namespace diff_robot_control

// ============================================================
// 插件注册宏
// 将 DiffDriveHardware 注册为 ros2_control 的硬件接口插件
// 第一个参数：类名
// 第二个参数：基类名
// 
// 注册后，可以在 URDF 中通过 <plugin> 标签引用：
// <plugin>diff_robot_control/DiffDriveHardware</plugin>
// ============================================================
PLUGINLIB_EXPORT_CLASS(
    diff_robot_control::DiffDriveHardware,
    hardware_interface::SystemInterface
)
