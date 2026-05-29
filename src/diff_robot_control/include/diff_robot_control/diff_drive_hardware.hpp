/**
 * @file diff_drive_hardware.hpp
 * @brief 两轮差速机器人硬件接口头文件
 * 
 * 功能：实现 ros2_control 的 SystemInterface
 * 用途：不使用 Gazebo 时的 mock 硬件接口，或真实机器人的硬件驱动
 * 
 * 知识点：
 * - hardware_interface::SystemInterface 是 ros2_control 的核心抽象
 * - 需要实现 on_init, on_configure, on_activate, on_deactivate 等生命周期方法
 * - 需要声明 command_interface 和 state_interface
 * - read() 从硬件读取状态，write() 向硬件写入命令
 */

#ifndef DIFF_ROBOT_CONTROL__DIFF_DRIVE_HARDWARE_HPP_
#define DIFF_ROBOT_CONTROL__DIFF_DRIVE_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace diff_robot_control
{

class DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware)

    // ============================================================
    // 生命周期回调方法
    // ============================================================

    /**
     * on_init：初始化硬件接口
     * 在控制器管理器加载硬件时调用
     * 从 URDF 的 <ros2_control> 标签中读取配置
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;

    /**
     * on_configure：配置硬件
     * 在 State::UNCONFIGURED → State::INACTIVE 转换时调用
     * 用于硬件的初始化配置（如打开串口、连接驱动器等）
     */
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    /**
     * on_activate：激活硬件
     * 在 State::INACTIVE → State::ACTIVE 转换时调用
     * 用于启动硬件（如使能电机、开始读取传感器等）
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    /**
     * on_deactivate：停用硬件
     * 在 State::ACTIVE → State::INACTIVE 转换时调用
     * 用于安全停止硬件
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    // ============================================================
    // 数据读写方法
    // ============================================================

    /**
     * read：从硬件读取状态
     * 在每个控制循环中调用
     * 将硬件的实际状态写入 state_interface
     * 
     * 执行顺序：read() → update(controller) → write()
     */
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    /**
     * write：向硬件写入命令
     * 在每个控制循环中调用
     * 从 command_interface 读取控制器计算出的命令，发送给硬件
     */
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // ============================================================
    // 硬件状态变量
    // 这些变量存储关节的位置和速度
    // read() 更新这些值，write() 使用这些值
    // ============================================================

    // 左轮状态
    double hw_left_wheel_position_ = 0.0;    // 位置（弧度）
    double hw_left_wheel_velocity_ = 0.0;    // 速度（弧度/秒）

    // 右轮状态
    double hw_right_wheel_position_ = 0.0;
    double hw_right_wheel_velocity_ = 0.0;

    // 左轮命令
    double hw_left_wheel_cmd_ = 0.0;         // 速度命令（弧度/秒）

    // 右轮命令
    double hw_right_wheel_cmd_ = 0.0;

    // 硬件是否就绪
    bool hw_is_ready_ = false;
};

}  // namespace diff_robot_control

#endif  // DIFF_ROBOT_CONTROL__DIFF_DRIVE_HARDWARE_HPP_
