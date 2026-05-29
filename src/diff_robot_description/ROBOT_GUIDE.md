# 两轮差速机器人建模、仿真与 ros2_control 完全指南

> 本文档涵盖 `diff_robot_description`（建模仿真）和 `diff_robot_control`（ros2_control）两个功能包

---

## 目录

1. [差速机器人概述](#1-差速机器人概述)
2. [URDF 建模基础](#2-urdf-建模基础)
3. [物理属性与惯性参数](#3-物理属性与惯性参数)
4. [传感器建模](#4-传感器建模)
5. [Gazebo 仿真](#5-gazebo-仿真)
6. [RViz 可视化](#6-rviz-可视化)
7. [ros2_control 原理](#7-ros2_control-原理)
8. [ros2_control 架构详解](#8-ros2_control-架构详解)
9. [控制器配置与使用](#9-控制器配置与使用)
10. [硬件接口实现](#10-硬件接口实现)
11. [差速运动学](#11-差速运动学)
12. [常见问题与调试](#12-常见问题与调试)
13. [快速参考](#13-快速参考)

---

## 1. 差速机器人概述

### 1.1 什么是两轮差速机器人？

两轮差速（Differential Drive）机器人是最常见的移动机器人底盘形式：

```
              前方
               ↑
        ┌──────────────┐
        │   激光雷达    │
        │  ┌──────┐    │
        │  │camera│    │
   左轮 ─┤  └──────┘    ├─ 右轮
  (L) ●──│    IMU       │──● (R)
        │              │
        └──────●───────┘
            万向轮
```

**运动原理：**
- 两个驱动轮独立控制速度
- 通过两轮速度差实现转向
- 万向轮（ caster wheel）提供支撑，无驱动力

| 两轮速度 | 运动方式 |
|---------|---------|
| V_L = V_R | 直线前进/后退 |
| V_L > V_R | 右转 |
| V_L < V_R | 左转 |
| V_L = -V_R | 原地旋转 |

### 1.2 功能包结构

```
src/
├── diff_robot_description/          # 机器人描述包
│   ├── urdf/
│   │   ├── diff_robot.urdf.xacro    # 主 URDF 文件
│   │   ├── sensors/
│   │   │   ├── lidar.urdf.xacro    # 激光雷达宏
│   │   │   ├── camera.urdf.xacro   # 深度相机宏
│   │   │   └── imu.urdf.xacro      # IMU 宏
│   │   └── macros/
│   │       └── inertial_macros.xacro  # 惯性计算宏
│   ├── launch/
│   │   ├── display.launch.py       # RViz 显示
│   │   └── gazebo.launch.py        # Gazebo 仿真
│   ├── rviz/
│   │   └── diff_robot.rviz
│   └── config/
│
└── diff_robot_control/              # 控制包
    ├── include/diff_robot_control/
    │   └── diff_drive_hardware.hpp  # 硬件接口头文件
    ├── src/
    │   ├── diff_drive_hardware.cpp  # 硬件接口实现
    │   └── diff_odometry.cpp        # 里程计节点
    ├── config/
    │   ├── diff_bot_controllers.yaml        # 控制器配置
    │   └── mock_hardware_controllers.yaml   # Mock配置
    ├── launch/
    │   └── control.launch.py       # 控制启动
    └── diff_robot_hardware/
        └── diff_robot_hardware.xml  # 插件描述
```

---

## 2. URDF 建模基础

### 2.1 URDF 核心元素

| 元素 | 作用 | 示例 |
|------|------|------|
| `<link>` | 刚体（有质量、惯性） | 底盘、轮子、传感器 |
| `<joint>` | 连接两个 link | 固定、旋转、连续 |
| `<visual>` | 可视化几何形状 | RViz/Gazebo 显示 |
| `<collision>` | 碰撞检测几何 | 物理仿真 |
| `<inertial>` | 惯性参数 | 质量、惯性张量 |
| `<material>` | 材质/颜色 | 视觉区分 |

### 2.2 Xacro 的优势

Xacro 是 URDF 的宏预处理器，相比原始 URDF：

| 特性 | URDF | Xacro |
|------|------|-------|
| 变量定义 | 不支持 | `<xacro:property>` |
| 宏定义 | 不支持 | `<xacro:macro>` |
| 文件包含 | 不支持 | `<xacro:include>` |
| 数学运算 | 不支持 | `${expression}` |
| 条件逻辑 | 不支持 | `<xacro:if>` |
| 代码复用 | 困难 | 模块化宏 |

### 2.3 本机器人的 TF 树

```
base_footprint (Z=0, 地面投影)
    │  [fixed]
    └── base_link (底盘中心, Z=0.09m)
         │  [fixed]
         ├── caster_link (万向轮, 后下方)
         │  [continuous]         │  [continuous]
         ├── left_wheel_link     └── right_wheel_link
         │  [fixed]
         ├── laser_link (雷达, 前上方)
         │  [fixed]
         ├── imu_link (IMU, 中心偏上)
         │  [fixed]
         └── camera_link (相机, 前方)
              │  [fixed]
              ├── camera_color_optical_frame
              └── camera_depth_optical_frame
```

---

## 3. 物理属性与惯性参数

### 3.1 为什么惯性参数重要？

Gazebo 物理引擎需要惯性参数来计算：
- 牛顿-欧拉方程：F = ma, τ = Iα
- 碰撞响应力
- 摩擦力和接触力

**没有正确的惯性参数 → 仿真行为不真实！**

### 3.2 常见几何体的惯性公式

**长方体** (长l × 宽w × 高h, 质量m)：
```
Ixx = m/12 × (w² + h²)
Iyy = m/12 × (l² + h²)
Izz = m/12 × (l² + w²)
Ixy = Ixz = Iyz = 0（主轴对齐时非对角元素为0）
```

**圆柱体** (半径r × 高h, 质量m)：
```
Ixx = Iyy = m/12 × (3r² + h²)
Izz = m/2 × r²
```

**球体** (半径r, 质量m)：
```
Ixx = Iyy = Izz = 2/5 × m × r²
```

### 3.3 本机器人的惯性参数

| 部件 | 形状 | 质量(kg) | 尺寸(m) |
|------|------|---------|---------|
| 底盘 | 长方体 | 1.5 | 0.26×0.20×0.08 |
| 驱动轮 | 圆柱体 | 0.1 | R=0.033, H=0.018 |
| 万向轮 | 球体 | 0.05 | R=0.015 |
| 激光雷达 | 圆柱体 | 0.1 | R=0.03, H=0.04 |
| 相机 | 长方体 | 0.05 | 0.025×0.02×0.01 |
| IMU | 长方体 | 0.005 | 0.015×0.015×0.003 |

### 3.4 摩擦力配置

```xml
<!-- 驱动轮：高摩擦（提供驱动力） -->
<gazebo reference="left_wheel_link">
  <mu1>1.0</mu1>   <!-- 静摩擦系数 -->
  <mu2>1.0</mu2>   <!-- 动摩擦系数 -->
  <kp>1000000.0</kp>  <!-- 接触刚度 -->
  <kd>100.0</kd>       <!-- 接触阻尼 -->
  <minDepth>0.001</minDepth>
</gazebo>

<!-- 万向轮：零摩擦（允许自由滑动） -->
<gazebo reference="caster_link">
  <mu1>0.0</mu1>    <!-- 零摩擦！ -->
  <mu2>0.0</mu2>
</gazebo>
```

---

## 4. 传感器建模

### 4.1 激光雷达（LiDAR）

| 参数 | 值 | 说明 |
|------|---|------|
| 类型 | ray | Gazebo射线传感器 |
| 角度范围 | -180° ~ +180° | 360°全覆盖 |
| 采样点数 | 360 | 每度1个点 |
| 测量范围 | 0.15m ~ 12m | 近距离盲区0.15m |
| 频率 | 5Hz | 每秒5次扫描 |
| 噪声 | 高斯 σ=0.01m | 测量噪声 |
| 话题 | `/scan` | sensor_msgs/LaserScan |

### 4.2 深度相机

| 参数 | 值 | 说明 |
|------|---|------|
| 分辨率 | 640×480 | 像素 |
| FOV | 60° | 水平视野角 |
| 深度范围 | 0.05m ~ 10m | |
| 频率 | 30Hz | |
| RGB话题 | `/camera/color/image_raw` | |
| 深度话题 | `/camera/depth/image_raw` | |
| 点云话题 | `/camera/depth/points` | |

**光学坐标系约定：** ROS 图像使用光学坐标系（X右Y下Z前），需要从相机坐标系旋转：
```xml
<origin xyz="0 0 0" rpy="-1.5708 0 -1.5708" />
```

### 4.3 IMU

| 参数 | 值 | 说明 |
|------|---|------|
| 加速度噪声 | σ=0.01 m/s² | |
| 角速度噪声 | σ=0.001 rad/s | |
| 加速度偏差 | σ=0.001 m/s² | |
| 角速度偏差 | σ=0.0001 rad/s | |
| 频率 | 100Hz | |
| 话题 | `/imu/data` | sensor_msgs/Imu |

### 4.4 编码器

编码器不是独立的 Gazebo 传感器，而是通过 ros2_control 的 `state_interface` 实现：
- 轮子位置 (`position`)：累积旋转角度
- 轮子速度 (`velocity`)：角速度

通过 `joint_state_broadcaster` 发布到 `/joint_states` 话题。

---

## 5. Gazebo 仿真

### 5.1 Gazebo 插件清单

| 插件 | 文件 | 功能 |
|------|------|------|
| `libgazebo_ros_ray_sensor.so` | 雷达宏 | 激光扫描 |
| `libgazebo_ros_camera.so` | 相机宏 | RGB图像 |
| `libgazebo_ros_depth_camera.so` | 相机宏 | 深度图像+点云 |
| `libgazebo_ros_imu_sensor.so` | IMU宏 | 惯性测量 |
| `libgazebo_ros2_control.so` | 主URDF | ros2_control集成 |

### 5.2 启动 Gazebo 仿真

```bash
# 编译
colcon build --packages-select diff_robot_description diff_robot_control
source install/setup.bash

# 启动 Gazebo（仅模型+传感器）
ros2 launch diff_robot_description gazebo.launch.py

# 启动 Gazebo + ros2_control
ros2 launch diff_robot_control control.launch.py

# 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5.3 验证传感器数据

```bash
# 激光雷达
ros2 topic echo /scan
ros2 topic hz /scan

# 相机
ros2 topic echo /camera/color/image_raw
ros2 topic hz /camera/color/image_raw

# IMU
ros2 topic echo /imu/data
ros2 topic hz /imu/data

# 关节状态（编码器）
ros2 topic echo /joint_states

# 里程计
ros2 topic echo /odom
```

---

## 6. RViz 可视化

```bash
# 仅查看模型（不需要Gazebo）
ros2 launch diff_robot_description display.launch.py
```

RViz 中可显示：
- RobotModel：URDF 模型渲染
- TF：坐标系关系
- LaserScan：激光雷达数据（红点）
- Image：相机图像
- PointCloud2：点云

---

## 7. ros2_control 原理

### 7.1 为什么需要 ros2_control？

**没有 ros2_control：**
```
应用层（导航、SLAM）直接调用硬件驱动
→ 代码耦合、难以复用、切换硬件需改代码
```

**有 ros2_control：**
```
应用层（导航、SLAM）
         ↓ 标准 ROS2 话题/服务
   控制器（diff_drive_controller）
         ↓ 标准接口（command/state interface）
   硬件抽象层（hardware_interface）
         ↓
   实际硬件（Gazebo / 真实电机）
```

### 7.2 ros2_control 核心概念

```
┌─────────────────────────────────────────────────────┐
│                  controller_manager                   │
│  管理控制器的生命周期，调度 update 循环                │
│                                                       │
│  ┌──────────────┐  ┌──────────────────────────────┐  │
│  │  Controller1  │  │  Controller2                 │  │
│  │ (diff_drive)  │  │ (joint_state_broadcaster)    │  │
│  │               │  │                               │  │
│  │ 命令接口 ←──┐ │  │ ──→ 状态接口                │  │
│  │ 状态接口 ←──┤ │  │                               │  │
│  └──────────────┘  └──────────────────────────────┘  │
│                       │            │                   │
│         command_interface    state_interface           │
│                       ▼            ▼                   │
│  ┌────────────────────────────────────────────────┐  │
│  │         Hardware Interface (硬件抽象层)          │  │
│  │  - export_command_interfaces() → 命令通道       │  │
│  │  - export_state_interfaces()   → 状态通道       │  │
│  │  - read()  → 从硬件读状态                       │  │
│  │  - write() → 向硬件写命令                       │  │
│  └────────────────────────────────────────────────┘  │
│                       │                               │
│                       ▼                               │
│  ┌────────────────────────────────────────────────┐  │
│  │         实际硬件                                 │  │
│  │  Gazebo | 串口电机 | CAN总线 | Mock硬件          │  │
│  └────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### 7.3 数据流

```
控制循环（50Hz）：

1. read()
   编码器/IMU → state_interface → 硬件状态变量

2. update()
   控制器读取 state_interface
   控制器计算 → 写入 command_interface

3. write()
   command_interface → 电机驱动器
```

### 7.4 URDF 中的 ros2_control 标签

```xml
<ros2_control name="DiffBotSystem" type="system">
  <!-- 硬件接口插件 -->
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <!-- 关节定义 -->
  <joint name="left_wheel_joint">
    <command_interface name="velocity" />    <!-- 可接收速度命令 -->
    <state_interface name="position" />      <!-- 可读取位置 -->
    <state_interface name="velocity" />      <!-- 可读取速度 -->
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
</ros2_control>
```

**接口类型说明：**

| 接口名 | 方向 | 说明 |
|--------|------|------|
| `velocity` (command) | 控制器→硬件 | 速度命令 |
| `position` (command) | 控制器→硬件 | 位置命令 |
| `effort` (command) | 控制器→硬件 | 力矩命令 |
| `position` (state) | 硬件→控制器 | 当前位置 |
| `velocity` (state) | 硬件→控制器 | 当前速度 |

---

## 8. ros2_control 架构详解

### 8.1 生命周期状态机

```
          on_init()
UNCONFIGURED ──────► INACTIVE
                     │  ▲
          on_configure│  │on_deactivate
                     │  │
                     ▼  │
                    ACTIVE
                     │  ▲
          on_activate │  │
                     ▼  │
                   ACTIVE ← 控制循环运行中
```

### 8.2 硬件接口类型

| 类型 | 基类 | 用途 |
|------|------|------|
| `system` | SystemInterface | 完整系统（多关节联动） |
| `actuator` | ActuatorInterface | 单个执行器 |
| `sensor` | SensorInterface | 传感器（只读） |

**差速机器人使用 `system` 类型**，因为需要同时控制两个轮子。

### 8.3 控制器类型

| 控制器 | 包名 | 功能 |
|--------|------|------|
| JointStateBroadcaster | joint_state_broadcaster | 发布关节状态 |
| DiffDriveController | diff_drive_controller | 差速驱动控制 |
| ForwardCommandController | forward_command_controller | 直接转发命令 |
| JointTrajectoryController | joint_trajectory_controller | 关节轨迹控制 |
| VelocityController | velocity_controllers | 速度控制 |

---

## 9. 控制器配置与使用

### 9.1 DiffDriveController 详解

**输入：** `/cmd_vel` (geometry_msgs/Twist)
- `linear.x`：线速度 (m/s)
- `angular.z`：角速度 (rad/s)

**输出：**
- 轮子速度命令（通过 command_interface）
- `/odom` (nav_msgs/Odometry)：里程计
- TF: `odom → base_footprint`

**运动学逆解：**
```
v_left  = linear.x - angular.z × wheel_separation / 2
v_right = linear.x + angular.z × wheel_separation / 2
```

**运动学正解（里程计）：**
```
linear.x  = (v_right + v_left) × wheel_radius / 2
angular.z = (v_right - v_left) × wheel_radius / wheel_separation
```

### 9.2 控制器管理命令

```bash
# 列出所有控制器
ros2 control list_controllers

# 列出硬件接口
ros2 control list_hardware_interfaces

# 查看控制器状态
ros2 control list_controller_types

# 加载并激活控制器
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_bot_base_controller

# 停止控制器
ros2 control set_controller_state diff_bot_base_controller inactive

# 卸载控制器
ros2 control unload_controller diff_bot_base_controller
```

### 9.3 键盘遥控

```bash
# 启动控制
ros2 launch diff_robot_control control.launch.py

# 另一终端：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 或发布速度命令
ros2 topic pub /diff_bot_base_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

---

## 10. 硬件接口实现

### 10.1 自定义硬件接口的关键步骤

1. **继承 `SystemInterface`**
2. **实现生命周期方法**：`on_init`, `on_configure`, `on_activate`, `on_deactivate`
3. **实现读写方法**：`read()`, `write()`
4. **注册插件**：`PLUGINLIB_EXPORT_CLASS`

### 10.2 从 Gazebo 迁移到真实机器人

只需替换硬件接口插件：

```xml
<!-- Gazebo 仿真 -->
<hardware>
  <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</hardware>

<!-- 真实机器人（自定义硬件接口） -->
<hardware>
  <plugin>diff_robot_control/DiffDriveHardware</plugin>
  <param name="serial_port">/dev/ttyUSB0</param>
  <param name="baud_rate">115200</param>
</hardware>
```

**控制器代码完全不变！** 这就是 ros2_control 的价值。

---

## 11. 差速运动学

### 11.1 运动学模型

```
        ω (角速度)
         ↑
         │
    ●────┼────●
    │    │    │
  V_L   │   V_R   ← 轮子线速度
    │    │    │
    ●────┼────●
         │
         │
         └──→ v (线速度)

轮距：L = wheel_separation
轮径：r = wheel_radius
```

### 11.2 正运动学（轮速 → 机器人速度）

```python
v = (V_R + V_L) * r / 2       # 线速度
ω = (V_R - V_L) * r / L       # 角速度
```

### 11.3 逆运动学（机器人速度 → 轮速）

```python
V_L = (v - ω * L / 2) / r     # 左轮角速度
V_R = (v + ω * L / 2) / r     # 右轮角速度
```

### 11.4 里程计计算

```python
# 欧拉法
x_new     = x + v * cos(θ) * dt
y_new     = y + v * sin(θ) * dt
θ_new     = θ + ω * dt

# 中点法（更准确）
x_new     = x + v * cos(θ + ω*dt/2) * dt
y_new     = y + v * sin(θ + ω*dt/2) * dt
θ_new     = θ + ω * dt
```

---

## 12. 常见问题与调试

### 12.1 Gazebo 仿真问题

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| 机器人在 Gazebo 中不停抖动 | 惯性参数错误 | 检查 `<inertial>` 标签 |
| 机器人掉入地面 | 碰撞体定义错误 | 检查 `<collision>` 和 `<minDepth>` |
| 轮子打滑 | 摩擦系数太低 | 增大 `<mu1>` 和 `<mu2>` |
| 万向轮卡住 | 万向轮摩擦力太大 | 设置 `<mu1>=0.0` |
| 传感器无数据 | Gazebo 插件配置错误 | 检查 `<plugin>` 的 filename |

### 12.2 ros2_control 问题

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| 控制器加载失败 | YAML 配置错误 | 检查控制器类型名 |
| /cmd_vel 无响应 | 控制器话题名不匹配 | 检查 `cmd_vel` 重映射 |
| /joint_states 无数据 | joint_state_broadcaster 未启动 | `ros2 control load_controller joint_state_broadcaster` |
| 里程计漂移 | wheel_separation/ wheel_radius 不准确 | 标定参数 |
| URDF 中找不到 ros2_control 标签 | xacro 未正确处理 | 检查 xacro 命令 |

### 12.3 调试命令

```bash
# 检查 URDF 是否正确
check_urdf diff_robot.urdf

# 检查 xacro 输出
xacro diff_robot.urdf.xacro > test.urdf

# 查看控制器状态
ros2 control list_controllers

# 查看硬件接口
ros2 control list_hardware_interfaces

# 查看话题
ros2 topic list
ros2 topic echo /joint_states

# 查看参数
ros2 param list
```

---

## 13. 快速参考

### 13.1 编译与运行

```bash
# 编译
colcon build --packages-select diff_robot_description diff_robot_control
source install/setup.bash

# RViz 查看模型
ros2 launch diff_robot_description display.launch.py

# Gazebo 仿真（仅传感器）
ros2 launch diff_robot_description gazebo.launch.py

# Gazebo + ros2_control
ros2 launch diff_robot_control control.launch.py

# 键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 13.2 传感器话题速查

```bash
ros2 topic echo /scan                       # 激光雷达
ros2 topic echo /camera/color/image_raw     # RGB图像
ros2 topic echo /camera/depth/image_raw     # 深度图像
ros2 topic echo /camera/depth/points        # 点云
ros2 topic echo /imu/data                    # IMU
ros2 topic echo /joint_states                # 编码器（关节状态）
ros2 topic echo /odom                        # 里程计
ros2 topic echo /cmd_vel                     # 速度命令
```

### 13.3 URDF/Xacro 速查

```xml
<!-- Link：刚体 -->
<link name="my_link">
  <visual>...</visual>      <!-- 可视化 -->
  <collision>...</collision> <!-- 碰撞体 -->
  <inertial>...</inertial>  <!-- 惯性参数 -->
</link>

<!-- Joint：连接 -->
<joint name="my_joint" type="fixed|continuous|revolute">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="0 1 0" />     <!-- 旋转轴 -->
</joint>

<!-- Gazebo 属性 -->
<gazebo reference="link_name">
  <material>Gazebo/Grey</material>
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
</gazebo>

<!-- ros2_control -->
<ros2_control name="SystemName" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="joint_name">
    <command_interface name="velocity" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
</ros2_control>
```
