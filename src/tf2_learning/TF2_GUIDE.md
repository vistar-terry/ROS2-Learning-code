# ROS2 TF2 坐标变换完全指南

> 本文档是 `tf2_learning` 和 `tf2_learning_py` 功能包的配套学习文档，涵盖TF2原理、命令行工具、C++/Python代码实现及机器人实际应用案例。

---

## 目录

1. [TF2 概述与原理](#1-tf2-概述与原理)
2. [坐标变换数学基础](#2-坐标变换数学基础)
3. [TF2 核心概念](#3-tf2-核心概念)
4. [TF2 命令行工具](#4-tf2-命令行工具)
5. [C++ 代码实现](#5-c-代码实现)
6. [Python 代码实现](#6-python-代码实现)
7. [机器人实际应用案例](#7-机器人实际应用案例)
8. [REP-105 坐标系约定](#8-rep-105-坐标系约定)
9. [TF2 高级话题](#9-tf2-高级话题)
10. [常见问题与调试](#10-常见问题与调试)
11. [快速参考卡片](#11-快速参考卡片)

---

## 1. TF2 概述与原理

### 1.1 什么是 TF2？

**TF2**（Transform Framework 2）是 ROS2 中用于管理坐标系之间变换关系的核心库。它解决了机器人系统中最基本的问题之一：

> **"这个传感器检测到的障碍物，在机器人底盘坐标系中位于哪里？"**

一个典型的机器人包含多个传感器和执行器，每个都安装在特定位置：

```
                激光雷达 (laser_link)
                    |
    摄像头           |          IMU (imu_link)
  (camera_link)      |           /
         \           |          /
          \          |         /
           ┌─────────────────────┐
           │    base_link        │  ← 机器人底盘中心
           │    (底盘坐标系)      │
           └─────────────────────┘
                    |
              base_footprint      ← 地面投影
                    |
                 地面 (odom)
```

**TF2 的核心作用：**
- 在任意两个坐标系之间进行坐标变换
- 管理随时间变化的动态坐标系关系
- 高效地存储和查询变换历史
- 自动处理坐标系的连接路径（TF树）

### 1.2 TF2 vs TF1

| 特性 | TF1 (ROS1) | TF2 (ROS2) |
|------|-----------|-----------|
| 语言支持 | C++/Python | C++/Python |
| 时间机制 | ros::Time | rclcpp::Time / builtin_interfaces::Time |
| 线程安全 | 部分 | 完全线程安全 |
| 性能 | 较慢 | 优化后的BufferCore |
| 静态变换 | 反复发布 | 一次发布，持久缓存 |
| 话题名称 | /tf, /tf_static | /tf, /tf_static |
| 异常处理 | tf::TransformException | tf2::TransformException系列 |

### 1.3 TF2 架构图

```
┌─────────────────────────────────────────────────────────┐
│                     TF2 系统架构                         │
│                                                         │
│  ┌──────────────┐    /tf     ┌───────────────────┐     │
│  │   Broadcaster │──────────▶│                   │     │
│  │  (发布变换)    │           │                   │     │
│  └──────────────┘            │   BufferCore      │     │
│                              │  (存储+插值+图搜索) │     │
│  ┌──────────────┐  /tf_static│                   │     │
│  │  Static       │──────────▶│                   │     │
│  │  Broadcaster  │           └────────┬──────────┘     │
│  └──────────────┘                     │                 │
│                                       │                 │
│  ┌──────────────┐                     ▼                 │
│  │   Listener    │◀───────────────────                  │
│  │  (查询变换)    │   lookupTransform / canTransform     │
│  └──────────────┘                                       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 1.4 TF2 的工作流程

```
1. 广播器发布变换 ──▶ /tf 或 /tf_static 话题
2. 监听器订阅话题 ──▶ 存入 BufferCore 缓冲区
3. 用户查询变换  ──▶ BufferCore 搜索TF树 + 时间插值
4. 返回结果      ──▶ TransformStamped 消息
```

---

## 2. 坐标变换数学基础

### 2.1 刚体变换

在三维空间中，一个坐标系到另一个坐标系的变换由**旋转**和**平移**两部分组成：

```
p_B = R_AB * p_A + t_AB
```

其中：
- `p_A`：点在坐标系A中的坐标
- `p_B`：同一点在坐标系B中的坐标
- `R_AB`：从A到B的旋转矩阵（3×3）
- `t_AB`：从A到B的平移向量（3×1）

### 2.2 齐次坐标

为了方便连续变换的运算，使用4×4齐次变换矩阵：

```
┌       ┐   ┌         ┐ ┌     ┐
│ p_B   │   │ R_AB  t │ │ p_A │
│ 1     │ = │ 0     1 │ │  1  │
└       ┘   └         ┘ └     ┘

T_AB = ┌ R_AB  t_AB ┐
       │ 0     1    │
       └            ┘
```

**连续变换的性质：**
```
T_AC = T_AB * T_BC
```

### 2.3 旋转的表示方法

| 表示方法 | 参数个数 | 优点 | 缺点 |
|---------|---------|------|------|
| **欧拉角** (RPY) | 3 | 直观，易理解 | 万向锁问题 |
| **四元数** | 4 | 无万向锁，插值平滑 | 不直观 |
| **旋转矩阵** | 9 | 直接运算 | 冗余参数多 |
| **轴角** | 4 | 物理意义明确 | 不常用 |

### 2.4 四元数详解

四元数是TF2中旋转的标准表示：

```
q = w + xi + yj + zk = (x, y, z, w)
```

**性质：**
- 单位四元数：`x² + y² + z² + w² = 1`
- 旋转角度θ，旋转轴(a, b, c)：
  - `w = cos(θ/2)`
  - `x = a * sin(θ/2)`
  - `y = b * sin(θ/2)`
  - `z = c * sin(θ/2)`

**欧拉角 → 四元数：**
```cpp
// C++
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);  // 弧度制
```
```python
# Python
import math
def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    w = cr*cp*cy + sr*sp*sy
    return (x, y, z, w)
```

**四元数 → 欧拉角：**
```cpp
// C++
tf2::Matrix3x3 m(q);
double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
```

### 2.5 ROS2 中的坐标系约定

- **右手坐标系**：X前，Y左，Z上
- **欧拉角顺序**：Roll(X) → Pitch(Y) → Yaw(Z)
- **角度正方向**：从轴正方向看，逆时针为正

```
        Z (上)
        │
        │
        │───── Y (左)
       ╱
      ╱
     X (前)
```

---

## 3. TF2 核心概念

### 3.1 TF树（TF Tree）

TF2中所有坐标系构成一棵有向树：

```
                    map (全局坐标系)
                     │
                   odom (里程计坐标系)
                     │
                 base_link (机器人底盘)
                ╱    │    ╲       ╲
    base_footprint  imu   laser  camera
```

**关键规则：**
1. TF树中**不能有环**（不能形成循环依赖）
2. 两个坐标系之间**只有一条路径**
3. 每个坐标系只有一个父坐标系
4. `map`和`odom`是根坐标系（无父节点）

### 3.2 变换方向

TF2中的变换是有方向的：

```
parent_frame ──▶ child_frame

含义：child_frame 在 parent_frame 中的位置和姿态
```

**查询方向可以反转：**
```cpp
// 正向查询：从 base_link 到 laser_link
tf_buffer->lookupTransform("base_link", "laser_link", time);

// 反向查询：从 laser_link 到 base_link（TF2自动求逆）
tf_buffer->lookupTransform("laser_link", "base_link", time);
```

### 3.3 时间机制

TF2的核心特性之一是**时间感知**：

- 每个变换都有一个时间戳
- BufferCore存储变换历史
- 查询特定时间的变换时，进行线性插值
- 缓存有大小限制（默认10秒）

```
时间轴: ───t1────t2────t3────t4────t5────now──▶
              │     │     │     │     │
变换数据:     T1    T2    T3    T4    T5

查询 t3 时刻 → 直接返回 T3
查询 t2.5 时刻 → 在 T2 和 T3 之间线性插值
查询 t0 时刻 → ExtrapolationException（超出缓存范围）
```

### 3.4 话题与消息

| 话题 | 消息类型 | 用途 |
|-----|---------|------|
| `/tf` | `tf2_msgs/TFMessage` | 动态变换（随时间变化） |
| `/tf_static` | `tf2_msgs/TFMessage` | 静态变换（固定不变） |

**TransformStamped 消息结构：**
```
std_msgs/Header header
  builtin_interfaces/Time stamp    # 时间戳
  string frame_id                   # 父坐标系
string child_frame_id               # 子坐标系
geometry_msgs/Transform transform
  geometry_msgs/Vector3 translation  # 平移 (x, y, z)
  geometry_msgs/Quaternion rotation  # 旋转 (x, y, z, w)
```

### 3.5 核心类说明

| 类名 | 作用 | 头文件/模块 |
|-----|------|------------|
| `TransformBroadcaster` | 发布动态变换 | `tf2_ros/transform_broadcaster.h` |
| `StaticTransformBroadcaster` | 发布静态变换 | `tf2_ros/static_transform_broadcaster.h` |
| `Buffer` | 存储变换，支持查询 | `tf2_ros/buffer.h` |
| `TransformListener` | 订阅TF话题，填充Buffer | `tf2_ros/transform_listener.h` |

---

## 4. TF2 命令行工具

### 4.1 查看TF变换：tf2_echo

实时打印两个坐标系之间的变换：

```bash
# 语法：ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>

# 查看最新的变换
ros2 run tf2_ros tf2_echo base_link laser_link

# 输出示例：
# At time 1234.567
# - Translation: [0.300, 0.000, 0.150]
# - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
# - Rotation: in RPY (radian) [0.000, -0.000, 0.000]
# - Rotation: in RPY (degree) [0.000, -0.000, 0.000]
```

### 4.2 查看TF树：view_frames

生成TF树的PDF图：

```bash
# 生成 frames.pdf 文件
ros2 run tf2_tools view_frames

# 查看生成的PDF（Linux）
evince frames.pdf

# 输出示例：
# Listening to tf data during 5 seconds...
# Generating graph in frames.pdf file...
```

### 4.3 监控TF延迟：tf2_monitor

```bash
# 监控所有变换的更新频率
ros2 run tf2_ros tf2_monitor

# 监控特定变换
ros2 run tf2_ros tf2_monitor base_link laser_link

# 输出示例：
# RESULTS: for all Frames
# Frames:
# Frame: base_link, published by <no authority>, Average Delay: 0.001s, Max Delay: 0.003s
# Frame: laser_link, published by <no authority>, Average Delay: 0.000s, Max Delay: 0.001s
```

### 4.4 发布静态变换：static_transform_publisher

通过命令行快速发布静态变换（无需写代码）：

```bash
# 方法1：使用欧拉角（x, y, z, roll, pitch, yaw, parent, child）
ros2 run tf2_ros static_transform_publisher 0.3 0 0.2 0 0 0 base_link laser_link

# 方法2：使用四元数（x, y, z, qx, qy, qz, qw, parent, child）
ros2 run tf2_ros static_transform_publisher 0.3 0 0.2 0 0 0 1 base_link laser_link
```

**使用场景：**
- 快速测试TF功能
- 发布传感器URDF中没有的静态变换
- 调试时临时添加坐标系

### 4.5 查看TF话题

```bash
# 查看 /tf 话题的消息
ros2 topic echo /tf

# 查看 /tf_static 话题的消息
ros2 topic echo /tf_static

# 查看 /tf 话题的发布频率
ros2 topic hz /tf

# 查看 /tf 话题的信息
ros2 topic info /tf
```

### 4.6 查看坐标系列表

```bash
# 列出所有已知的坐标系
ros2 run tf2_ros tf2_echo --list

# 或通过话题查看
ros2 topic echo /tf --field transforms[].child_frame_id
```

---

## 5. C++ 代码实现

### 5.1 功能包结构

```
tf2_learning/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── static_tf_broadcaster.cpp     # 静态坐标广播器
│   ├── dynamic_tf_broadcaster.cpp    # 动态坐标广播器
│   ├── tf_listener.cpp               # 坐标监听器
│   ├── tf_point_transform.cpp        # 坐标点变换
│   ├── robot_tf_broadcaster.cpp      # 机器人TF树
│   └── tf_time_travel.cpp            # TF时间旅行
├── launch/
│   ├── tf2_demo.launch.py            # 基本演示
│   └── robot_demo.launch.py          # 机器人演示
└── rviz/
    └── robot_tf.rviz                 # RViz配置
```

### 5.2 静态坐标广播器

**文件：** `src/static_tf_broadcaster.cpp`

**核心步骤：**
1. 创建 `StaticTransformBroadcaster`
2. 构造 `TransformStamped` 消息
3. 调用 `sendTransform()` 发布（只需一次）

**关键代码：**
```cpp
// 创建静态广播器
auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

// 构造变换消息
geometry_msgs::msg::TransformStamped t;
t.header.frame_id = "base_link";       // 父坐标系
t.child_frame_id = "sensor_lidar";     // 子坐标系
t.transform.translation.x = 0.3;      // 平移
t.transform.translation.y = 0.0;
t.transform.translation.z = 0.2;
tf2::Quaternion q;
q.setRPY(0, 0, 0);                     // 欧拉角 → 四元数
t.transform.rotation = tf2::toMsg(q);  // 旋转

// 发布
broadcaster->sendTransform(t);
```

**运行与验证：**
```bash
# 编译
colcon build --packages-select tf2_learning

# 运行
ros2 run tf2_learning static_tf_broadcaster

# 验证
ros2 run tf2_ros tf2_echo base_link sensor_lidar
```

### 5.3 动态坐标广播器

**文件：** `src/dynamic_tf_broadcaster.cpp`

**核心步骤：**
1. 创建 `TransformBroadcaster`（注意不是Static）
2. 创建定时器，周期性发布
3. 每次更新变换数据并发布

**与静态广播器的区别：**
| 特性 | 静态广播器 | 动态广播器 |
|------|-----------|-----------|
| 发布频率 | 一次即可 | 需要持续发布 |
| 话题 | /tf_static | /tf |
| 缓存方式 | 永久缓存 | 按时间缓存 |
| 典型频率 | N/A | 10-100Hz |

**运行：**
```bash
ros2 run tf2_learning dynamic_tf_broadcaster
```

### 5.4 坐标监听器

**文件：** `src/tf_listener.cpp`

**核心步骤：**
1. 创建 `Buffer` 和 `TransformListener`
2. 在定时器回调中调用 `lookupTransform()`
3. 处理异常

**关键API：**
```cpp
// 查询最新变换
auto transform = tf_buffer->lookupTransform(
    "target_frame",     // 目标坐标系
    "source_frame",     // 源坐标系
    tf2::TimePointZero  // 最新时间
);

// 查询特定时间的变换
auto transform = tf_buffer->lookupTransform(
    "target_frame",
    "source_frame",
    time_point          // 特定时间
);

// 检查变换是否可用
bool available = tf_buffer->canTransform(
    "target_frame", "source_frame", tf2::TimePointZero);
```

### 5.5 坐标点变换

**文件：** `src/tf_point_transform.cpp`

**两种方法：**

**方法1（推荐）：使用 `doTransform`**
```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 必须包含

geometry_msgs::msg::PointStamped source_point;  // 源点
geometry_msgs::msg::PointStamped target_point;  // 目标点

auto transform = tf_buffer->lookupTransform(
    "target_frame", "source_frame", source_point.header.stamp);

tf2::doTransform(source_point, target_point, transform);
```

**方法2：手动矩阵运算**
```cpp
// 适用于批量变换（复用变换矩阵）
auto transform = tf_buffer->lookupTransform("target", "source", time);
// 从四元数构造旋转矩阵，然后 p_target = R * p_source + t
```

### 5.6 机器人TF树

**文件：** `src/robot_tf_broadcaster.cpp`

模拟一个完整的移动机器人TF树，包含：
- `map → odom`（定位节点发布）
- `odom → base_link`（里程计节点发布）
- `base_link → base_footprint`（静态）
- `base_link → imu_link`（静态）
- `base_link → laser_link`（静态）
- `base_link → camera_link`（静态）

**运行：**
```bash
ros2 run tf2_learning robot_tf_broadcaster

# 另一终端查看TF树
ros2 run tf2_tools view_frames

# 使用RViz可视化
ros2 launch tf2_learning robot_demo.launch.py
```

### 5.7 TF时间旅行

**文件：** `src/tf_time_travel.cpp`

演示TF2的时间查询能力：
- 查询最新变换
- 查询历史变换（2秒前）
- 高级API（不同时间点的坐标系变换）

**高级API详解：**
```cpp
// 将 source_frame 在 source_time 时刻的位置
// 变换到 target_frame 在 target_time 时刻的位置
// 参考坐标系：fixed_frame
auto transform = tf_buffer->lookupTransform(
    "target_frame",    target_time,
    "source_frame",    source_time,
    "fixed_frame",     timeout
);
```

---

## 6. Python 代码实现

### 6.1 功能包结构

```
tf2_learning_py/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── tf2_learning_py
├── tf2_learning_py/
│   ├── __init__.py
│   ├── static_tf_broadcaster.py     # 静态坐标广播器
│   ├── dynamic_tf_broadcaster.py    # 动态坐标广播器
│   ├── tf_listener.py               # 坐标监听器
│   ├── tf_point_transform.py        # 坐标点变换
│   ├── robot_tf_broadcaster.py      # 机器人TF树
│   └── tf_time_travel.py            # TF时间旅行
├── launch/
│   └── tf2_demo_py.launch.py
└── rviz/
    └── robot_tf_py.rviz
```

### 6.2 关键API对照（C++ vs Python）

| 功能 | C++ | Python |
|------|-----|--------|
| 静态广播器 | `tf2_ros::StaticTransformBroadcaster` | `tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster` |
| 动态广播器 | `tf2_ros::TransformBroadcaster` | `tf2_ros.transform_broadcaster.TransformBroadcaster` |
| 缓冲区 | `tf2_ros::Buffer` | `tf2_ros.buffer.Buffer` |
| 监听器 | `tf2_ros::TransformListener` | `tf2_ros.transform_listener.TransformListener` |
| 查询变换 | `tf_buffer->lookupTransform(...)` | `tf_buffer.lookup_transform(...)` |
| 点变换 | `tf2::doTransform(point, result, transform)` | `tf2_geometry_msgs.do_transform_point(point, transform)` |
| 欧拉角→四元数 | `q.setRPY(r, p, y)` | 手动计算或使用 `transforms3d` |
| 获取最新时间 | `tf2::TimePointZero` | `rclpy.time.Time()` |

### 6.3 Python代码要点

**发布变换：**
```python
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

broadcaster = TransformBroadcaster(node)
t = TransformStamped()
t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = 'parent'
t.child_frame_id = 'child'
t.transform.translation.x = 1.0
t.transform.rotation.w = 1.0
broadcaster.sendTransform(t)
```

**查询变换：**
```python
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

transform = tf_buffer.lookup_transform('target', 'source', rclpy.time.Time())
```

**点变换：**
```python
import tf2_geometry_msgs  # 必须导入，注册do_transform_point
from geometry_msgs.msg import PointStamped

point = PointStamped()
point.header.frame_id = 'source'
point.point.x = 1.0

transform = tf_buffer.lookup_transform('target', 'source', rclpy.time.Time())
result = tf2_geometry_msgs.do_transform_point(point, transform)
```

### 6.4 运行方式

```bash
# 编译
colcon build --packages-select tf2_learning_py
source install/setup.bash

# 运行各个示例
ros2 run tf2_learning_py static_tf_broadcaster_py
ros2 run tf2_learning_py dynamic_tf_broadcaster_py
ros2 run tf2_learning_py tf_listener_py
ros2 run tf2_learning_py tf_point_transform_py
ros2 run tf2_learning_py robot_tf_broadcaster_py
ros2 run tf2_learning_py tf_time_travel_py

# 使用launch文件
ros2 launch tf2_learning_py tf2_demo_py.launch.py
```

---

## 7. 机器人实际应用案例

### 7.1 案例1：激光雷达避障

**场景：** 激光雷达检测到前方障碍物，需要在机器人底盘坐标系中定位

```cpp
// 在激光雷达坐标系中检测到障碍物
geometry_msgs::msg::PointStamped obstacle_in_laser;
obstacle_in_laser.header.frame_id = "laser_link";
obstacle_in_laser.header.stamp = this->now();
obstacle_in_laser.point.x = 2.0;  // 雷达前方2米
obstacle_in_laser.point.y = 0.5;  // 右偏0.5米

// 变换到机器人底盘坐标系
geometry_msgs::msg::PointStamped obstacle_in_base;
tf2::doTransform(obstacle_in_laser, obstacle_in_base,
    tf_buffer_->lookupTransform("base_link", "laser_link",
        obstacle_in_laser.header.stamp));

// 现在可以基于base_link坐标做出避障决策
RCLCPP_INFO(this->get_logger(),
    "障碍物在机器人前方%.2f米，右方%.2f米",
    obstacle_in_base.point.x, obstacle_in_base.point.y);
```

### 7.2 案例2：自主导航中的坐标变换链

**场景：** 导航系统需要将全局路径点变换到机器人当前坐标系

```
变换链：map → odom → base_link

map:      全局坐标系（与建图一致）
odom:     里程计坐标系（短期一致，会漂移）
base_link: 机器人底盘坐标系
```

```cpp
// 目标点在地图坐标系中
geometry_msgs::msg::PoseStamped goal_in_map;
goal_in_map.header.frame_id = "map";
goal_in_map.pose.position.x = 10.0;
goal_in_map.pose.position.y = 5.0;

// 变换到机器人坐标系
geometry_msgs::msg::PoseStamped goal_in_base;
tf2::doTransform(goal_in_map, goal_in_base,
    tf_buffer_->lookupTransform("base_link", "map", this->now()));

// 计算机器人需要移动的距离和方向
double dx = goal_in_base.pose.position.x;
double dy = goal_in_base.pose.position.y;
double distance = std::sqrt(dx*dx + dy*dy);
double angle = std::atan2(dy, dx);
```

### 7.3 案例3：多传感器融合

**场景：** 激光雷达和摄像头检测到同一目标，需要融合数据

```cpp
// 激光雷达检测到的目标位置
geometry_msgs::msg::PointStamped target_in_laser;
target_in_laser.header.frame_id = "laser_link";
target_in_laser.header.stamp = this->now();
target_in_laser.point.x = 3.0;
target_in_laser.point.y = 0.0;

// 摄像头检测到的目标位置
geometry_msgs::msg::PointStamped target_in_camera;
target_in_camera.header.frame_id = "camera_link";
target_in_camera.header.stamp = this->now();
target_in_camera.point.x = 2.5;
target_in_camera.point.y = 0.1;

// 统一变换到 base_link 坐标系
geometry_msgs::msg::PointStamped target_from_laser, target_from_camera;
tf2::doTransform(target_in_laser, target_from_laser,
    tf_buffer_->lookupTransform("base_link", "laser_link", this->now()));
tf2::doTransform(target_in_camera, target_from_camera,
    tf_buffer_->lookupTransform("base_link", "camera_link", this->now()));

// 现在可以在同一坐标系下进行数据融合
// 例如：卡尔曼滤波、加权平均等
```

### 7.4 案例4：机械臂抓取

**场景：** 摄像头检测到目标物体位置，需要变换到机械臂基座坐标系

```
TF树：
base_link → shoulder_link → upper_arm → forearm → wrist → gripper
         → camera_link
```

```cpp
// 摄像头检测到物体
geometry_msgs::msg::PoseStamped object_in_camera;
object_in_camera.header.frame_id = "camera_link";
object_in_camera.header.stamp = this->now();
object_in_camera.pose.position.x = 0.3;
object_in_camera.pose.position.y = -0.1;
object_in_camera.pose.position.z = 0.5;

// 变换到机械臂基座坐标系
geometry_msgs::msg::PoseStamped object_in_arm_base;
tf2::doTransform(object_in_camera, object_in_arm_base,
    tf_buffer_->lookupTransform("shoulder_link", "camera_link", this->now()));

// 使用变换后的位置计算逆运动学
// IK求解器需要目标在机械臂基座坐标系中的位置
```

### 7.5 案例5：SLAM建图

**场景：** 使用激光雷达SLAM建图时，TF树的角色

```
SLAM节点发布的变换：
  map → odom（修正里程计漂移）

里程计节点发布的变换：
  odom → base_link（轮式里程计）

robot_state_publisher发布的变换：
  base_link → laser_link（静态，来自URDF）
  base_link → imu_link（静态，来自URDF）

完整TF树：
  map → odom → base_link → laser_link
                       → imu_link
                       → camera_link
```

### 7.6 案例6：无人机位姿估计

**场景：** 无人机使用IMU和GPS融合定位

```cpp
// IMU提供的姿态（在imu_link坐标系下）
sensor_msgs::msg::Imu imu_data;
// ... 接收IMU数据

// 将IMU姿态变换到base_link
geometry_msgs::msg::QuaternionStamped imu_quat;
imu_quat.header = imu_data.header;
imu_quat.quaternion = imu_data.orientation;

geometry_msgs::msg::QuaternionStamped base_quat;
tf2::doTransform(imu_quat, base_quat,
    tf_buffer_->lookupTransform("base_link", "imu_link",
        imu_data.header.stamp));

// 结合GPS位置和IMU姿态，发布 map → base_link 变换
geometry_msgs::msg::TransformStamped tf;
tf.header.stamp = this->now();
tf.header.frame_id = "map";
tf.child_frame_id = "base_link";
tf.transform.translation.x = gps_latitude;   // GPS位置
tf.transform.translation.y = gps_longitude;
tf.transform.translation.z = gps_altitude;
tf.transform.rotation = base_quat.quaternion; // IMU姿态
tf_broadcaster_->sendTransform(tf);
```

---

## 8. REP-105 坐标系约定

**REP-105** 是ROS中关于移动机器人坐标系的标准化约定，定义了以下核心坐标系：

### 8.1 核心坐标系

| 坐标系 | 全称 | 说明 | 发布者 |
|--------|------|------|--------|
| `map` | 全局坐标系 | 固定的世界坐标系，Z轴向上 | SLAM/定位节点 |
| `odom` | 里程计坐标系 | 局部一致，会漂移 | 里程计节点 |
| `base_link` | 机器人底盘 | 机器人中心，Z轴向上 | robot_state_publisher |
| `base_footprint` | 地面投影 | base_link在Z=0的投影 | robot_state_publisher |

### 8.2 变换链

```
map ──▶ odom ──▶ base_link ──▶ sensors/actuators
 │       │          │
 │       │          └── 固定变换（URDF定义）
 │       └────────── 里程计发布（短期一致，会漂移）
 └────────────────── 定位修正（SLAM/AMCL，修正漂移）
```

### 8.3 命名规范

- 传感器：`<sensor_type>_link`（如 `laser_link`, `camera_link`）
- 执行器：`<joint_name>_link`（如 `wheel_left_link`）
- 坐标轴：X前、Y左、Z上（右手坐标系）

---

## 9. TF2 高级话题

### 9.1 URDF 与 robot_state_publisher

**最佳实践：** 使用URDF定义机器人的TF树，而不是手动发布

```xml
<!-- robot.urdf -->
<robot name="my_robot">
  <!-- 底盘 -->
  <link name="base_link"/>
  
  <!-- 激光雷达 -->
  <link name="laser_link"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  
  <!-- 摄像头 -->
  <link name="camera_link"/>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.12" rpy="0 -0.349 0"/>
  </joint>
</robot>
```

```python
# launch文件中使用 robot_state_publisher
from launch_ros.actions import Node
import os

urdf = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': open(urdf).read()}]
)
```

**优势：**
- TF变换与物理模型统一管理
- 自动发布所有静态变换
- 支持关节状态驱动的动态变换

### 9.2 TF2 与时间同步

多传感器融合时，需要处理传感器时间戳不一致的问题：

```cpp
// 方法1：使用传感器的时间戳查询变换
auto transform = tf_buffer_->lookupTransform(
    "base_link",
    sensor_msg->header.frame_id,
    sensor_msg->header.stamp,           // 使用传感器时间
    std::chrono::milliseconds(100)      // 超时等待
);

// 方法2：使用近似时间
auto transform = tf_buffer_->lookupTransform(
    "base_link",
    "sensor_link",
    this->now() - rclcpp::Duration(0, 50e6),  // 50ms延迟补偿
    std::chrono::milliseconds(100)
);
```

### 9.3 TF2 性能优化

**1. 减少不必要的TF发布**
- 固定变换使用 `StaticTransformBroadcaster`
- 降低动态TF的发布频率（够用即可）

**2. 合理设置缓存时间**
```cpp
// 默认10秒，根据需求调整
auto buffer = std::make_shared<tf2_ros::Buffer>(
    this->get_clock(),
    std::chrono::seconds(30)  // 增大缓存
);
```

**3. 避免重复查询**
```cpp
// 不好：每次都查询
for (const auto& point : points) {
    auto transform = tf_buffer_->lookupTransform(...);  // 重复查询
    tf2::doTransform(point, result, transform);
}

// 好：查询一次，复用变换
auto transform = tf_buffer_->lookupTransform(...);      // 只查一次
for (const auto& point : points) {
    tf2::doTransform(point, result, transform);          // 复用
}
```

**4. 使用 canTransform 预检查**
```cpp
if (tf_buffer_->canTransform("target", "source", time)) {
    auto transform = tf_buffer_->lookupTransform("target", "source", time);
    // ... 处理变换
} else {
    // 变换不可用，跳过或使用默认值
}
```

### 9.4 TF2 支持的数据类型

| 数据类型 | 头文件/模块 | 说明 |
|---------|------------|------|
| `PointStamped` | `tf2_geometry_msgs` | 3D点 |
| `PoseStamped` | `tf2_geometry_msgs` | 3D位姿（位置+姿态） |
| `Vector3Stamped` | `tf2_geometry_msgs` | 3D向量 |
| `TransformStamped` | `tf2_geometry_msgs` | 变换 |
| `PolygonStamped` | `tf2_geometry_msgs` | 多边形 |
| `WrenchStamped` | `tf2_geometry_msgs` | 力/力矩 |
| `TwistStamped` | `tf2_geometry_msgs` | 速度 |
| `PointCloud2` | `tf2_sensor_msgs` | 点云 |

### 9.5 自定义数据类型的变换

可以为自定义数据类型实现 `doTransform` 特化：

```cpp
// 自定义消息类型 MyMsg
#include <tf2/tf2.h>

namespace tf2 {
    template<>
    inline my_package::msg::MyMsg doTransform(
        const my_package::msg::MyMsg& data_in,
        const geometry_msgs::msg::TransformStamped& transform)
    {
        my_package::msg::MyMsg data_out;
        // 实现变换逻辑
        return data_out;
    }
}
```

---

## 10. 常见问题与调试

### 10.1 常见异常

| 异常类型 | 原因 | 解决方法 |
|---------|------|---------|
| `LookupException` | 坐标系不存在 | 检查frame_id拼写，确认广播器已启动 |
| `ConnectivityException` | TF树断裂，无法连接 | 检查是否所有中间变换都已发布 |
| `ExtrapolationException` | 请求的时间超出缓存 | 增大缓存时间，或使用最新时间 |
| `InvalidArgumentException` | frame_id为空 | 检查frame_id是否正确设置 |

### 10.2 调试步骤

**1. 检查坐标系是否存在**
```bash
ros2 run tf2_ros tf2_echo base_link laser_link
```
如果报错 `Frame [laser_link] does not exist`，说明该坐标系未被发布。

**2. 检查TF树完整性**
```bash
ros2 run tf2_tools view_frames
# 打开生成的 frames.pdf 查看
```

**3. 检查TF发布频率**
```bash
ros2 topic hz /tf
# 正常应该在 10-100Hz 之间
```

**4. 检查变换延迟**
```bash
ros2 run tf2_ros tf2_monitor
# Average Delay 应该接近0
```

**5. 检查消息内容**
```bash
ros2 topic echo /tf --field transforms
# 检查 frame_id, child_frame_id, 时间戳等
```

### 10.3 常见错误模式

**错误1：TF树断裂**
```
map → odom          (由SLAM发布)
base_link → laser   (由URDF发布)

缺少: odom → base_link (应由里程计发布)
```

**错误2：时间戳不正确**
```cpp
// 错误：时间戳为0
t.header.stamp = builtin_interfaces::msg::Time();

// 正确：使用当前时间
t.header.stamp = this->now();
```

**错误3：混淆父/子坐标系**
```cpp
// 错误：反了父子关系
t.header.frame_id = "laser_link";    // 应该是父
t.child_frame_id = "base_link";      // 应该是子

// 正确：
t.header.frame_id = "base_link";     // 父坐标系
t.child_frame_id = "laser_link";     // 子坐标系
```

**错误4：静态变换反复发布**
```cpp
// 错误：使用动态广播器发布静态变换
auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
timer = node->create_wall_timer(100ms, [&]() {
    broadcaster->sendTransform(static_transform);  // 浪费带宽
});

// 正确：使用静态广播器
auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
broadcaster->sendTransform(static_transform);  // 只需一次
```

### 10.4 使用 ros2 doctor 检查

```bash
# 检查ROS2系统状态
ros2 doctor

# 检查TF相关的问题
ros2 doctor --report | grep -i transform
```

---

## 11. 快速参考卡片

### 11.1 命令行速查

```bash
# 查看变换
ros2 run tf2_ros tf2_echo <parent> <child>

# 查看TF树
ros2 run tf2_tools view_frames

# 监控TF延迟
ros2 run tf2_ros tf2_monitor

# 发布静态变换（欧拉角）
ros2 run tf2_ros static_transform_publisher x y z roll pitch yaw parent child

# 发布静态变换（四元数）
ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw parent child

# 查看TF话题
ros2 topic echo /tf
ros2 topic hz /tf
```

### 11.2 C++ 速查

```cpp
// === 包含头文件 ===
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

// === 发布动态变换 ===
auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
geometry_msgs::msg::TransformStamped t;
t.header.stamp = node->now();
t.header.frame_id = "parent";
t.child_frame_id = "child";
t.transform.translation.x = 1.0;
tf2::Quaternion q; q.setRPY(0,0,yaw);
t.transform.rotation.x = q.x();  // 手动赋值四元数字段
t.transform.rotation.y = q.y();
t.transform.rotation.z = q.z();
t.transform.rotation.w = q.w();
broadcaster->sendTransform(t);

// === 发布静态变换 ===
auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
static_broadcaster->sendTransform(t);  // 只需一次

// === 查询变换 ===
auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
auto transform = buffer->lookupTransform("target", "source", tf2::TimePointZero);

// === 点变换 ===
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
geometry_msgs::msg::PointStamped src, dst;
tf2::doTransform(src, dst, transform);
```

### 11.3 Python 速查

```python
# === 导入 ===
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_geometry_msgs  # 注册 do_transform_point

# === 发布变换 ===
broadcaster = TransformBroadcaster(node)
t = TransformStamped()
t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = 'parent'
t.child_frame_id = 'child'
t.transform.translation.x = 1.0
t.transform.rotation.w = 1.0
broadcaster.sendTransform(t)

# === 查询变换 ===
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
transform = tf_buffer.lookup_transform('target', 'source', rclpy.time.Time())

# === 点变换 ===
point = PointStamped()
point.header.frame_id = 'source'
point.point.x = 1.0
result = tf2_geometry_msgs.do_transform_point(point, transform)
```

---

## 附录：编译与运行

### 编译

```bash
cd z:/learning/ROS2-Learning-main/code

# 编译C++包
colcon build --packages-select tf2_learning

# 编译Python包
colcon build --packages-select tf2_learning_py

# 同时编译两个包
colcon build --packages-select tf2_learning tf2_learning_py

# 刷新环境
source install/setup.bash  # Linux
# 或
install/setup.ps1          # Windows PowerShell
```

### 运行示例

```bash
# === C++ 示例 ===
ros2 run tf2_learning static_tf_broadcaster
ros2 run tf2_learning dynamic_tf_broadcaster
ros2 run tf2_learning tf_listener
ros2 run tf2_learning tf_point_transform
ros2 run tf2_learning robot_tf_broadcaster
ros2 run tf2_learning tf_time_travel

# === Python 示例 ===
ros2 run tf2_learning_py static_tf_broadcaster_py
ros2 run tf2_learning_py dynamic_tf_broadcaster_py
ros2 run tf2_learning_py tf_listener_py
ros2 run tf2_learning_py tf_point_transform_py
ros2 run tf2_learning_py robot_tf_broadcaster_py
ros2 run tf2_learning_py tf_time_travel_py

# === Launch 方式 ===
ros2 launch tf2_learning tf2_demo.launch.py
ros2 launch tf2_learning robot_demo.launch.py
ros2 launch tf2_learning_py tf2_demo_py.launch.py
```
