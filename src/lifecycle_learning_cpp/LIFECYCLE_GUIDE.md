# ROS2 生命周期节点（Lifecycle Node）完全指南 —— C++ 版

> 本文档是 `lifecycle_learning_cpp` 功能包的配套说明，涵盖 ROS2 生命周期节点的所有核心概念、状态机、转换回调、Launch 编排和机器人实战案例。

---

## 目录

1. [生命周期节点概述](#1-生命周期节点概述)
2. [状态机详解](#2-状态机详解)
3. [转换回调](#3-转换回调)
4. [LifecyclePublisher](#4-lifecyclepublisher)
5. [生命周期订阅策略](#5-生命周期订阅策略)
6. [外部状态管理](#6-外部状态管理)
7. [Launch 编排](#7-launch-编排)
8. [生命周期节点与回调组](#8-生命周期节点与回调组)
9. [机器人实战案例](#9-机器人实战案例)
10. [高级话题](#10-高级话题)
11. [常见问题与调试](#11-常见问题与调试)
12. [快速参考](#12-快速参考)

---

## 1. 生命周期节点概述

### 1.1 什么是生命周期节点？

生命周期节点（LifecycleNode / ManagedNode）是 ROS2 引入的**有状态节点**，它将节点的生命周期划分为明确的状态，每个状态之间有严格的转换规则。

```
普通节点：
  创建 → 运行 → 销毁    （无状态管理，无法安全启停）

生命周期节点：
  Unconfigured → Inactive → Active → Inactive → Unconfigured → Finalized
  （每一步都有明确的语义和回调，支持安全启停）
```

### 1.2 为什么需要生命周期节点？

| 场景 | 普通节点 | 生命周期节点 |
|------|---------|-------------|
| 传感器驱动 | 启动即开始发布，无法暂停 | configure 连接硬件，activate 开始发布 |
| 导航系统 | 启动顺序不可控 | 按依赖顺序启动 |
| 安全关键系统 | 崩溃后无法安全恢复 | deactivate → 修复 → activate |
| 多机器人系统 | 无法统一管理启停 | 生命周期管理器统一控制 |
| 调试 | 只能 kill + restart | deactivate 暂停，activate 恢复 |
| 硬件资源 | 占用后无法释放 | cleanup 释放资源 |

### 1.3 ROS1 vs ROS2

| 方面 | ROS1 | ROS2 |
|------|------|------|
| 节点生命周期 | 无 | LifecycleNode |
| 启动控制 | 无法控制 | 状态机精确控制 |
| 安全停机 | kill 信号 | deactivate → cleanup → shutdown |
| 资源管理 | 无法释放 | cleanup 释放所有资源 |
| 错误恢复 | 进程重启 | on_error 回调恢复 |
| 统一管理 | 无 | lifecycle_manager |

---

## 2. 状态机详解

### 2.1 四个主状态

```
[1] Unconfigured — 刚创建，未分配任何资源
[2] Inactive     — 已配置（资源已分配），但未激活
[3] Active       — 正常工作
[4] Finalized    — 已关闭，不可恢复
```

### 2.2 六个过渡状态

| 过渡状态 | ID | 含义 |
|---------|-----|------|
| Configuring | 10 | 正在执行 on_configure |
| CleaningUp | 11 | 正在执行 on_cleanup |
| ShuttingDown | 12 | 正在执行 on_shutdown |
| Activating | 13 | 正在执行 on_activate |
| Deactivating | 14 | 正在执行 on_deactivate |
| ErrorProcessing | 15 | 正在执行 on_error |

### 2.3 合法转换路径

```
                        configure
    Unconfigured ──────────────────► Inactive
         │                              │  ▲
         │                    activate  │  │ deactivate
         │                              ▼  │
         │                          Active
         │
         │  cleanup (Inactive→Unconfigured)
         │  shutdown (任何→Finalized)
         └────────────────────────────────► Finalized
```

**关键规则**：

- ❌ 不能跳状态（如 Unconfigured → Active）
- ❌ 不能反向转换（如 Active → Unconfigured，必须先 deactivate）
- ✅ 从任何主状态都可以 shutdown

---

## 3. 转换回调

### 3.1 六个回调函数

| 回调 | 触发转换 | 典型操作 |
|------|---------|---------|
| `on_configure` | Unconfigured→Inactive | 分配资源、初始化硬件、读取配置 |
| `on_activate` | Inactive→Active | 启动数据流、激活发布器、使能硬件 |
| `on_deactivate` | Active→Inactive | 暂停数据流、停用发布器、禁用输出 |
| `on_cleanup` | Inactive→Unconfigured | 释放资源、断开硬件、重置状态 |
| `on_shutdown` | 任何→Finalized | 安全关闭、释放所有资源 |
| `on_error` | 错误恢复 | 复位硬件、尝试恢复 |

### 3.2 CallbackReturn 返回值

| 返回值 | 含义 | 后续行为 |
|--------|------|---------|
| `SUCCESS` | 转换成功 | 进入目标状态 |
| `FAILURE` | 转换失败 | 回到之前的状态 |
| `ERROR` | 严重错误 | 进入 ErrorProcessing → 调用 on_error |

### 3.3 错误恢复流程

```
on_configure/on_activate/... 返回 ERROR
         │
         ▼
    ErrorProcessing 状态
         │
         ▼
    on_error() 回调
      ┌────┴────┐
   SUCCESS   FAILURE
      │         │
      ▼         ▼
  Unconfigured  Finalized
  (恢复成功)    (节点死亡)
```

### 3.4 ⚠️ on_activate/on_deactivate 必须调用父类

```cpp
CallbackReturn on_activate(const State & prev) override {
    // 自定义逻辑...
    rclcpp_lifecycle::LifecycleNode::on_activate(prev);  // 必须！激活所有 LifecyclePublisher
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_deactivate(const State & prev) override {
    rclcpp_lifecycle::LifecycleNode::on_deactivate(prev);  // 必须！停用所有 LifecyclePublisher
    // 自定义逻辑...
    return CallbackReturn::SUCCESS;
}
```

---

## 4. LifecyclePublisher

### 4.1 与普通 Publisher 的区别

| 方面 | `rclcpp::Publisher` | `LifecyclePublisher` |
|------|--------------------|---------------------------------------|
| Inactive 下 publish | 正常发布 | **静默丢弃** |
| is_activated() | 不存在 | true(Active) / false(Inactive) |
| 自动管理 | 无 | on_activate/on_deactivate 自动管理 |

### 4.2 状态行为

```
节点状态          is_activated()   publish() 行为
──────────────   ──────────────   ──────────────
Unconfigured     false            不发布（消息丢弃）
Inactive         false            不发布（消息丢弃）
Active           true             正常发布
```

---

## 5. 生命周期订阅策略

### 5.1 ⚠️ 关键区别：订阅不会自动停用

```
LifecyclePublisher:  Inactive 状态下自动不发布 ✓
普通订阅:            Inactive 状态下回调仍执行 ✗
```

### 5.2 推荐设计模式

```cpp
// 模式一：在回调中检查状态（推荐）
sub_ = create_subscription<Msg>("/topic", 10,
    [this](const Msg::SharedPtr msg) {
        if (get_current_state().id() == PRIMARY_STATE_ACTIVE) {
            process_data(msg);  // Active 才处理
        }
    });

// 模式二：手动销毁/重建订阅
// on_activate:  sub_ = create_subscription(...)
// on_deactivate: sub_.reset()
```

---

## 6. 外部状态管理

### 6.1 生命周期节点自动暴露的服务

| 服务 | 类型 | 用途 |
|------|------|------|
| `/<node>/change_state` | `ChangeState` | 触发状态转换 |
| `/<node>/get_state` | `GetState` | 查询当前状态 |
| `/<node>/get_available_transitions` | `GetAvailableTransitions` | 查询可用转换 |

### 6.2 命令行操作

```bash
ros2 lifecycle get /my_node                        # 查询状态
ros2 lifecycle list /my_node                       # 查询可用转换
ros2 lifecycle set /my_node configure              # 触发转换
ros2 lifecycle set /my_node activate
ros2 lifecycle set /my_node deactivate
ros2 lifecycle set /my_node cleanup
ros2 lifecycle set /my_node shutdown
```

### 6.3 Nav2 Lifecycle Manager 模式

```python
# Nav2 使用专门的 lifecycle_manager 统一管理
lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    parameters=[{
        'node_names': ['controller_server', 'planner_server'],
        'autostart': True,
    }],
)
```

---

## 7. Launch 编排

### 7.1 LifecycleNode Action

```python
from launch_ros.actions import LifecycleNode

lifecycle_node = LifecycleNode(
    package='my_package',
    executable='my_node',
    name='my_lifecycle_node',
    parameters=[{'param1': 'value1'}],
)
```

### 7.2 Launch 中自动转换状态

```python
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# 进程启动后自动 configure + activate
configure_event = ChangeState(
    lifecycle_node_matcher=matches_action(lifecycle_node),
    transition_id=Transition.TRANSITION_CONFIGURE,
)
```

---

## 8. 生命周期节点与回调组

### 8.1 LifecycleNode 中的回调组

```cpp
class MyNode : public rclcpp_lifecycle::LifecycleNode {
    MyNode() : LifecycleNode("my_node") {
        sensor_group_ = create_callback_group(CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions opts;
        opts.callback_group = sensor_group_;
        sub_ = create_subscription<Msg>("/topic", 10, cb, opts);
    }
};
```

### 8.2 生命周期服务使用的回调组

生命周期转换服务（change_state/get_state）使用**默认回调组**。

在 MultiThreadedExecutor 下：
- 生命周期转换与传感器/处理回调可以并发
- 转换服务之间是串行的（同一默认组）

---

## 9. 机器人实战案例

### 9.1 传感器驱动（示例⑧）

```
on_configure  → 打开设备、设置波特率、验证硬件
on_activate   → 启动采集、开始发布传感器数据
on_deactivate → 暂停采集（硬件保持连接！可快速恢复）
on_cleanup    → 断开硬件、释放设备资源
on_error      → 复位硬件、尝试恢复连接
```

### 9.2 导航栈（示例⑨）

```
on_configure  → 验证参数、创建定时器（不启动）
on_activate   → 启动控制循环、激活速度发布器
on_deactivate → ⚠️ 发送零速度！停止控制循环
on_cleanup    → 释放定时器和发布器

关键安全措施：
  - deactivate 时必须发送零速度
  - 安全监控始终运行
  - 急停功能独立于生命周期状态
```

### 9.3 多子系统协调（示例⑩）

```
启动顺序（按依赖关系）：
  传感器驱动 → 定位节点 → 路径规划 → 速度控制

关闭顺序（逆序）：
  速度控制 → 路径规划 → 定位节点 → 传感器驱动
```

### 9.4 机械臂控制器

```
on_configure  → 加载URDF、初始化运动学、连接驱动器
on_activate   → 使能电机、启动轨迹执行
on_deactivate → 减速停止、禁用电机（保持位置！）
on_error      → 紧急停止、报告故障码
```

### 9.5 无人机飞控

```
on_configure  → 校准IMU、初始化姿态估计、检查GPS
on_activate   → 解锁电机、启动姿态控制
on_deactivate → 降落、锁定电机
on_error      → 紧急降落
```

---

## 10. 高级话题

### 10.1 C++ vs Python 生命周期 API 对照

| 操作 | Python (rclpy) | C++ (rclcpp) |
|------|---------------|-------------|
| 基类 | `LifecycleNode` | `rclcpp_lifecycle::LifecycleNode` |
| LifecyclePublisher | 自动返回 | `LifecyclePublisher<MsgT>::SharedPtr` |
| is_activated | `pub.activated` | `pub->is_activated()` |
| 返回值 | `TransitionCallbackReturn.SUCCESS` | `CallbackReturn::SUCCESS` |
| 父类调用 | `super().on_activate(state)` | `LifecycleNode::on_activate(prev)` |

### 10.2 生命周期参数验证

```cpp
this->register_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & param : params) {
            if (param.get_name() == "max_speed") {
                if (param.as_double() < 0.0) {
                    return rcl_interfaces::msg::SetParametersResult()
                        .set__successful(false);
                }
            }
        }
        return rcl_interfaces::msg::SetParametersResult()
            .set__successful(true);
    });
```

### 10.3 生命周期事件监控

```cpp
auto event_sub = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    "/my_node/transition_event", 10,
    [](const TransitionEvent::SharedPtr event) {
        RCLCPP_INFO(logger, "状态转换: %s → %s",
            event->start_state.label.c_str(),
            event->goal_state.label.c_str());
    });
```

### 10.4 Composition 与生命周期节点

```cpp
class MyComponent : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MyComponent(const rclcpp::NodeOptions & options)
        : rclcpp_lifecycle::LifecycleNode("my_component", options) {}
};
RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

---

## 11. 常见问题与调试

### 11.1 LifecyclePublisher publish 不发出消息

**原因**：节点不在 Active 状态

**解决**：确保 on_activate 中调用了父类方法
```cpp
rclcpp_lifecycle::LifecycleNode::on_activate(prev);
```

### 11.2 订阅回调在 Inactive 状态仍执行

**这是正常行为**！在回调中手动检查状态

### 11.3 状态转换超时

**原因**：on_configure 等回调中执行了阻塞操作

**解决**：回调中不要长时间阻塞，耗时操作放到工作线程

### 11.4 设计清单

- [ ] on_activate/on_deactivate 是否调用了父类方法？
- [ ] deactivate 时是否发送了零速度/安全状态？
- [ ] 订阅回调中是否检查了节点状态？
- [ ] on_error 是否能恢复到 Unconfigured？
- [ ] on_configure 中的参数验证是否完整？
- [ ] 硬件连接失败时是否返回 FAILURE？
- [ ] 多节点启动顺序是否符合依赖关系？

---

## 12. 快速参考

### 12.1 状态速查

```
ID  状态              可执行转换
─── ──────────────── ──────────────────────────────
1   Unconfigured     configure → Inactive / shutdown → Finalized
2   Inactive         activate → Active / cleanup → Unconfigured / shutdown → Finalized
3   Active           deactivate → Inactive / shutdown → Finalized
4   Finalized        (不可转换)
```

### 12.2 转换 ID 速查

```
ID  转换名            触发回调           路径
─── ──────────────── ────────────────── ──────────────────
1   configure        on_configure       Unconfigured → Inactive
2   cleanup          on_cleanup         Inactive → Unconfigured
3   shutdown         on_shutdown        * → Finalized
4   activate         on_activate        Inactive → Active
5   deactivate       on_deactivate      Active → Inactive
```

### 12.3 C++ API 速查

```cpp
// ═══ 创建生命周期节点 ═══
class MyNode : public rclcpp_lifecycle::LifecycleNode {
    MyNode() : LifecycleNode("my_node") {}
};

// ═══ 创建 LifecyclePublisher ═══
auto pub = create_publisher<MsgType>("/topic", qos);
bool active = pub->is_activated();

// ═══ 获取当前状态 ═══
auto state = get_current_state();
state.id();      // 状态 ID
state.label();   // 状态名称

// ═══ 回调返回值 ═══
CallbackReturn::SUCCESS
CallbackReturn::FAILURE
CallbackReturn::ERROR

// ═══ 必须调用父类 ═══
LifecycleNode::on_activate(prev_state);
LifecycleNode::on_deactivate(prev_state);
```

### 12.4 设计模式速查

```
场景                         → 推荐设计
───────────────────────────── ──────────────────────────────
传感器驱动                   → 生命周期驱动：configure连硬件/activate采数据
导航栈                       → 生命周期管理器：有序启停
安全关键系统                 → deactivate发送零速度+始终监控
多子系统                     → 协调器：按依赖顺序启动/逆序关闭
调试/维护                    → deactivate暂停/activate恢复
硬件故障                     → on_error恢复/on_configure重试
```

---

## 示例索引

| # | 文件 | 知识点 | 运行方式 |
|---|------|--------|---------|
| ① | `lifecycle_basic.cpp` | 4个转换回调、状态查询、命令行操作 | `ros2 run lifecycle_learning_cpp lifecycle_basic` |
| ② | `lifecycle_transitions.cpp` | 状态转换路径、返回值、on_error | `ros2 run lifecycle_learning_cpp lifecycle_transitions` |
| ③ | `lifecycle_publisher.cpp` | LifecyclePublisher 自动启用/禁用 | `ros2 run lifecycle_learning_cpp lifecycle_publisher` |
| ④ | `lifecycle_subscriber.cpp` | 订阅在 Inactive 下仍执行、状态检查 | `ros2 run lifecycle_learning_cpp lifecycle_subscriber` |
| ⑤ | `lifecycle_manager_node.cpp` | 服务调用管理、自动启停 | `ros2 run lifecycle_learning_cpp lifecycle_manager_node` |
| ⑥ | `lifecycle_launch_node.cpp` | Launch 编排生命周期 | `ros2 launch lifecycle_learning_cpp 06_lifecycle_launch.launch.py` |
| ⑦ | `lifecycle_callback_groups.cpp` | LifecycleNode+回调组 | `ros2 run lifecycle_learning_cpp lifecycle_callback_groups` |
| ⑧ | `robot_sensor_driver.cpp` | IMU驱动生命周期 | `ros2 run lifecycle_learning_cpp robot_sensor_driver` |
| ⑨ | `robot_nav_lifecycle.cpp` | 导航安全关键系统 | `ros2 run lifecycle_learning_cpp robot_nav_lifecycle` |
| ⑩ | `robot_system_coordinator.cpp` | 多子系统有序启停 | `ros2 run lifecycle_learning_cpp robot_system_coordinator` |
