# ROS2 Action 完全指南

> 从基础概念到机器人实战，系统掌握 ROS2 Action 通信机制

---

## 目录

1. [Action 概述](#1-action-概述)
2. [Action vs Topic vs Service](#2-action-vs-topic-vs-service)
3. [Action 通信模型](#3-action-通信模型)
4. [Action 消息定义](#4-action-消息定义)
5. [Action Server 详解](#5-action-server-详解)
6. [Action Client 详解](#6-action-client-详解)
7. [Goal 生命周期](#7-goal-生命周期)
8. [反馈机制](#8-反馈机制)
9. [取消机制](#9-取消机制)
10. [目标策略](#10-目标策略)
11. [Action 与 LifecycleNode](#11-action-与-lifecyclenode)
12. [实战案例](#12-实战案例)
13. [高级话题](#13-高级话题)
14. [最佳实践与常见问题](#14-最佳实践与常见问题)
15. [快速参考](#15-快速参考)

---

## 1. Action 概述

### 什么是 Action？

Action 是 ROS2 中的**第三种通信机制**，专为**长时间运行的任务**设计。它结合了 Service 的请求-响应模式和 Topic 的持续数据流，提供了：

- **Goal**（目标）：客户端发送的任务请求
- **Feedback**（反馈）：执行过程中的实时进度
- **Result**（结果）：任务最终完成状态

### 为什么需要 Action？

| 场景 | Topic | Service | Action |
|------|-------|---------|--------|
| 传感器数据流 | ✅ | ❌ | ❌ |
| 一次性查询 | ❌ | ✅ | ❌ |
| 导航到目标点 | ❌ | ⚠️ 无进度 | ✅ |
| 机械臂运动 | ❌ | ⚠️ 无进度 | ✅ |
| 文件下载/处理 | ❌ | ⚠️ 无进度 | ✅ |

**核心判断标准**：任务是否**耗时**且需要**实时进度**？如果是 → Action。

### Action 的典型应用

```
🤖 机器人导航    → "去厨房" (Goal) → "距离5m..." (Feedback) → "已到达" (Result)
🦾 机械臂控制    → "转到90°" (Goal) → "当前45°..." (Feedback) → "到位" (Result)
📸 图像处理     → "识别物体" (Goal) → "处理50%..." (Feedback) → "结果" (Result)
🔧 系统校准     → "校准传感器" (Goal) → "第3/5步..." (Feedback) → "完成" (Result)
📦 任务调度     → "执行任务A" (Goal) → "子任务2/5..." (Feedback) → "完成" (Result)
```

---

## 2. Action vs Topic vs Service

### 三种通信机制对比

| 特性 | Topic | Service | Action |
|------|-------|---------|--------|
| 通信模式 | 发布-订阅 | 请求-响应 | 目标-反馈-结果 |
| 方向 | 单向（多对多） | 双向（一对一） | 双向（一对一） |
| 同步/异步 | 异步 | 同步 | 异步 |
| 持续性 | 持续数据流 | 一次性 | 持续到完成 |
| 进度反馈 | ❌ | ❌ | ✅ |
| 可取消 | ❌ | ❌ | ✅ |
| 执行时间 | 无限制 | 应短（<1s） | 可长（秒~分~时） |
| 典型频率 | 10~1000Hz | 按需 | 按需 |

### 底层实现

Action **并非**一种全新的传输层，而是构建在 Topic 和 Service 之上的：

```
Action 底层通信：
  Goal   → Service (客户端调用, 服务端响应接受/拒绝)
  Cancel → Service (客户端请求取消)
  Result → Service (服务端返回最终结果)
  Feedback → Topic (服务端持续发布)
  Status   → Topic (Action 状态信息)
```

可以通过 `ros2 topic list` 和 `ros2 service list` 观察这些底层接口。

---

## 3. Action 通信模型

### 完整通信流程

```
    Action Client                           Action Server
        |                                       |
        |──── Goal Request ────────────────────>|    (Service调用)
        |                                       | handle_goal()
        |<─── Goal Response (ACCEPT/REJECT) ────|
        |                                       |
        |                                       | handle_accepted()
        |                                       |   → 启动 execute()
        |                                       |
        |<─── Feedback ─────────────────────────|    (Topic发布)
        |<─── Feedback ─────────────────────────|    可多次
        |<─── Feedback ─────────────────────────|
        |                                       |
        |──── Cancel Request ──────────────────>|    (可选, Service调用)
        |<─── Cancel Response ──────────────────|
        |                                       | is_canceling() → true
        |<─── Feedback (canceled) ──────────────|
        |                                       |
        |<─── Result ───────────────────────────|    (Service调用)
        |                                       |
```

### 关键时间点

| 时间点 | 触发条件 | 回调 |
|--------|----------|------|
| `handle_goal` | 客户端发送目标 | 决定接受/拒绝 |
| `handle_accepted` | 目标被接受 | 启动执行逻辑 |
| `publish_feedback` | 执行过程中 | 发布进度 |
| `is_canceling` | 客户端请求取消 | 检查并处理 |
| `succeed/abort/canceled` | 执行结束 | 返回最终结果 |

---

## 4. Action 消息定义

### .action 文件格式

Action 消息定义在 `.action` 文件中，三个部分用 `---` 分隔：

```
# Goal    —— 客户端发送的目标内容
<类型> <字段名>
---
# Result  —— 服务端返回的最终结果
<类型> <字段名>
---
# Feedback —— 服务端发布的实时反馈
<类型> <字段名>
```

### 本包定义的 3 个 Action

#### CountUp.action — 基础计数

```
# Goal
int64 target                # 目标计数值
---
# Result
int64 final_count           # 最终计数值
int64[] sequence            # 完整序列
---
# Feedback
int64 current_count         # 当前值
float32 progress_percent    # 进度百分比
```

#### MoveArm.action — 机械臂控制

```
# Goal
float64 target_angle        # 目标角度 (弧度)
float64 speed               # 速度因子 (0.1~2.0)
---
# Result
bool success                # 是否成功
float64 final_angle         # 最终角度
float64 elapsed_time        # 耗时(秒)
string message              # 描述信息
---
# Feedback
float64 current_angle       # 当前角度
float32 progress_percent    # 进度百分比
string status               # 状态描述
```

#### Navigate.action — 机器人导航

```
# Goal
float64 target_x            # 目标X坐标
float64 target_y            # 目标Y坐标
float64 speed               # 速度因子
---
# Result
bool success                # 是否成功
float64 final_x             # 最终X
float64 final_y             # 最终Y
float64 elapsed_time        # 耗时
string message              # 描述
---
# Feedback
float64 current_x           # 当前X
float64 current_y           # 当前Y
float32 progress_percent    # 进度
float64 distance_remaining  # 剩余距离
string status               # 状态
```

### 自动生成的 C++ 类型

`.action` 文件编译后生成以下 C++ 类型：

```cpp
// 包含头文件
#include <action_learning_cpp/action/count_up.hpp>

// 生成的类型
action_learning_cpp::action::CountUp          // Action 类型本身
action_learning_cpp::action::CountUp::Goal    // 目标消息
action_learning_cpp::action::CountUp::Result  // 结果消息
action_learning_cpp::action::CountUp::Feedback // 反馈消息
```

### CMakeLists.txt 配置

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUp.action"
  "action/MoveArm.action"
  "action/Navigate.action"
)

# ROS2 Jazzy 需要链接类型支持
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(your_executable "${cpp_typesupport_target}")
```

---

## 5. Action Server 详解

### 创建 Action Server

```cpp
#include <rclcpp_action/rclcpp_action.hpp>

auto server = rclcpp_action::create_server<ActionType>(
    node_shared_ptr,           // 节点指针
    "action_name",             // Action 名称
    handle_goal_callback,      // 目标请求回调
    handle_cancel_callback,    // 取消请求回调
    handle_accepted_callback   // 目标接受后的回调
);
```

### 三个核心回调

#### handle_goal — 决定是否接受目标

```cpp
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,     // 目标唯一ID
    std::shared_ptr<const ActionType::Goal> goal) // 目标内容
{
    // 验证目标合法性
    if (goal->target <= 0) {
        return rclcpp_action::GoalResponse::REJECT;  // 拒绝
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // 接受
    // return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER; // 接受但延迟
}
```

| 返回值 | 含义 |
|--------|------|
| `ACCEPT_AND_EXECUTE` | 立即接受目标，触发 `handle_accepted` |
| `REJECT` | 拒绝目标，客户端的 `goal_response_callback` 收到空指针 |
| `ACCEPT_AND_DEFER` | 接受但不立即执行，需后续手动调用 `execute()` |

#### handle_cancel — 决定是否允许取消

```cpp
rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
    return rclcpp_action::CancelResponse::ACCEPT;   // 允许取消
    // return rclcpp_action::CancelResponse::REJECT; // 拒绝取消
}
```

#### handle_accepted — 启动执行

```cpp
void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
    // ⚠️ 必须在新线程中执行！不能阻塞 executor
    std::thread{std::bind(&MyNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}
```

### execute 函数 — 执行逻辑

```cpp
void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ActionType::Result>();
    auto feedback = std::make_shared<ActionType::Feedback>();

    for (...) {
        // 1. 检查取消
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);  // ← 终止方式之一
            return;
        }

        // 2. 发布反馈
        goal_handle->publish_feedback(feedback);

        // 3. 执行一步
        // ...
    }

    // 4. 成功完成
    goal_handle->succeed(result);   // ← 终止方式之一
    // 或失败
    // goal_handle->abort(result);  // ← 终止方式之一
}
```

### GoalHandle 的三种终止方式

| 方法 | 含义 | 客户端 ResultCode |
|------|------|-------------------|
| `succeed(result)` | 执行成功 | `SUCCEEDED` |
| `abort(result)` | 执行失败/异常终止 | `ABORTED` |
| `canceled(result)` | 被取消 | `CANCELED` |

⚠️ **每个 GoalHandle 只能调用一次终止方法！调用后不能再 publish_feedback。**

---

## 6. Action Client 详解

### 创建 Action Client

```cpp
auto client = rclcpp_action::create_client<ActionType>(node_shared_ptr, "action_name");

// 等待服务端上线
if (!client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(logger, "Server 未上线");
}
```

### 发送目标

```cpp
auto goal_msg = ActionType::Goal();
goal_msg.target = 10;

auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();

// 回调1: 服务端接受/拒绝
send_goal_options.goal_response_callback =
    [](const rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr& goal_handle) {
        if (!goal_handle) {
            // 目标被拒绝
        } else {
            // 目标被接受
        }
    };

// 回调2: 收到反馈
send_goal_options.feedback_callback =
    [](rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr,
       const std::shared_ptr<const ActionType::Feedback> feedback) {
        // 处理反馈
    };

// 回调3: 收到结果
send_goal_options.result_callback =
    [](const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: break;
        case rclcpp_action::ResultCode::ABORTED:   break;
        case rclcpp_action::ResultCode::CANCELED:  break;
        default: break;
        }
    };

client_->async_send_goal(goal_msg, send_goal_options);
```

### 取消目标

```cpp
// 需要保存 goal_handle
auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);
// ... 稍后取消
client_->async_cancel_goal(goal_handle);
// 或取消所有目标
client_->async_cancel_all_goals();
```

### 获取结果（异步方式）

```cpp
// 方法1：通过 result_callback（推荐）
send_goal_options.result_callback = [](const WrappedResult& result) { ... };

// 方法2：单独请求结果
auto goal_handle = ...; // 从 goal_response_callback 获取
auto result_future = client_->async_get_result(goal_handle);
```

---

## 7. Goal 生命周期

### 状态机

```
                          handle_goal() 返回 ACCEPT
                          ┌──────────────────────────┐
                          │                          ▼
  [新建] ──send_goal──► [PENDING] ──────────► [EXECUTING]
                          │                     │  │  │
                          │ REJECT              │  │  │ succeed()
                          ▼                     │  │  ▼
                       [REJECTED]               │  │ [SUCCEEDED]
                                               │  │
                        handle_cancel()         │  │ abort()
                        + is_canceling()        │  │   ▼
                              │                 │  │ [ABORTED]
                              ▼                 │  │
                         [CANCELING] ───────────┘  │
                              │                    │
                              │ canceled()         │
                              ▼                    │
                         [CANCELED]                │
```

### GoalUUID

每个目标都有唯一的 UUID，可用于日志追踪：

```cpp
rclcpp_action::GoalUUID uuid;  // std::array<uint8_t, 16>
// 转换为字符串
std::string uuid_str = rclcpp_action::to_string(uuid);
```

---

## 8. 反馈机制

### 反馈频率选择

| 应用场景 | 推荐频率 | 说明 |
|----------|----------|------|
| UI 进度条 | 10~30Hz | 人眼流畅 |
| 机器人控制 | 50~200Hz | 控制循环需要 |
| 状态监控 | 1~5Hz | 节省带宽 |
| 文件处理 | 0.5~1Hz | 进度变化慢 |

### 反馈数据设计原则

1. **自包含**：每条反馈应包含完整信息，不依赖前一条
2. **轻量级**：反馈数据应尽量小（高频发布）
3. **可计算**：提供足够信息让客户端计算 ETA、速度等

```cpp
// ✅ 好的设计：自包含
feedback->current_angle = 1.57;
feedback->progress_percent = 50.0f;

// ❌ 差的设计：依赖增量
feedback->delta_angle = 0.01;  // 需要累加才能得到当前角度
```

### ETA 估算（客户端）

```cpp
send_goal_options.feedback_callback =
    [this, start_time](auto, auto feedback) {
        float progress = feedback->progress_percent / 100.0f;
        auto elapsed = (this->now() - start_time).seconds();
        if (progress > 0.01) {
            double eta = elapsed / progress * (1.0 - progress);
            RCLCPP_INFO(logger, "ETA: %.1fs", eta);
        }
    };
```

---

## 9. 取消机制

### 取消流程

```
Client                        Server
  |                              |
  |── async_cancel_goal() ──────>|  handle_cancel() 返回 ACCEPT
  |                              |  is_canceling() == true
  |                              |  清理资源...
  |<── Result (CANCELED) ───────|  goal_handle->canceled(result)
```

### 服务端优雅取消

```cpp
void execute(GoalHandleSharedPtr goal_handle) {
    // ... 执行中 ...
    if (goal_handle->is_canceling()) {
        // 1. 保存中间结果
        result->partial_data = current_data;
        // 2. 清理资源（关闭文件、停止硬件等）
        cleanup_resources();
        // 3. 标记为已取消
        goal_handle->canceled(result);
        return;
    }
}
```

### 客户端主动取消

```cpp
// 方法1：保存 goal_handle 后取消
auto send_options = SendGoalOptions();
send_options.goal_response_callback = [this](auto handle) {
    if (handle) {
        current_goal_handle_ = handle;
        // 设置定时器 3 秒后取消
        cancel_timer_ = create_wall_timer(3s, [this]() {
            client_->async_cancel_goal(current_goal_handle_);
            cancel_timer_->cancel();
        });
    }
};

// 方法2：取消所有目标
client_->async_cancel_all_goals();
```

---

## 10. 目标策略

### 三种常见策略

#### REJECT — 忙碌时拒绝

```cpp
rclcpp_action::GoalResponse handle_goal(const auto& uuid, const auto& goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
        RCLCPP_WARN(logger, "忙碌中，拒绝新目标");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

**适用**：关键任务不允许中断（如手术机器人、精密装配）

#### PREEMPT — 抢占旧目标

```cpp
rclcpp_action::GoalResponse handle_goal(const auto& uuid, const auto& goal) {
    // 抢占：直接接受新目标，旧目标在 execute 中检测到被替换后自行取消
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

**适用**：需要响应最新指令（如导航中的目标点更新、遥控操作）

#### QUEUE — 排队执行

```cpp
// 需要维护一个目标队列
std::queue<GoalHandleSharedPtr> goal_queue_;

rclcpp_action::GoalResponse handle_goal(const auto& uuid, const auto& goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    // 目标加入队列，在当前目标完成后依次执行
}
```

**适用**：批处理场景（如 3D 打印、图像批量处理）

### 策略选择指南

```
是否允许中断当前任务？
    ├── 是 → 对最新指令响应优先级高？
    │       ├── 是 → PREEMPT（导航、遥控）
    │       └── 否 → QUEUE（批处理、3D打印）
    └── 否 → REJECT（手术、精密装配）
```

---

## 11. Action 与 LifecycleNode

### 生命周期与 Action 的对应关系

| 生命周期状态 | Action 行为 | 说明 |
|-------------|------------|------|
| Unconfigured | 服务不存在 | Action Server 未创建 |
| Inactive | 服务存在但拒绝目标 | 可查询/取消，但不接受新目标 |
| Active | 正常接受和执行 | 完整功能 |
| Finalized | 服务已销毁 | 不可用 |

### 代码模式

```cpp
class LifecycleActionServer : public rclcpp_lifecycle::LifecycleNode {
    std::atomic<bool> accepting_goals_{false};

    CallbackReturn on_configure(const State&) {
        // 创建 Action Server
        action_server_ = rclcpp_action::create_server<ActionType>(
            this->shared_from_this(), "action_name", ...);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const State&) {
        accepting_goals_ = true;   // 开始接受目标
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const State&) {
        accepting_goals_ = false;  // 停止接受新目标
        return CallbackReturn::SUCCESS;
    }

    rclcpp_action::GoalResponse handle_goal(...) {
        if (!accepting_goals_) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
};
```

⚠️ **LifecycleNode 创建 Action Server 时，需使用 `this->shared_from_this()` 而非 `this`**。

---

## 12. 实战案例

### 案例一：机械臂控制（08_robot_arm_demo.cpp）

**场景**：单关节机械臂，接收目标角度和速度指令，平滑运动到目标位置。

```
客户端发送: target_angle=1.57, speed=1.0
服务端执行: 0.0 → 0.05 → 0.10 → ... → 1.57 (20Hz反馈)
服务端反馈: current_angle, progress_percent, status
服务端结果: success, final_angle, elapsed_time
```

**关键设计**：
- 20Hz 控制频率（平滑运动）
- 速度因子控制（0.1x~2.0x）
- 目标验证（角度范围、速度范围）
- 取消时保存当前角度

### 案例二：机器人导航（09_robot_nav_demo.cpp）

**场景**：2D 平面导航，模拟机器人移动到目标位置，支持障碍物检测。

```
客户端发送: target_x=3.0, target_y=4.0, speed=1.0
服务端执行: (0,0) → (0.05,0.07) → ... → (3.0,4.0) (20Hz反馈)
服务端反馈: current_x, current_y, distance_remaining, status
服务端结果: success, final_x, final_y, elapsed_time
```

**关键设计**：
- 距离和进度计算
- 障碍物模拟（2%概率触发，0.5s延迟）
- 路径点巡逻（4个路径点循环）

### 案例三：多 Action 协调（10_task_coordinator.cpp）

**场景**：协调机械臂和导航两个 Action Server，执行复合任务。

```
Phase 1 (并行): 导航到目标 + 机械臂准备
Phase 2 (顺序): 机械臂抓取
Phase 3 (并行): 导航返回 + 机械臂收回
```

**关键设计**：
- `std::promise/std::future` 模式汇聚异步结果
- 轮询式等待（100ms 间隔，不阻塞 executor）
- 任一阶段失败则终止流水线

---

## 13. 高级话题

### Action Server 的执行线程

```cpp
// 方式1：detach 线程（简单但不可控）
void handle_accepted(GoalHandleSharedPtr handle) {
    std::thread{std::bind(&Node::execute, this, _1), handle}.detach();
}

// 方式2：线程池（可控并发数）
std::vector<std::thread> thread_pool_;
void handle_accepted(GoalHandleSharedPtr handle) {
    thread_pool_.emplace_back(&Node::execute, this, handle);
}

// 方式3：std::async（RAII 管理）
void handle_accepted(GoalHandleSharedPtr handle) {
    std::async(std::launch::async, &Node::execute, this, handle);
}
```

### 多 Action Server 共存

一个节点可以同时拥有多个 Action Server：

```cpp
arm_server_ = rclcpp_action::create_server<MoveArm>(this, "move_arm", ...);
nav_server_ = rclcpp_action::create_server<Navigate>(this, "navigate", ...);
gripper_server_ = rclcpp_action::create_server<Gripper>(this, "gripper", ...);
```

### Action 与 QoS

Action 底层使用不同的 QoS 策略：

| 底层通道 | 默认 QoS |
|----------|----------|
| Goal Service | ServicesQoS (Reliable) |
| Cancel Service | ServicesQoS (Reliable) |
| Result Service | ServicesQoS (Reliable) |
| Feedback Topic | SensorDataQoS (BestEffort) |
| Status Topic | SystemDefaultQoS |

### Action 的 ROS2 CLI 工具

```bash
# 列出所有 Action
ros2 action list

# 查看 Action 类型
ros2 action type /count_up

# 查看 Action 信息
ros2 action info /count_up

# 发送目标（CLI 方式）
ros2 action send_goal /count_up action_learning_cpp/action/CountUp "{target: 5}"

# 发送目标并显示反馈
ros2 action send_goal /count_up action_learning_cpp/action/CountUp "{target: 5}" --feedback

# 查看消息定义
ros2 interface show action_learning_cpp/action/CountUp
```

---

## 14. 最佳实践与常见问题

### 最佳实践

#### 1. execute 函数必须不阻塞 executor

```cpp
// ❌ 错误：在 handle_accepted 中直接执行
void handle_accepted(GoalHandleSharedPtr handle) {
    execute(handle);  // 阻塞 executor！无法处理其他回调
}

// ✅ 正确：在新线程中执行
void handle_accepted(GoalHandleSharedPtr handle) {
    std::thread{std::bind(&Node::execute, this, _1), handle}.detach();
}
```

#### 2. 每一步都检查取消

```cpp
// ❌ 错误：循环中不检查取消
for (int i = 0; i < target; i++) {
    do_work();
    // 用户取消后不会立即停止
}

// ✅ 正确：每步检查
for (int i = 0; i < target; i++) {
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
    }
    do_work();
}
```

#### 3. 取消后做资源清理

```cpp
if (goal_handle->is_canceling()) {
    // 1. 停止硬件
    stop_motor();
    // 2. 保存中间状态
    result->partial_state = get_current_state();
    // 3. 释放锁
    release_locks();
    // 4. 返回取消结果
    goal_handle->canceled(result);
    return;
}
```

#### 4. 反馈要轻量且自包含

```cpp
// ✅ 好：客户端无需累加
feedback->current_angle = get_current_angle();
feedback->progress_percent = compute_progress();

// ❌ 差：需要客户端维护状态
feedback->delta_angle = get_delta();
```

#### 5. 目标验证放在 handle_goal 中

```cpp
rclcpp_action::GoalResponse handle_goal(...) {
    // ✅ 提前验证，避免无效执行
    if (goal->target < 0) return GoalResponse::REJECT;
    if (is_busy()) return GoalResponse::REJECT;
    return GoalResponse::ACCEPT_AND_EXECUTE;
}
```

### 常见问题

#### Q1：future.wait_for() 在 SingleThreadedExecutor 中超时？

**原因**：`wait_for()` 阻塞了 executor 线程，服务响应回调无法被投递。

**解决**：使用回调模式 `async_send_request(request, callback)`，或使用 `MultiThreadedExecutor`，或使用轮询 + `wait_for(0ms)`。

#### Q2：如何让 Action Server 只执行一个目标？

在 `handle_goal` 中检查是否有正在执行的目标：

```cpp
if (current_goal_handle_ && current_goal_handle_->is_active()) {
    return rclcpp_action::GoalResponse::REJECT;
}
```

#### Q3：execute 中如何获取节点时间？

```cpp
void execute(GoalHandleSharedPtr handle) {
    auto start = this->now();  // 使用节点时间（可仿真）
    // ...
    auto elapsed = (this->now() - start).seconds();
}
```

#### Q4：GoalHandle 的SharedPtr 生命周期？

- `handle_accepted` 提供的 `goal_handle` 是 `SharedPtr`
- 必须保存它直到 execute 完成
- 如果用 `std::thread` + `std::bind`，bind 会复制 SharedPtr，保证生命周期

#### Q5：如何调试 Action 通信？

```bash
# 监控 Action 状态
ros2 action list
ros2 action info /move_arm

# 监控底层话题
ros2 topic echo /move_arm/_action/feedback
ros2 topic echo /move_arm/_action/status

# CLI 发送目标
ros2 action send_goal /move_arm action_learning_cpp/action/MoveArm \
    "{target_angle: 1.57, speed: 1.0}" --feedback
```

---

## 15. 快速参考

### Server 速查

```cpp
// 创建
auto server = rclcpp_action::create_server<ActionType>(
    node, "action_name", handle_goal, handle_cancel, handle_accepted);

// handle_goal
GoalResponse::ACCEPT_AND_EXECUTE  // 接受
GoalResponse::REJECT    // 拒绝

// handle_cancel
CancelResponse::ACCEPT  // 允许取消
CancelResponse::REJECT  // 拒绝取消

// execute 中
goal_handle->is_canceling()        // 是否被取消
goal_handle->publish_feedback(fb)  // 发布反馈
goal_handle->succeed(result)       // 成功完成
goal_handle->abort(result)         // 异常终止
goal_handle->canceled(result)      // 被取消
```

### Client 速查

```cpp
// 创建
auto client = rclcpp_action::create_client<ActionType>(node, "action_name");
client->wait_for_action_server(timeout);

// 发送目标
auto goal = ActionType::Goal();
SendGoalOptions options;
options.goal_response_callback = ...;   // 接受/拒绝回调
options.feedback_callback = ...;        // 反馈回调
options.result_callback = ...;          // 结果回调
client->async_send_goal(goal, options);

// 取消
client->async_cancel_goal(goal_handle);
client->async_cancel_all_goals();

// 结果代码
ResultCode::SUCCEEDED  // 成功
ResultCode::ABORTED    // 失败
ResultCode::CANCELED   // 取消
ResultCode::UNKNOWN    // 未知
```

### 本包 10 个节点对照表

| 编号 | 节点 | 类型 | 知识点 |
|------|------|------|--------|
| 01 | action_server_basic | Server | 基础创建、handle_goal/cancel/accepted、execute |
| 02 | action_client_basic | Client | 基础发送、三个回调、ResultCode |
| 03 | action_server_feedback | Server | 10Hz反馈、互斥执行、优雅取消 |
| 04 | action_client_feedback | Client | 进度条、ETA估算、参数化目标 |
| 05 | action_cancel_demo | Server+Client | 取消流程、资源清理、定时取消 |
| 06 | action_server_policy | Server | REJECT/PREEMPT策略、目标验证 |
| 07 | action_lifecycle_server | Lifecycle | 按状态启停Action、lifecycle+action |
| 08 | robot_arm_demo | Server+Client | MoveArm实战、关节模拟、速度控制 |
| 09 | robot_nav_demo | Server+Client | Navigate实战、路径点、障碍检测 |
| 10 | task_coordinator | Client | 多Action协调、并行+顺序、promise/future |

### Launch 文件对照表

| 编号 | Launch | 节点组合 |
|------|--------|----------|
| 01 | 01_action_basic | server_basic + client_basic |
| 02 | 02_action_feedback | server_feedback + client_feedback |
| 03 | 03_action_cancel | cancel_demo (自包含) |
| 04 | 04_action_goal_policy | server_policy + client_basic |
| 05 | 05_action_lifecycle | lifecycle_server + client_basic |
| 06 | 06_robot_arm | robot_arm_demo (自包含) |
| 07 | 07_robot_nav | robot_nav_demo (自包含) |
| 08 | 08_task_coordinator | arm_demo + nav_demo + coordinator |
