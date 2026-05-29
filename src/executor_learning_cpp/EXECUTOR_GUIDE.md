# ROS2 执行器（Executor）完全指南 —— C++ 版

> 本文档是 `executor_learning_cpp` 功能包的配套说明，涵盖 ROS2 执行器的所有核心概念、API、实战模式和机器人应用案例。

---

## 目录

1. [执行器概述](#1-执行器概述)
2. [执行模型深度解析](#2-执行模型深度解析)
3. [三种执行器类型](#3-三种执行器类型)
4. [spin 系列方法详解](#4-spin-系列方法详解)
5. [多节点管理](#5-多节点管理)
6. [多执行器架构](#6-多执行器架构)
7. [执行器与回调组配合](#7-执行器与回调组配合)
8. [线程安全](#8-线程安全)
9. [机器人实战案例](#9-机器人实战案例)
10. [高级话题](#10-高级话题)
11. [常见问题与调试](#11-常见问题与调试)
12. [快速参考](#12-快速参考)

---

## 1. 执行器概述

### 1.1 什么是执行器？

执行器（Executor）是 ROS2 中的**核心调度组件**，负责：

1. **等待**：监听所有已注册节点的等待集（wait set），检测哪些回调已就绪
2. **调度**：从就绪的回调中选择一个（或多个）执行
3. **执行**：在执行器线程中运行回调函数

```
┌─────────────────────────────────────────┐
│              Executor                    │
│                                          │
│  ① wait_for_ready_callbacks()           │
│     ↓                                    │
│  ② 检查回调组互斥约束                    │
│     ↓                                    │
│  ③ execute_callback()                   │
│     ↓                                    │
│  ④ 回到 ①                               │
│                                          │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐   │
│  │Node A│ │Node B│ │Node C│ │Node D│   │
│  └──────┘ └──────┘ └──────┘ └──────┘   │
└─────────────────────────────────────────┘
```

### 1.2 为什么需要执行器？

| 方面 | ROS1 | ROS2 |
|------|------|------|
| 回调调度 | 全局多线程（每个回调一个线程） | 执行器统一调度 |
| 线程模型 | 不可控 | 可选择单线程/多线程 |
| 回调互斥 | 无保证 | 回调组精确控制 |
| 实时性 | 无法保证 | StaticSingleThreadedExecutor 支持确定性 |
| 资源消耗 | 回调数量 × 线程 | 执行器线程池 |

### 1.3 rclcpp::spin() 的本质

```cpp
// rclcpp::spin(node) 等价于：
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();

// spin() 内部伪代码：
while (rclcpp::ok()) {
    wait_for_ready_callbacks(timeout);  // 等待回调就绪
    execute_next_callback();             // 执行一个就绪回调
}
```

---

## 2. 执行模型深度解析

### 2.1 等待-调度-执行循环

```
时间 ──────────────────────────────────────────►

SingleThreadedExecutor:
├── wait ──┤── exec_A ──┤── wait ──┤── exec_B ──┤── wait ──┤
                         ↑ 串行执行，一次只执行一个回调

MultiThreadedExecutor (2线程):
├── wait ──┤── exec_A ───────────────┤
├── wait ──┤── exec_B ──┤── exec_C ──┤
             ↑ 多线程并发执行

StaticSingleThreadedExecutor:
├── wait ──┤── exec_A ──┤── exec_B ──┤── exec_C ──┤
             ↑ 零分配等待，确定性执行
```

### 2.2 等待集（Wait Set）

执行器内部维护一个**等待集**，包含：
- 定时器句柄
- 订阅句柄
- 服务端句柄
- 客户端句柄
- 等待（waitable）句柄

```cpp
// 等待集的工作方式
wait_set.add_timer(timer1_handle);
wait_set.add_subscription(sub1_handle);
wait_set.add_service(srv1_handle);
// ...

// 阻塞等待直到任一实体就绪
wait_set.wait(timeout);

// 检查哪些就绪了
if (wait_set.is_timer_ready(timer1_handle)) { /* 执行 timer1 回调 */ }
if (wait_set.is_subscription_ready(sub1_handle)) { /* 执行 sub1 回调 */ }
```

### 2.3 调度策略

ROS2 执行器采用**先就绪先执行**策略（FIFO），但有回调组约束：

```
就绪队列: [Timer_A, Sub_B, Timer_C, Sub_D]

检查回调组约束:
  Timer_A (group1) → group1 未被占用 → 执行
  Sub_B   (group1) → group1 已被占用 → 跳过
  Timer_C (group2) → group2 未被占用 → 执行
  Sub_D   (reentrant) → 不受约束 → 执行
```

---

## 3. 三种执行器类型

### 3.1 SingleThreadedExecutor

```cpp
#include <rclcpp/executors/single_threaded_executor.hpp>

// 创建方式
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

**特点**：
- 单线程执行所有回调
- 严格串行，无并发
- 无线程安全问题
- 支持动态 add_node / remove_node
- **适用**：简单节点、不需要并发的场景

**内部流程**：
1. 收集所有节点的可等待实体
2. 调用 `rcl_wait()` 等待回调就绪
3. **每次 spin_once 都重建等待集** → 有内存分配
4. 按优先级选择一个就绪回调执行

### 3.2 MultiThreadedExecutor

```cpp
#include <rclcpp/executors/multi_threaded_executor.hpp>

// 创建方式（指定线程数）
rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),  // 选项
    4                           // 线程数
);
executor.add_node(node);
executor.spin();
```

**特点**：
- 线程池并发执行回调
- 遵守回调组互斥约束
- 不同回调组可并发执行
- 线程数默认 = `std::thread::hardware_concurrency()`
- **适用**：多回调组、需要并发、I/O 密集型

**线程数选择**：
```
推荐线程数 = max(回调组数量, 关键回调数 + 1)

示例：
  3 个互斥回调组 + 1 个可重入组 → 4 线程
  1 个高频控制组 + 2 个低频处理组 → 3 线程
```

**内部流程**：
1. 主线程收集就绪回调
2. 将就绪回调分发给工作线程
3. 工作线程检查回调组约束后执行
4. 主线程继续等待新回调

### 3.3 StaticSingleThreadedExecutor

```cpp
#include <rclcpp/executors/static_single_threaded_executor.hpp>

// 创建方式
rclcpp::executors::StaticSingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

**特点**：
- **首次 spin 时一次性收集所有回调** → 之后零分配
- 单线程，确定性执行
- **不支持**动态 add_node / remove_node（添加后不会生效！）
- 最低延迟抖动
- **适用**：实时系统、高频控制、嵌入式平台

**与 SingleThreadedExecutor 的关键区别**：

| 方面 | SingleThreadedExecutor | StaticSingleThreadedExecutor |
|------|----------------------|------------------------------|
| 等待集构建 | 每次 spin_once 重建 | 首次构建，之后复用 |
| 内存分配 | 每次有分配 | 零分配 |
| add_node/remove_node | 支持 | 不支持（静默失败） |
| 延迟抖动 | 较高 | 最低 |
| 适用场景 | 一般应用 | 实时控制 |

### 3.4 三种执行器选择决策树

```
需要并发执行回调？
├── 否
│   ├── 需要动态添加/移除节点？
│   │   ├── 是 → SingleThreadedExecutor
│   │   └── 否 → StaticSingleThreadedExecutor（更优性能）
│   └── 需要最低延迟抖动？
│       ├── 是 → StaticSingleThreadedExecutor
│       └── 否 → SingleThreadedExecutor
└── 是
    └── MultiThreadedExecutor
        └── 线程数 ≥ 回调组数量
```

---

## 4. spin 系列方法详解

### 4.1 spin()

```cpp
// 阻塞式无限循环
executor.spin();

// 内部等价于：
while (rclcpp::ok()) {
    execute_callback(wait_for_ready_callback());
}
```

- **阻塞**当前线程
- **无限循环**直到 `rclcpp::shutdown()` 或 `executor.cancel()`
- 最常用的方式

### 4.2 spin_once()

```cpp
// 执行一个回调（带超时）
executor.spin_once(std::chrono::milliseconds(100));

// 内部等价于：
auto callback = wait_for_ready_callback(timeout);
if (callback) {
    callback();  // 执行一个回调
}
// 返回 —— 只执行了一个！
```

- **非阻塞或短阻塞**（取决于超时）
- 每次调用只执行**一个**回调
- 需要放在循环中才有持续效果
- 适用：需要在回调之间插入自定义逻辑

```cpp
// 典型用法：自定义执行循环
while (rclcpp::ok()) {
    executor.spin_once(100ms);  // 执行一个回调

    // 在这里插入自定义逻辑
    update_gui();
    check_system_status();
}
```

### 4.3 spin_some()

```cpp
// 执行所有当前就绪的回调（有最大时长）
executor.spin_some(std::chrono::milliseconds(50));

// 内部等价于：
auto start = now();
while (has_ready_callbacks() && (now() - start) < max_duration) {
    execute_next_callback();
}
```

- **非阻塞**（不等待新回调就绪）
- 执行**所有当前就绪**的回调（受 max_duration 限制）
- 适用：与其他事件循环集成

```cpp
// 典型用法：与 GUI 事件循环集成
while (rclcpp::ok()) {
    executor.spin_some(50ms);  // 处理所有就绪回调（最多50ms）

    // GUI 事件处理
    cv::imshow("image", image);
    if (cv::waitKey(1) == 27) break;
}
```

### 4.4 三种方法对比

| 方法 | 阻塞行为 | 执行数量 | 适用场景 |
|------|---------|---------|---------|
| `spin()` | 阻塞 | 所有（持续） | 标准节点 |
| `spin_once(timeout)` | 短阻塞 | 1个 | 自定义循环、状态机 |
| `spin_some(max_duration)` | 非阻塞 | 所有就绪的 | GUI集成、优先级调度 |

---

## 5. 多节点管理

### 5.1 单执行器多节点

```cpp
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node1);
executor.add_node(node2);
executor.add_node(node3);
executor.spin();
```

**优点**：
- 简单，一个执行器管理所有
- 节点间通信在同一进程，零拷贝优化

**缺点**：
- 一个节点的耗时回调可能影响其他节点（单线程时）
- 无法给不同节点分配不同的实时性保证

### 5.2 动态添加/移除节点

```cpp
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node1);

// 运行中添加节点
executor.add_node(node2);

// 运行中移除节点
executor.remove_node(node1);

// ⚠️ StaticSingleThreadedExecutor 不支持动态操作！
```

### 5.3 get_all_nodes()

```cpp
// 获取执行器管理的所有节点
auto nodes = executor.get_all_nodes();
for (auto& node : nodes) {
    RCLCPP_INFO(logger, "节点: %s", node->get_name());
}
```

---

## 6. 多执行器架构

### 6.1 设计模式

```
模式一：单执行器多节点（最简单）
┌────────────────────────┐
│  MultiThreadedExecutor │
│  ┌──────┐ ┌──────┐    │
│  │Node A│ │Node B│    │
│  └──────┘ └──────┘    │
└────────────────────────┘

模式二：多执行器隔离（推荐用于实时系统）
┌──────────────┐  ┌──────────────┐
│ Executor 1   │  │ Executor 2   │
│ ┌──────┐     │  │ ┌──────┐     │
│ │Node A│     │  │ │Node B│     │
│ └──────┘     │  │ └──────┘     │
└──────────────┘  └──────────────┘
   Thread 1           Thread 2

模式三：混合架构
┌──────────────────┐  ┌──────────────────┐
│ StaticSingle     │  │ MultiThreaded    │
│ ┌──────────────┐ │  │ ┌──────┐ ┌────┐ │
│ │ 实时控制节点  │ │  │ │Node B│ │Node│ │
│ └──────────────┘ │  │ └──────┘ └────┘ │
└──────────────────┘  └──────────────────┘
```

### 6.2 多执行器实现

```cpp
// 创建多个执行器
auto rt_executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
auto normal_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 2);

rt_executor->add_node(rt_node);
normal_executor->add_node(node1);
normal_executor->add_node(node2);

// 每个执行器在独立线程中运行
std::thread rt_thread([rt_executor]() { rt_executor->spin(); });
std::thread normal_thread([normal_executor]() { normal_executor->spin(); });

// 等待结束
rt_thread.join();
normal_thread.join();
```

### 6.3 优雅关闭

```cpp
// 方式一：使用 atomic 标志
std::atomic<bool> shutdown_flag{false};
std::signal(SIGINT, [](int) { shutdown_flag = true; });

// 在独立线程中驱动执行器
std::thread t([&]() { executor.spin(); });

// 主线程监控
while (rclcpp::ok() && !shutdown_flag) {
    std::this_thread::sleep_for(100ms);
}

// 优雅取消
executor.cancel();
t.join();
rclcpp::shutdown();

// 方式二：使用 rclcpp::Context
auto context = std::make_shared<rclcpp::Context>();
// ... 创建节点时传入 context ...
// 关闭时：
context->shutdown("reason");
```

---

## 7. 执行器与回调组配合

### 7.1 完整配合矩阵

```
MultiThreadedExecutor 下的行为：

              │ 互斥组A  │ 互斥组B  │ 可重入组 │ 默认组
──────────────┼──────────┼──────────┼──────────┼──────────
互斥组A回调1  │  串行    │  并发    │  并发    │  并发
互斥组A回调2  │  串行    │  并发    │  并发    │  并发
互斥组B回调1  │  并发    │  串行    │  并发    │  并发
可重入组回调1 │  并发    │  并发    │  并发    │  并发
默认组回调    │  并发    │  并发    │  并发    │  串行*

* 默认组 = MutuallyExclusive，同组内串行
```

### 7.2 关键规则

1. **SingleThreadedExecutor** → 所有回调串行，回调组区分无意义
2. **MultiThreadedExecutor + 互斥组** → 同组串行，不同组并发
3. **MultiThreadedExecutor + 可重入组** → 同组可并发
4. **线程数 < 回调组数** → 部分回调组无法并发执行

### 7.3 服务调用场景

```cpp
// ❌ 死锁：同组内服务端+客户端
auto group = node->create_callback_group(CallbackGroupType::MutuallyExclusive);
auto srv = node->create_service<SrvType>("/service", cb, ServicesQoS(), group);
auto client = node->create_client<SrvType>("/service", ServicesQoS(), group);
// 定时器在 group 中调用 client->call() → 死锁！

// ✅ 正确：服务端和调用方在不同回调组
auto srv_group = node->create_callback_group(CallbackGroupType::MutuallyExclusive);
auto call_group = node->create_callback_group(CallbackGroupType::MutuallyExclusive);
auto srv = node->create_service<SrvType>("/service", cb, ServicesQoS(), srv_group);
auto client = node->create_client<SrvType>("/service", ServicesQoS(), call_group);
```

---

## 8. 线程安全

### 8.1 跨回调组数据访问

```cpp
class MyNode : public rclcpp::Node {
    std::mutex data_mutex_;
    double shared_data_;

    // 回调组A的回调
    void callback_a() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        shared_data_ = compute_value();
    }

    // 回调组B的回调（可能与A并发执行！）
    void callback_b() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        double value = shared_data_;  // 安全读取
    }
};
```

### 8.2 std::atomic vs std::mutex

| 工具 | 适用场景 | 开销 |
|------|---------|------|
| `std::atomic<int>` | 简单计数器、标志位 | 最低 |
| `std::mutex` | 复杂共享数据结构 | 中等 |
| `std::shared_mutex` | 读多写少 | 较高 |

```cpp
// 简单计数器 —— 用 atomic
std::atomic<int> message_count_{0};
message_count_++;  // 无需加锁

// 复杂数据 —— 用 mutex
std::mutex data_mutex_;
std::vector<Detection> detections_;
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    detections_.push_back(new_detection);
}
```

---

## 9. 机器人实战案例

### 9.1 案例一：导航栈（示例⑨）

**问题**：速度控制 50Hz 绝不能被路径规划 0.5Hz 阻塞

**方案**：5 个独立回调组 + MultiThreadedExecutor

```
优先级     回调组        频率     说明
───────── ───────────── ──────── ──────────
最高       control_group 50Hz    速度控制，绝不阻塞
高         imu_group     100Hz   IMU 数据
高         odom_group    50Hz    里程计
中         scan_group    10Hz    激光扫描
低         planning_group 0.5Hz  路径规划（可耗时1-2秒）
```

### 9.2 案例二：多臂协调（示例⑩）

**问题**：左臂/右臂/底盘需要独立实时性保证

**方案**：3 个独立执行器，每个在独立线程中运行

```
执行器1 (StaticSingleThreaded): 左臂 100Hz
执行器2 (StaticSingleThreaded): 右臂 100Hz
执行器3 (SingleThreaded):       底盘 50Hz + 协调器 1Hz
```

### 9.3 案例三：SLAM 系统

```
┌─────────────────────────────────────────────────┐
│  MultiThreadedExecutor (4线程)                    │
│                                                   │
│  ┌─────────────┐  ┌─────────────┐               │
│  │ 里程计组     │  │ 激光组       │               │
│  │ (50Hz)      │  │ (10Hz)      │               │
│  └─────────────┘  └─────────────┘               │
│                                                   │
│  ┌─────────────┐  ┌─────────────┐               │
│  │ 地图更新组   │  │ 回环检测组   │               │
│  │ (1Hz)       │  │ (0.2Hz)     │               │
│  └─────────────┘  └─────────────┘               │
└─────────────────────────────────────────────────┘
```

### 9.4 案例四：无人机飞控

```
┌──────────────────────────────────────────────────┐
│  混合执行器架构                                    │
│                                                    │
│  执行器1 (StaticSingleThreaded):                   │
│    ┌────────────────────────────┐                 │
│    │ 姿态控制 500Hz              │  ← 最高优先级  │
│    └────────────────────────────┘                 │
│                                                    │
│  执行器2 (MultiThreadedExecutor, 2线程):           │
│    ┌──────────┐  ┌──────────┐                    │
│    │ GPS 10Hz │  │ 气压计   │                    │
│    └──────────┘  └──────────┘                    │
│                                                    │
│  执行器3 (SingleThreadedExecutor):                 │
│    ┌──────────┐  ┌──────────┐                    │
│    │ 任务规划  │  │ 通信     │                    │
│    └──────────┘  └──────────┘                    │
└──────────────────────────────────────────────────┘
```

### 9.5 案例五：工业机器人

```
┌──────────────────────────────────────────────────┐
│  执行器1 (StaticSingleThreaded): 安全控制器 1kHz  │
│  执行器2 (StaticSingleThreaded): 运动控制器 500Hz │
│  执行器3 (MultiThreadedExecutor): 视觉处理 30Hz   │
│  执行器4 (SingleThreadedExecutor): HMI/日志 1Hz   │
└──────────────────────────────────────────────────┘
```

---

## 10. 高级话题

### 10.1 ExecutorOptions

```cpp
rclcpp::ExecutorOptions opts;
// 共享上下文（用于多执行器共享同一 DDS 域）
opts.context = custom_context;

rclcpp::executors::MultiThreadedExecutor executor(opts, 4);
```

### 10.2 自定义执行器

```cpp
class PriorityExecutor : public rclcpp::Executor {
public:
    using rclcpp::Executor::Executor;

    void spin() override {
        while (rclcpp::ok()) {
            // 自定义调度逻辑
            auto callbacks = wait_for_ready_callbacks(timeout);
            // 按优先级排序
            sort_by_priority(callbacks);
            // 执行最高优先级
            execute_callback(callbacks.front());
        }
    }

private:
    void sort_by_priority(std::vector<Callback>& callbacks) {
        // 自定义优先级排序
    }
};
```

### 10.3 Composition 与执行器

```cpp
// 组件化节点可以手动指定执行器
class MyComponent : public rclcpp::Node {
    // ...
};

// 在组件管理器中运行时，默认使用 MultiThreadedExecutor
// 可以通过参数指定执行器类型
```

### 10.4 执行时间监控

```cpp
class MonitoredExecutor : public rclcpp::Executor {
    void execute_callback(Callback& cb) override {
        auto start = std::chrono::steady_clock::now();

        cb();  // 执行回调

        auto duration = std::chrono::steady_clock::now() - start;
        if (duration > threshold) {
            RCLCPP_WARN(logger, "回调耗时超限: %.3fms",
                std::chrono::duration<double, std::milli>(duration).count());
        }
    }
};
```

### 10.5 C++ vs Python 执行器 API 对照

| 操作 | Python (rclpy) | C++ (rclcpp) |
|------|---------------|-------------|
| 单线程执行器 | `SingleThreadedExecutor()` | `SingleThreadedExecutor exec;` |
| 多线程执行器 | `MultiThreadedExecutor(num_threads=4)` | `MultiThreadedExecutor exec(opts, 4);` |
| 静态执行器 | `StaticSingleThreadedExecutor()` (Jazzy+) | `StaticSingleThreadedExecutor exec;` |
| 添加节点 | `executor.add_node(node)` | `executor.add_node(node)` |
| 移除节点 | `executor.remove_node(node)` | `executor.remove_node(node)` |
| spin | `executor.spin()` | `executor.spin()` |
| spin_once | `executor.spin_once(timeout_sec)` | `executor.spin_once(timeout)` |
| spin_some | `executor.spin_some(timeout_sec)` | `executor.spin_some(max_duration)` |
| 取消 | `executor.shutdown()` | `executor.cancel()` |
| 获取节点 | `executor.get_nodes()` | `executor.get_all_nodes()` |

---

## 11. 常见问题与调试

### 11.1 高频回调被低频回调阻塞

**症状**：控制循环 50Hz 实际只能 10Hz

**原因**：
- 使用了 SingleThreadedExecutor
- 高频和低频回调在同一互斥组

**解决**：
```cpp
// 1. 换用 MultiThreadedExecutor
// 2. 高频回调放在独立回调组
auto control_group = node->create_callback_group(CallbackGroupType::MutuallyExclusive);
control_timer = node->create_wall_timer(20ms, callback, control_group);
```

### 11.2 StaticSingleThreadedExecutor 添加节点无效

**症状**：add_node 后节点不执行

**原因**：StaticSingleThreadedExecutor 在首次 spin 时固定回调列表

**解决**：
```cpp
// ✅ 在 spin 之前添加所有节点
executor.add_node(node1);
executor.add_node(node2);
executor.spin();

// ❌ spin 后添加无效
executor.spin();  // 开始执行
executor.add_node(node3);  // 无效！
```

### 11.3 多线程下数据竞争

**症状**：偶尔出现段错误或数据异常

**调试**：
```cpp
// 1. 启用 ThreadSanitizer
// CMake: add_compile_options(-fsanitize=thread)

// 2. 检查跨回调组数据访问
// 所有跨回调组的共享数据必须加锁

// 3. 使用 RCLCPP_INFO 输出线程 ID
RCLCPP_INFO(logger, "线程=%zu", get_thread_id());
```

### 11.4 服务调用死锁

**症状**：调用服务后永远无响应

**原因**：调用方和服务端在同一互斥回调组

**解决**：参见[7.3 服务调用场景](#73-服务调用场景)

### 11.5 性能优化清单

- [ ] 高频回调放在独立回调组
- [ ] 实时控制使用 StaticSingleThreadedExecutor
- [ ] 线程数 ≥ 回调组数量
- [ ] 回调内避免阻塞操作（sleep、等待I/O）
- [ ] 使用 `rclcpp::SensorDataQoS()` 减少传感器数据拷贝
- [ ] 共享数据用 `std::atomic` 代替 `std::mutex`（简单变量时）
- [ ] 回调中尽量用 `RCLCPP_DEBUG` / `RCLCPP_INFO_THROTTLE` 减少日志开销

---

## 12. 快速参考

### 12.1 执行器速查

```
┌────────────────────────┬────────────┬─────────────┬────────────────────┐
│                        │ Single     │ Multi       │ StaticSingle       │
├────────────────────────┼────────────┼─────────────┼────────────────────┤
│ 线程数                  │ 1          │ N (可配置)   │ 1                  │
│ 并发执行                │ 否         │ 是           │ 否                 │
│ 动态add/remove_node    │ 是         │ 是           │ 否                 │
│ 内存分配                │ 每次       │ 每次         │ 零分配（首次后）    │
│ 延迟抖动                │ 中         │ 高           │ 最低               │
│ 实时性                  │ 中         │ 低           │ 最高               │
│ 适用场景                │ 一般应用    │ 并发处理     │ 实时控制/嵌入式     │
└────────────────────────┴────────────┴─────────────┴────────────────────┘
```

### 12.2 spin 方法速查

```
┌───────────┬──────────────┬───────────┬────────────────────────────┐
│ 方法       │ 阻塞          │ 执行数量   │ 适用                       │
├───────────┼──────────────┼───────────┼────────────────────────────┤
│ spin()    │ 无限阻塞      │ 所有(持续) │ 标准节点                   │
│ spin_once │ 短阻塞(超时)  │ 1个       │ 自定义循环、状态机          │
│ spin_some │ 非阻塞       │ 就绪的全部 │ GUI集成、优先级调度         │
└───────────┴──────────────┴───────────┴────────────────────────────┘
```

### 12.3 C++ API 速查

```cpp
// ═══ 执行器创建 ═══
rclcpp::executors::SingleThreadedExecutor executor;
rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
rclcpp::executors::StaticSingleThreadedExecutor executor;

// ═══ 节点管理 ═══
executor.add_node(node_ptr);
executor.remove_node(node_ptr);
auto nodes = executor.get_all_nodes();

// ═══ 驱动方式 ═══
executor.spin();
executor.spin_once(timeout);
executor.spin_some(max_duration);
executor.cancel();  // 取消 spin

// ═══ rclcpp 快捷方式 ═══
rclcpp::spin(node);                           // = SingleThreaded + spin
rclcpp::spin_some(node);                      // = SingleThreaded + spin_some
rclcpp::executors::MultiThreadedExecutor exec;
exec.add_node(node);
exec.spin();
```

### 12.4 设计模式速查

```
场景                          → 推荐架构
────────────────────────────── ──────────────────────────────
简单节点，无并发需求            → SingleThreadedExecutor
多传感器并发处理                → MultiThreadedExecutor + 互斥组
实时控制(>100Hz)               → StaticSingleThreadedExecutor
导航栈(控制+规划)              → MultiThreadedExecutor + 5组
多臂协调                       → 多执行器(Static × N)
GUI + ROS                     → spin_some() + GUI主循环
SLAM(里程计+地图+回环)        → MultiThreadedExecutor + 4组
无人机飞控                     → 混合架构(Static+Multi+Single)
```

---

## 示例索引

| # | 文件 | 知识点 | 运行方式 |
|---|------|--------|---------|
| ① | `single_threaded_basic.cpp` | spin()原理、手动创建执行器 | `ros2 run executor_learning_cpp single_threaded_basic` |
| ② | `multi_threaded_demo.cpp` | MultiThreadedExecutor、线程安全 | `ros2 run executor_learning_cpp multi_threaded_demo` |
| ③ | `static_executor.cpp` | StaticSingleThreadedExecutor、零分配 | `ros2 run executor_learning_cpp static_executor` |
| ④ | `executor_benchmark.cpp` | 三种执行器性能对比 | `ros2 run executor_learning_cpp executor_benchmark [1|2|3]` |
| ⑤ | `spin_methods.cpp` | spin/spin_once/spin_some | `ros2 run executor_learning_cpp spin_methods [1|2|3|4]` |
| ⑥ | `multi_node_single_executor.cpp` | 多节点单执行器 | `ros2 run executor_learning_cpp multi_node_single_executor [1|2|3]` |
| ⑦ | `multi_executor_arch.cpp` | 多执行器隔离 | `ros2 run executor_learning_cpp multi_executor_arch` |
| ⑧ | `executor_callback_groups.cpp` | 执行器+回调组配合 | `ros2 run executor_learning_cpp executor_callback_groups` |
| ⑨ | `robot_nav_executor.cpp` | 导航栈实战 | `ros2 run executor_learning_cpp robot_nav_executor` |
| ⑩ | `robot_multi_arm.cpp` | 多臂协调实战 | `ros2 run executor_learning_cpp robot_multi_arm` |
