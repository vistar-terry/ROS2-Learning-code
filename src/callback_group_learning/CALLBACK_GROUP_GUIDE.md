# ROS2 回调组（Callback Group）完全指南

> 本文档是 `callback_group_learning` 功能包的配套指南，全面覆盖 ROS2 回调组的原理、使用方法和机器人实际应用。

---

## 目录

1. [概述：为什么需要回调组](#1-概述为什么需要回调组)
2. [执行模型：单线程 vs 多线程](#2-执行模型单线程-vs-多线程)
3. [回调组类型详解](#3-回调组类型详解)
4. [执行器详解](#4-执行器详解)
5. [回调组与执行器的配合矩阵](#5-回调组与执行器的配合矩阵)
6. [线程安全专题](#6-线程安全专题)
7. [死锁：原因与解决方案](#7-死锁原因与解决方案)
8. [回调组设计原则](#8-回调组设计原则)
9. [示例代码详解](#9-示例代码详解)
10. [机器人实际应用案例](#10-机器人实际应用案例)
11. [高级话题](#11-高级话题)
12. [常见问题与调试](#12-常见问题与调试)
13. [快速参考](#13-快速参考)

---

## 1. 概述：为什么需要回调组

### 1.1 问题背景

在 ROS1 中，回调是串行执行的（单线程 spin），不存在并发问题。但在 ROS2 中，引入了多线程执行器，多个回调可以并发执行，这带来了：

- **竞态条件**：多个回调同时访问共享变量
- **回调阻塞**：耗时回调阻塞紧急回调
- **死锁**：回调间互相等待

**回调组（Callback Group）** 就是 ROS2 用来解决这些问题的核心机制。

### 1.2 什么是回调组

回调组是一组回调的集合，定义了组内回调的并发规则：

```
┌─────────────────── Node ───────────────────┐
│                                             │
│  ┌── CallbackGroup A ──┐  ┌── CallbackGroup B ──┐
│  │  - timer1_callback  │  │  - sub1_callback     │
│  │  - sub2_callback     │  │  - timer2_callback   │
│  │  - srv_callback      │  │                      │
│  └──────────────────────┘  └──────────────────────┘
│                                             │
│  组内规则由回调组类型决定                     │
│  组间关系：不同组可并发执行                    │
└─────────────────────────────────────────────┘
```

### 1.3 回调组控制的回调类型

以下所有类型的回调都受回调组控制：

| 回调类型 | 创建方式 | 说明 |
|---------|---------|------|
| 定时器回调 | `create_timer()` | 周期性执行 |
| 订阅回调 | `create_subscription()` | 收到消息时触发 |
| 服务端回调 | `create_service()` | 收到服务请求时触发 |
| 客户端响应回调 | `call_async()` + `add_done_callback()` | 服务响应到达时 |
| 服务事件回调 | `send_request()` (rclpy) | 请求发送/响应接收 |

### 1.4 ROS1 vs ROS2 对比

| 特性 | ROS1 | ROS2 |
|------|------|------|
| 默认执行模型 | 单线程 spin() | 单线程但支持多线程 |
| 回调并发 | 不支持 | 通过回调组控制 |
| 回调优先级 | 不支持 | 通过回调组间接实现 |
| 线程安全 | 不需要 | 多线程执行器下必须考虑 |
| 回调阻塞 | 全局影响 | 限于同组回调 |

---

## 2. 执行模型：单线程 vs 多线程

### 2.1 ROS2 的执行流程

```
┌───────────────────────────────────────────────────┐
│                   Executor                         │
│                                                   │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐     │
│   │ 线程1   │    │ 线程2   │    │ 线程3   │     │
│   └────┬────┘    └────┬────┘    └────┬────┘     │
│        │              │              │            │
│        ▼              ▼              ▼            │
│   ┌─────────────────────────────────────────┐    │
│   │           就绪回调队列                     │    │
│   │  [timer_cb] [sub_cb] [srv_cb] [timer_cb] │    │
│   └─────────────────────────────────────────┘    │
│                    │                              │
│        根据回调组规则调度执行                       │
└───────────────────────────────────────────────────┘
```

### 2.2 SingleThreadedExecutor

```python
# rclpy.spin() 默认使用 SingleThreadedExecutor
rclpy.spin(node)
```

- 只有一个线程
- 所有回调串行执行
- 无论回调组类型如何，都无法并发
- **最简单、最安全**，但无法利用多核

### 2.3 MultiThreadedExecutor

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
rclpy.spin(node, executor=executor)
```

- 多个线程
- 不同回调组的回调可以并发
- 同一 MutuallyExclusiveCallbackGroup 内仍串行
- 同一 ReentrantCallbackGroup 内可并发
- **需要考虑线程安全**

### 2.4 手动执行器循环

```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)

while rclpy.ok():
    # 执行一批就绪回调
    executor.spin_once(timeout_sec=0.1)
    
    # 在此处插入自定义逻辑
    custom_processing()
```

---

## 3. 回调组类型详解

### 3.1 MutuallyExclusiveCallbackGroup（互斥回调组）

**规则**：同一组内的回调**不能并发执行**。

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# 创建互斥回调组
my_group = MutuallyExclusiveCallbackGroup()

# 将回调分配到此组
self.create_timer(1.0, self.timer_cb, callback_group=my_group)
self.create_subscription(MsgType, '/topic', self.sub_cb, 10, callback_group=my_group)
```

**行为表**：

| 情况 | 行为 |
|------|------|
| callback_a 执行中，callback_b 就绪 | callback_b 等待 callback_a 完成 |
| callback_a 和 callback_b 同时就绪 | 先执行一个，再执行另一个 |
| 与其他组的回调 | 可以并发 |

**适用场景**：
- 保护共享数据（变量、列表、字典）
- 保护硬件接口（串口、GPIO）
- 确保操作的原子性

### 3.2 ReentrantCallbackGroup（可重入回调组）

**规则**：同一组内的回调**可以并发执行**（包括同一回调的多次调用）。

```python
from rclpy.callback_groups import ReentrantCallbackGroup

# 创建可重入回调组
my_group = ReentrantCallbackGroup()

self.create_timer(1.0, self.timer_cb, callback_group=my_group)
self.create_subscription(MsgType, '/topic', self.sub_cb, 10, callback_group=my_group)
```

**行为表**：

| 情况 | 行为 |
|------|------|
| callback_a 执行中，callback_b 就绪 | callback_b 立即执行（并发） |
| 同一 callback 的多次调用 | 可以并发（同一回调的不同调用可以重叠） |
| 与其他组的回调 | 可以并发 |

**适用场景**：
- 无状态的计算任务
- 耗时的服务端处理（允许并发处理多个请求）
- 需要最大并发度的场景

**⚠️ 注意**：使用 ReentrantCallbackGroup 时，必须自行保证线程安全！

### 3.3 默认回调组

每个节点都有一个默认回调组，类型为 `MutuallyExclusiveCallbackGroup`：

```python
# 查看默认回调组
default_group = node.default_callback_group
print(type(default_group))  # MutuallyExclusiveCallbackGroup

# 不指定 callback_group 时，回调自动加入默认回调组
self.create_timer(1.0, self.timer_cb)  # → 默认回调组
```

---

## 4. 执行器详解

### 4.1 执行器架构

```
┌────────────────────────────────────────────────┐
│                  Executor                       │
│                                                │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │ 线程池   │ │ 线程池   │ │ 线程池   │       │
│  │ Thread-0 │ │ Thread-1 │ │ Thread-2 │       │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘       │
│       │             │             │            │
│  ┌────▼─────────────▼─────────────▼──────┐     │
│  │         WaitSet (等待集)               │     │
│  │  监控所有注册节点的就绪事件             │     │
│  └──────────────────┬───────────────────┘     │
│                     │                          │
│  ┌──────────────────▼───────────────────┐     │
│  │         回调调度器                     │     │
│  │  根据回调组规则决定哪些回调可并发       │     │
│  └──────────────────────────────────────┘     │
│                                                │
│  注册的节点:                                    │
│  - node_a                                      │
│  - node_b                                      │
│  - ...                                         │
└────────────────────────────────────────────────┘
```

### 4.2 执行器类型对比

| 特性 | SingleThreadedExecutor | MultiThreadedExecutor |
|------|----------------------|----------------------|
| 线程数 | 1 | 可配置（默认=CPU核数） |
| 回调并发 | 不可能 | 不同回调组可并发 |
| MutuallyExclusive 组 | 串行（自然如此） | 同组串行，不同组并发 |
| Reentrant 组 | 串行（单线程限制） | 可并发 |
| 线程安全 | 不需要 | 必须考虑 |
| 性能 | 低 | 高 |
| 复杂度 | 简单 | 复杂 |

### 4.3 执行器创建方式

```python
# 方式1：rclpy.spin()（默认 SingleThreadedExecutor）
rclpy.spin(node)

# 方式2：指定执行器
executor = MultiThreadedExecutor(num_threads=4)
rclpy.spin(node, executor=executor)

# 方式3：手动循环
executor = MultiThreadedExecutor()
executor.add_node(node)
while rclpy.ok():
    executor.spin_once(timeout_sec=0.1)

# 方式4：多节点执行器
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node_a)
executor.add_node(node_b)
rclpy.spin_once(timeout_sec=0.1, executor=executor)
```

### 4.4 Executor 的回调选择算法

当有多个就绪回调时，执行器的选择算法：

1. 检查回调的回调组
2. 如果回调组已有回调在执行中：
   - `MutuallyExclusive` → 跳过此回调
   - `Reentrant` → 可以执行
3. 如果回调组没有回调在执行中 → 可以执行
4. 选择可执行的回调分配给空闲线程

---

## 5. 回调组与执行器的配合矩阵

### 5.1 完整配合矩阵

```
                    │ MutuallyExclusive  │ Reentrant
────────────────────┼────────────────────┼──────────────────
SingleThreaded      │ 同组串行           │ 串行(单线程)
MultiThreaded      │ 同组串行,不同组并发 │ 同组可并发
```

### 5.2 多个回调组的交互

```
┌─ MutuallyExclusive Group A ─┐  ┌─ MutuallyExclusive Group B ─┐
│  timer1_cb  ─┐               │  │  timer3_cb  ─┐               │
│  sub1_cb   ─┘ 串行执行       │  │  sub3_cb   ─┘ 串行执行       │
└──────────────────────────────┘  └──────────────────────────────┘
         │                                  │
         └──── 可以并发 ────┘

┌─ Reentrant Group C ──────────────────────────────────────────┐
│  timer2_cb ─┐                                               │
│  sub2_cb  ─┘ 可以并发（含自身的多次调用）                     │
└──────────────────────────────────────────────────────────────┘
         │
         └──── 与 Group A/B 也可以并发 ────┘
```

### 5.3 可视化执行时间线

```
SingleThreadedExecutor:
Thread-0  |---A1---|---B1---|---A2---|---B2---|---A3---|

MultiThreadedExecutor + MutuallyExclusive(A, B):
Thread-0  |---A1---|       |---A2---|       |---A3---|
Thread-1  |       |---B1---|       |---B2---|

MultiThreadedExecutor + Reentrant(A, B):
Thread-0  |---A1---|---A2---|---A3---|
Thread-1  |---B1---|---B2---|        |
           ↑ A1和B1并发执行 ↑ A2和B2并发执行
```

---

## 6. 线程安全专题

### 6.1 为什么需要线程安全

使用 `MultiThreadedExecutor` + 不同回调组时，多个回调可能同时访问共享数据：

```python
# ❌ 不安全！竞态条件
class UnsafeNode(Node):
    def __init__(self):
        super().__init__('unsafe_node')
        self.counter = 0
        
        self.group_a = MutuallyExclusiveCallbackGroup()
        self.group_b = MutuallyExclusiveCallbackGroup()
        
        # 不同回调组 → 可以并发 → 竞态！
        self.timer_a = self.create_timer(0.1, self.cb_a, callback_group=self.group_a)
        self.timer_b = self.create_timer(0.1, self.cb_b, callback_group=self.group_b)
    
    def cb_a(self):
        temp = self.counter  # 读取
        time.sleep(0.01)     # 被中断
        self.counter = temp + 1  # 写回（丢失更新）
    
    def cb_b(self):
        temp = self.counter
        time.sleep(0.01)
        self.counter = temp + 1
```

### 6.2 解决方案1：锁（Lock）

```python
import threading

class SafeNodeWithLock(Node):
    def __init__(self):
        super().__init__('safe_node')
        self.counter = 0
        self.lock = threading.Lock()  # 互斥锁
        
        self.group_a = MutuallyExclusiveCallbackGroup()
        self.group_b = MutuallyExclusiveCallbackGroup()
        
        self.timer_a = self.create_timer(0.1, self.cb_a, callback_group=self.group_a)
        self.timer_b = self.create_timer(0.1, self.cb_b, callback_group=self.group_b)
    
    def cb_a(self):
        with self.lock:  # 加锁
            self.counter += 1  # 原子操作
    
    def cb_b(self):
        with self.lock:
            self.counter += 1
```

### 6.3 解决方案2：同一 MutuallyExclusive 回调组

```python
class SafeNodeWithGroup(Node):
    def __init__(self):
        super().__init__('safe_node')
        self.counter = 0
        
        # 两个回调放在同一个 MutuallyExclusive 回调组
        # 串行执行，无需加锁
        self.safe_group = MutuallyExclusiveCallbackGroup()
        
        self.timer_a = self.create_timer(0.1, self.cb_a, callback_group=self.safe_group)
        self.timer_b = self.create_timer(0.1, self.cb_b, callback_group=self.safe_group)
    
    def cb_a(self):
        self.counter += 1  # 安全，无需加锁
    
    def cb_b(self):
        self.counter += 1  # 安全，无需加锁
```

### 6.4 两种方案对比

| 特性 | Lock | MutuallyExclusive 回调组 |
|------|------|--------------------------|
| 并发性 | 可以（其他代码仍并发） | 不可以（同组串行） |
| 粒度 | 细粒度（只锁共享数据） | 粗粒度（整个回调串行） |
| 死锁风险 | 有（锁顺序不当） | 有（服务调用场景） |
| 代码复杂度 | 需要手动加锁 | 声明式，简单 |
| 适用场景 | 需要并发但保护共享数据 | 不需要并发，只需安全 |

### 6.5 Python 线程安全工具

```python
import threading

# 1. Lock（互斥锁）—— 最常用
lock = threading.Lock()
with lock:
    # 临界区
    pass

# 2. RLock（可重入锁）—— 同一线程可多次获取
rlock = threading.RLock()
with rlock:
    with rlock:  # 不会死锁
        pass

# 3. Event（事件）—— 线程间通知
event = threading.Event()
event.wait()       # 等待
event.set()        # 通知

# 4. Condition（条件变量）—— 复杂等待/通知
cond = threading.Condition()
with cond:
    cond.wait_for(lambda: data_ready)
    # 处理数据

# 5. Queue（线程安全队列）—— 生产者-消费者
from queue import Queue
q = Queue()
q.put(data)       # 生产
data = q.get()     # 消费
```

---

## 7. 死锁：原因与解决方案

### 7.1 经典死锁场景：同组服务调用

```
┌─────────── MutuallyExclusive Group ──────────┐
│                                              │
│  timer_callback:                             │
│    1. 获取组锁 ✓                              │
│    2. 调用服务 srv.call_async()               │
│    3. 等待服务回调执行...                      │
│                                              │
│  service_callback:                           │
│    1. 需要获取组锁 ✗（已被timer持有）           │
│    2. 无法执行 → 服务永远无法响应               │
│                                              │
│  结果: 死锁！timer等待service，service等待锁   │
└──────────────────────────────────────────────┘
```

### 7.2 解决方案1：不同回调组

```python
# ✅ 正确：服务端和调用方在不同回调组
self.timer_group = MutuallyExclusiveCallbackGroup()
self.service_group = MutuallyExclusiveCallbackGroup()

self.create_service(SrvType, '/service', self.srv_cb, 
                    callback_group=self.service_group)
self.create_timer(1.0, self.timer_cb,
                  callback_group=self.timer_group)
```

### 7.3 解决方案2：ReentrantCallbackGroup

```python
# ✅ 正确：使用 Reentrant 回调组
self.reentrant_group = ReentrantCallbackGroup()

self.create_service(SrvType, '/service', self.srv_cb,
                    callback_group=self.reentrant_group)
self.create_timer(1.0, self.timer_cb,
                  callback_group=self.reentrant_group)
```

### 7.4 解决方案3：直接方法调用（最佳实践）

```python
# ✅ 最佳：不在回调中调用自己的服务
# 而是直接调用内部方法
def timer_cb(self):
    # 直接调用方法，不走服务
    result = self.do_something()

def do_something(self):
    """核心逻辑封装为普通方法"""
    return True, 'success'
```

### 7.5 跨节点死锁

```
Node A: callback_a 中 call_async(Node B 的服务)
Node B: callback_b 中 call_async(Node A 的服务)

如果双方都在 MutuallyExclusive 回调组中且使用单线程执行器 → 死锁
解决方案：
1. 为服务调用创建独立的回调组
2. 使用 MultiThreadedExecutor
3. 避免循环依赖
```

### 7.6 死锁检测清单

- [ ] 服务端和调用方是否在同一 MutuallyExclusive 回调组？
- [ ] 是否在回调中同步等待（`future.result()` 或 `rclpy.spin_until_future_complete`）？
- [ ] 是否有跨节点循环服务调用？
- [ ] 是否使用了单线程执行器但需要并发？
- [ ] 锁的获取顺序是否一致？

---

## 8. 回调组设计原则

### 8.1 设计决策树

```
需要多个回调吗？
├── 否 → 使用默认回调组即可
└── 是 → 有回调需要并发执行吗？
    ├── 否 → 全部放在一个 MutuallyExclusive 回调组
    └── 是 → 有共享数据需要保护吗？
        ├── 否 → 使用 ReentrantCallbackGroup
        └── 是 → 共享数据的回调放在同一 MutuallyExclusive 回调组
                  不共享数据的回调放在独立的回调组
```

### 8.2 设计模式

#### 模式1：按数据分组（最常用）

```python
# 每组共享数据对应一个 MutuallyExclusive 回调组
self.lidar_group = MutuallyExclusiveCallbackGroup()  # 保护障碍物数据
self.imu_group = MutuallyExclusiveCallbackGroup()    # 保护姿态数据
self.camera_group = ReentrantCallbackGroup()          # 无共享数据，最大并发
```

#### 模式2：按优先级分组

```python
# 安全关键模块使用独立回调组，确保不被阻塞
self.critical_group = MutuallyExclusiveCallbackGroup()  # 避障、急停
self.normal_group = MutuallyExclusiveCallbackGroup()     # 常规处理
self.background_group = MutuallyExclusiveCallbackGroup() # 后台任务
```

#### 模式3：按功能模块分组

```python
# 每个功能模块独立回调组
self.navigation_group = MutuallyExclusiveCallbackGroup()
self.perception_group = ReentrantCallbackGroup()
self.communication_group = MutuallyExclusiveCallbackGroup()
```

### 8.3 最佳实践总结

| 实践 | 原因 |
|------|------|
| 默认使用 MutuallyExclusive | 安全、简单 |
| 高频回调放在独立回调组 | 避免被低频回调阻塞 |
| 共享数据的回调放在同一互斥组 | 避免竞态条件 |
| 耗时处理用 Reentrant | 允许并发 |
| 服务端和调用方不在同一互斥组 | 避免死锁 |
| 需要并发时才用 MultiThreadedExecutor | 减少复杂度 |
| 使用锁保护跨组共享数据 | 即使回调组不同也要保护 |

---

## 9. 示例代码详解

### 9.1 示例总览

| 编号 | 文件 | 核心知识点 | 回调组配置 |
|------|------|-----------|-----------|
| ① | `default_behavior.py` | 默认回调组阻塞 | 默认 MutuallyExclusive |
| ② | `mutually_exclusive_demo.py` | 同组互斥、不同组并发 | 2个 MutuallyExclusive |
| ③ | `reentrant_demo.py` | 同组可并发、线程安全 | 1个 Reentrant + 1个 MutuallyExclusive |
| ④ | `executor_types.py` | 执行器类型对比 | 1个 MutuallyExclusive + 1个 Reentrant |
| ⑤ | `multi_node_executor.py` | 多节点单执行器 | 混合 |
| ⑥ | `robot_sensor_fusion.py` | 传感器融合实战 | 4个独立回调组 |
| ⑦ | `robot_nav_stack.py` | 导航栈实战 | 4个优先级回调组 |
| ⑧ | `deadlock_demo.py` | 死锁与解决 | 3种方案对比 |

### 9.2 运行方式

```bash
# 编译
colcon build --packages-select callback_group_learning
source install/setup.bash

# 方式1：直接运行节点
ros2 run callback_group_learning default_behavior

# 方式2：使用launch文件
ros2 launch callback_group_learning 01_default_behavior.launch.py

# 死锁演示
ros2 run callback_group_learning deadlock_demo  # 默认模式3
ros2 run callback_group_learning deadlock_demo 1  # 死锁模式
```

---

## 10. 机器人实际应用案例

### 10.1 案例1：多传感器融合节点

**场景**：移动机器人配备激光雷达(10Hz)、IMU(100Hz)、相机(30Hz)

**设计思路**：
- IMU 最高频，独立回调组，永不被阻塞
- 激光雷达需要保护障碍物数据，用 MutuallyExclusive
- 相机处理耗时，用 Reentrant 允许并发
- 融合回调独立，定期读取所有数据

```
┌───────────────────────────────────────────────┐
│           RobotSensorFusionNode                │
│                                               │
│  ┌── lidar_group (ME) ──┐  障碍物数据保护    │
│  │  lidar_callback       │                    │
│  └───────────────────────┘                    │
│                                               │
│  ┌── imu_group (ME) ────┐  姿态数据保护      │
│  │  imu_callback         │  100Hz 不被阻塞   │
│  └───────────────────────┘                    │
│                                               │
│  ┌── camera_group (Reentrant) ─┐  允许并发   │
│  │  camera_callback              │            │
│  └──────────────────────────────┘            │
│                                               │
│  ┌── fusion_group (ME) ─┐  融合逻辑         │
│  │  fusion_callback       │                   │
│  └───────────────────────┘                    │
│                                               │
│  MultiThreadedExecutor(4)                     │
└───────────────────────────────────────────────┘
```

### 10.2 案例2：导航栈

**场景**：速度控制(50Hz)、避障(10Hz)、路径规划(1Hz)、地图更新(0.5Hz)

**设计思路**：
- 安全关键模块（速度控制、避障）独立回调组，保证实时性
- 耗时规划用 Reentrant，允许并发
- 地图操作用 MutuallyExclusive 保护地图数据

```
优先级从高到低：

  ┌── control_group (ME, 50Hz) ──┐  ← 最高优先级
  │  速度控制 + 里程计发布          │
  └───────────────────────────────┘
  
  ┌── safety_group (ME, 10Hz) ──┐  ← 安全关键
  │  避障检测 + 紧急停止          │
  └───────────────────────────────┘
  
  ┌── planning_group (Reentrant, 1Hz) ──┐  ← 允许并发
  │  路径规划                              │
  └───────────────────────────────────────┘
  
  ┌── mapping_group (ME, 0.5Hz) ──┐  ← 后台
  │  地图更新                        │
  └──────────────────────────────────┘
```

### 10.3 案例3：机械臂控制

```python
# 机械臂控制节点的回调组设计
class RobotArmNode(Node):
    def __init__(self):
        super().__init__('robot_arm')
        
        # 关节控制 —— 最高优先级，独立回调组
        self.joint_control_group = MutuallyExclusiveCallbackGroup()
        
        # 力矩传感器订阅 —— 安全关键
        self.force_sensor_group = MutuallyExclusiveCallbackGroup()
        
        # 视觉处理 —— 耗时，允许并发
        self.vision_group = ReentrantCallbackGroup()
        
        # 抓取规划 —— 独立
        self.grasp_planning_group = MutuallyExclusiveCallbackGroup()
        
        # 状态发布 —— 独立
        self.status_group = MutuallyExclusiveCallbackGroup()
```

### 10.4 案例4：多机器人调度系统

```python
# 调度系统需要同时与多个机器人通信
class MultiRobotScheduler(Node):
    def __init__(self):
        super().__init__('scheduler')
        
        # 每个机器人的通信放在独立回调组
        # 避免一个机器人通信阻塞其他
        self.robot_a_group = MutuallyExclusiveCallbackGroup()
        self.robot_b_group = MutuallyExclusiveCallbackGroup()
        self.robot_c_group = MutuallyExclusiveCallbackGroup()
        
        # 全局调度逻辑
        self.scheduler_group = MutuallyExclusiveCallbackGroup()
        
        # 任务规划 —— 耗时，允许并发
        self.planning_group = ReentrantCallbackGroup()
```

### 10.5 案例5：SLAM 系统

```python
# SLAM 系统回调组设计
class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam')
        
        # 激光雷达回调 —— 高频，独立
        self.scan_group = MutuallyExclusiveCallbackGroup()
        
        # 里程计回调 —— 高频，独立
        self.odom_group = MutuallyExclusiveCallbackGroup()
        
        # 回环检测 —— 耗时，Reentrant
        self.loop_closure_group = ReentrantCallbackGroup()
        
        # 地图发布 —— 独立
        self.map_publish_group = MutuallyExclusiveCallbackGroup()
        
        # 地图优化 —— 耗时，Reentrant
        self.optimization_group = ReentrantCallbackGroup()
```

### 10.6 案例6：无人机飞控

```python
# 无人机飞控系统回调组设计
class DroneFlightControl(Node):
    def __init__(self):
        super().__init__('flight_control')
        
        # 姿态控制 —— 最高优先级(200Hz)
        self.attitude_control_group = MutuallyExclusiveCallbackGroup()
        
        # IMU 数据接收 —— 高频(100Hz)
        self.imu_group = MutuallyExclusiveCallbackGroup()
        
        # GPS 数据 —— 中频(10Hz)
        self.gps_group = MutuallyExclusiveCallbackGroup()
        
        # 气压计数据 —— 中频(50Hz)
        self.baro_group = MutuallyExclusiveCallbackGroup()
        
        # 路径规划 —— 低频
        self.planning_group = MutuallyExclusiveCallbackGroup()
        
        # 视觉避障 —— 耗时
        self.visual_avoidance_group = ReentrantCallbackGroup()
```

---

## 11. 高级话题

### 11.1 回调组与 QoS 的关系

QoS（Quality of Service）影响消息传递，回调组影响回调执行。两者独立但互补：

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 高频传感器：最佳努力 + 独立回调组
imu_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
imu_group = MutuallyExclusiveCallbackGroup()

self.imu_sub = self.create_subscription(
    Imu, '/imu/data', self.imu_cb, imu_qos,
    callback_group=imu_group,
)
```

### 11.2 回调组与生命周期节点

生命周期节点（LifecycleNode）的回调也受回调组控制：

```python
from rclpy_lifecycle import LifecycleNode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
        
        self.group = MutuallyExclusiveCallbackGroup()
        
        # on_configure, on_activate 等生命周期转换回调
        # 也可以指定回调组
        
        # 普通回调
        self.timer = self.create_timer(
            1.0, self.timer_cb,
            callback_group=self.group,
        )
```

### 11.3 回调组与 NodeOptions

```python
# 允许节点参数覆盖
from rclpy.parameter import Parameter

node = Node(
    'my_node',
    allow_undeclared_parameters=True,
    automatically_declare_parameters_from_overrides=True,
)

# 回调组的选择不受 NodeOptions 影响
# 但参数回调也受回调组控制
```

### 11.4 回调执行时间监控

```python
import time

class MonitoredNode(Node):
    def __init__(self):
        super().__init__('monitored_node')
        self.group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.monitored_cb, 
                                        callback_group=self.group)
        self.max_elapsed = 0.0
        self.total_elapsed = 0.0
        self.call_count = 0
    
    def monitored_cb(self):
        start = time.time()
        
        # ... 回调逻辑 ...
        time.sleep(0.01)  # 模拟处理
        
        elapsed = time.time() - start
        self.call_count += 1
        self.total_elapsed += elapsed
        self.max_elapsed = max(self.max_elapsed, elapsed)
        
        # 检测回调执行时间过长
        if elapsed > 0.05:  # 超过50ms告警
            self.get_logger().warn(
                f'回调执行耗时 {elapsed*1000:.1f}ms，超过阈值！'
            )
    
    def print_stats(self):
        if self.call_count > 0:
            avg = self.total_elapsed / self.call_count
            self.get_logger().info(
                f'回调统计: 平均={avg*1000:.1f}ms, '
                f'最大={self.max_elapsed*1000:.1f}ms, '
                f'调用次数={self.call_count}'
            )
```

### 11.5 回调组数量优化

```
回调组数量选择指南：

1个回调组 → 最简单，所有回调串行（默认）
2-3个 → 大多数场景足够
4-6个 → 复杂机器人系统
7+个 → 过度设计，考虑简化

原则：
- 共享数据的回调放在同一个互斥组
- 独立的计算用独立组
- 不要为每个回调都创建单独的组
```

### 11.6 C++ 中的回调组

```cpp
// C++ 中创建回调组
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // 创建互斥回调组
        exclusive_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // 创建可重入回调组
        reentrant_group_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        // 订阅时指定回调组
        sub_ = create_subscription<std_msgs::msg::String>(
            "/topic", 10,
            std::bind(&MyNode::callback, this, std::placeholders::_1),
            rclcpp::SubscriptionOptions()
                .callback_group(exclusive_group_));  // 指定回调组
        
        // 定时器时指定回调组
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MyNode::timer_callback, this),
            reentrant_group_);  // 指定回调组
    }

private:
    rclcpp::CallbackGroup::SharedPtr exclusive_group_;
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// 使用 MultiThreadedExecutor
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    
    // 多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 4);  // 4个线程
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
}
```

### 11.7 回调组与 Composition

在组件化节点中，回调组同样有效：

```python
from rclpy_components import NodeComponent

class MyComponent(NodeComponent):
    def __init__(self, node_options=None):
        super().__init__('my_component', node_options=node_options)
        
        # 组件中的回调组用法与普通节点完全相同
        self.group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            1.0, self.timer_cb, callback_group=self.group)
```

---

## 12. 常见问题与调试

### 12.1 常见问题表

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 回调延迟高 | 默认回调组中所有回调串行 | 为高频回调创建独立回调组 |
| 回调丢失 | 慢回调阻塞快回调 | 分离到不同回调组 |
| 死锁 | 同组内服务调用 | 服务端和调用方分到不同组 |
| 竞态条件 | Reentrant + 共享数据无锁 | 使用锁或改为 MutuallyExclusive |
| Reentrant 不并发 | 使用了 SingleThreadedExecutor | 换用 MultiThreadedExecutor |
| 服务响应慢 | 服务回调被同组回调阻塞 | 服务端独立回调组 |

### 12.2 调试方法

#### 方法1：线程ID打印

```python
import threading

def my_callback(self, msg):
    thread_id = threading.current_thread().name
    self.get_logger().info(f'线程: {thread_id}, 数据: {msg.data}')
```

如果不同回调打印相同的线程ID → 说明没有真正并发（SingleThreadedExecutor 或同组）。

#### 方法2：时间戳分析

```python
import time

def callback_a(self):
    t0 = time.time()
    self.get_logger().info(f'[A] 开始 t={t0:.3f}')
    time.sleep(0.5)
    self.get_logger().info(f'[A] 结束 t={time.time():.3f}')

def callback_b(self):
    self.get_logger().info(f'[B] 开始 t={time.time():.3f}')
```

如果 A 开始后 B 立即开始 → 并发（不同回调组）
如果 A 结束后 B 才开始 → 串行（同一组或单线程）

#### 方法3：回调组检查

```python
# 列出节点所有回调组
for group in node.callback_groups:
    print(f'回调组: {type(group).__name__}')
```

#### 方法4：执行器信息

```python
executor = MultiThreadedExecutor(num_threads=4)
print(f'执行器线程数: {executor._num_threads}')
```

### 12.3 性能优化建议

1. **选择合适的执行器**：不需要并发就用 SingleThreaded
2. **减少回调组数量**：过多回调组增加调度开销
3. **回调内避免阻塞**：使用 `call_async` 而非 `call`
4. **合理设置 QoS depth**：避免消息堆积
5. **避免在回调中 sleep**：使用定时器替代

---

## 13. 快速参考

### 13.1 回调组速查

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# 创建回调组
exclusive_group = MutuallyExclusiveCallbackGroup()  # 互斥（同组串行）
reentrant_group = ReentrantCallbackGroup()           # 可重入（同组可并发）

# 绑定回调到回调组
timer = node.create_timer(1.0, callback, callback_group=exclusive_group)
sub = node.create_subscription(Msg, '/topic', callback, 10, callback_group=reentrant_group)
srv = node.create_service(SrvType, '/service', callback, callback_group=exclusive_group)
```

### 13.2 执行器速查

```python
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

# 单线程（默认）
rclpy.spin(node)

# 多线程
executor = MultiThreadedExecutor(num_threads=4)
rclpy.spin(node, executor=executor)

# 手动循环
executor = MultiThreadedExecutor()
executor.add_node(node)
while rclpy.ok():
    executor.spin_once(timeout_sec=0.1)

# 多节点
executor.add_node(node_a)
executor.add_node(node_b)
rclpy.spin_once(timeout_sec=0.1, executor=executor)
```

### 13.3 设计模式速查

```
┌─────────────────────────────────────────────────┐
│              选择回调组的决策流程                 │
│                                                 │
│  开始                                            │
│   │                                              │
│   ├── 只有一个回调？ → 默认回调组即可             │
│   │                                              │
│   ├── 有耗时回调阻塞紧急回调？                    │
│   │   └── 是 → 独立回调组                        │
│   │                                              │
│   ├── 回调间有共享数据？                          │
│   │   ├── 是 → 同一 MutuallyExclusive            │
│   │   └── 否 → Reentrant（如需并发）               │
│   │                                              │
│   ├── 回调中有服务调用？                          │
│   │   └── 服务端和调用方分到不同回调组            │
│   │                                              │
│   └── 需要同一回调并发执行？                      │
│       └── Reentrant + 锁保护                     │
└─────────────────────────────────────────────────┘
```

### 13.4 API 速查

```python
# === 回调组 ===
node.default_callback_group                    # 默认回调组（MutuallyExclusive）
node.create_callback_group(GroupType)         # C++ 中创建回调组
MutuallyExclusiveCallbackGroup()               # Python 创建互斥组
ReentrantCallbackGroup()                       # Python 创建可重入组

# === 回调绑定 ===
node.create_timer(period, callback, callback_group=group)
node.create_subscription(msg_type, topic, callback, qos, callback_group=group)
node.create_service(srv_type, name, callback, callback_group=group)
node.create_client(srv_type, name, callback_group=group)  # 客户端也有回调组

# === 执行器 ===
SingleThreadedExecutor()
MultiThreadedExecutor(num_threads=N)
executor.add_node(node)
executor.remove_node(node)
executor.spin()
executor.spin_once(timeout_sec=T)
executor.spin_until_future_complete(future)
rclpy.spin(node, executor=executor)

# === 线程安全 ===
import threading
lock = threading.Lock()
with lock:
    # 临界区
    pass

# === 调试 ===
import threading
threading.current_thread().name   # 获取当前线程名
```

---

## 附录：参考资料

- [ROS2 官方文档 - Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
- [ROS2 官方文档 - Callback Groups](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Callback-Groups.html)
- [rclpy API 文档](https://docs.ros2.org/latest/api/rclpy/)
- [ROS2 Executor 论文](https://design.ros2.org/articles/executor.html)
