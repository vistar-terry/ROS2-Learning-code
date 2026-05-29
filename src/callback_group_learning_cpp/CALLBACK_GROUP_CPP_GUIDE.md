# ROS2 回调组 C++ 完全指南

> 本文档是 `callback_group_learning_cpp` 功能包的配套指南，重点讲解 **C++ 与 Python 的 API 差异**和C++特有的使用模式。完整的回调组原理请参考 [Python版文档](../callback_group_learning/CALLBACK_GROUP_GUIDE.md)。

---

## 目录

1. [C++ vs Python API 对照表](#1-c-vs-python-api-对照表)
2. [C++ 回调组创建与绑定](#2-c-回调组创建与绑定)
3. [C++ 执行器使用](#3-c-执行器使用)
4. [C++ 线程安全](#4-c-线程安全)
5. [C++ 特有注意事项](#5-c-特有注意事项)
6. [示例代码详解](#6-示例代码详解)
7. [快速参考](#7-快速参考)

---

## 1. C++ vs Python API 对照表

### 1.1 回调组创建

| 操作 | Python | C++ |
|------|--------|-----|
| 创建互斥组 | `MutuallyExclusiveCallbackGroup()` | `node->create_callback_group(CallbackGroupType::MutuallyExclusive)` |
| 创建可重入组 | `ReentrantCallbackGroup()` | `node->create_callback_group(CallbackGroupType::Reentrant)` |
| 获取默认组 | `node.default_callback_group` | `node->get_default_callback_group()` |

### 1.2 回调绑定

| 回调类型 | Python | C++ |
|---------|--------|-----|
| 定时器 | `create_timer(period, cb, callback_group=group)` | `create_wall_timer(period, std::bind(...), group)` |
| 订阅 | `create_subscription(Type, topic, cb, qos, callback_group=group)` | `create_subscription<Type>(topic, qos, std::bind(...), SubscriptionOptions{.callback_group=group})` |
| 服务端 | `create_service(Type, name, cb, callback_group=group)` | `create_service<Type>(name, std::bind(...), qos_profile, group)` |
| 客户端 | `create_client(Type, name, callback_group=group)` | `create_client<Type>(name, qos_profile, group)` |

### 1.3 执行器

| 操作 | Python | C++ |
|------|--------|-----|
| 单线程 | `SingleThreadedExecutor()` | `rclcpp::executors::SingleThreadedExecutor` |
| 多线程 | `MultiThreadedExecutor(num_threads=N)` | `rclcpp::executors::MultiThreadedExecutor(ExecutorOptions(), N)` |
| 注册节点 | `executor.add_node(node)` | `executor.add_node(node)` |
| 运行 | `executor.spin()` | `executor.spin()` |
| 单步 | `executor.spin_once(timeout_sec=T)` | `executor.spin_once(timeout)` |
| 等待Future | `rclpy.spin_until_future_complete(node, future)` | `executor.spin_until_future_complete(future)` |

### 1.4 线程安全

| 操作 | Python | C++ |
|------|--------|-----|
| 互斥锁 | `threading.Lock()` | `std::mutex` |
| 加锁 | `with lock: ...` | `std::lock_guard<std::mutex> lock(mutex_);` |
| 线程ID | `threading.current_thread().name` | `std::this_thread::get_id()` |
| 原子变量 | 无内置 | `std::atomic<T>` |

---

## 2. C++ 回调组创建与绑定

### 2.1 创建回调组

```cpp
// 创建互斥回调组
auto exclusive_group = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

// 创建可重入回调组
auto reentrant_group = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

// 获取默认回调组
auto default_group = this->get_default_callback_group();
```

### 2.2 定时器绑定回调组

```cpp
// ✅ C++ 中 create_wall_timer 的最后一个参数就是 callback_group
auto timer = this->create_wall_timer(
    100ms,                                    // 周期
    std::bind(&MyNode::timer_cb, this),       // 回调
    my_callback_group                         // 回调组（可选）
);
```

### 2.3 订阅绑定回调组

```cpp
// ✅ C++ 中订阅需要通过 SubscriptionOptions 指定回调组
rclcpp::SubscriptionOptions options;
options.callback_group = my_callback_group;   // 指定回调组

auto sub = this->create_subscription<std_msgs::msg::String>(
    "/topic",                                  // 话题名
    10,                                        // QoS depth
    std::bind(&MyNode::msg_cb, this, std::placeholders::_1),  // 回调
    options                                    // 选项（含回调组）
);
```

### 2.4 服务端绑定回调组

```cpp
// ✅ C++ 中 create_service 的最后一个参数是 callback_group
auto srv = this->create_service<example_interfaces::srv::Trigger>(
    "/my_service",                             // 服务名
    std::bind(&MyNode::srv_cb, this,
              std::placeholders::_1, std::placeholders::_2),  // 回调
    rclcpp::ServicesQoS(),                     // QoS（Jazzy 推荐用法）
    my_callback_group                          // 回调组
);
```

### 2.5 客户端绑定回调组

```cpp
// ✅ C++ 中 create_client 的最后一个参数是 callback_group
auto client = this->create_client<example_interfaces::srv::Trigger>(
    "/my_service",                             // 服务名
    rclcpp::ServicesQoS(),                     // QoS（Jazzy 推荐用法）
    my_callback_group                          // 回调组
);
```

---

## 3. C++ 执行器使用

### 3.1 SingleThreadedExecutor

```cpp
// 方式1：rclcpp::spin（默认 SingleThreadedExecutor）
rclcpp::spin(node);

// 方式2：显式创建
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

### 3.2 MultiThreadedExecutor

```cpp
// 创建多线程执行器（4个线程）
rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);

executor.add_node(node);
executor.spin();
```

### 3.3 手动执行循环

```cpp
rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
executor.add_node(node);

while (rclcpp::ok()) {
    // 执行一批就绪回调
    executor.spin_once(std::chrono::milliseconds(100));

    // 可以在这里插入自定义逻辑
    custom_processing();
}
```

### 3.4 多节点执行器

```cpp
auto node_a = std::make_shared<NodeA>();
auto node_b = std::make_shared<NodeB>();

rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
executor.add_node(node_a);
executor.add_node(node_b);
executor.spin();
```

### 3.5 等待 Future

```cpp
auto future = client->async_send_request(request);

// 方式1：使用执行器等待
executor.spin_until_future_complete(future);

// 方式2：使用 std::future 的 wait_for
auto status = future.wait_for(5s);
if (status == std::future_status::ready) {
    auto result = future.get();
}
```

---

## 4. C++ 线程安全

### 4.1 std::mutex（互斥锁）

```cpp
#include <mutex>

class MyNode : public rclcpp::Node {
    std::mutex mutex_;
    int shared_data_ = 0;

    void callback_a() {
        // lock_guard 在作用域结束时自动释放锁
        std::lock_guard<std::mutex> lock(mutex_);
        shared_data_++;  // 安全
    }

    void callback_b() {
        std::lock_guard<std::mutex> lock(mutex_);
        shared_data_ += 10;  // 安全
    }
};
```

### 4.2 std::atomic（原子变量）

```cpp
#include <atomic>

class MyNode : public rclcpp::Node {
    std::atomic<int> counter_{0};  // 原子变量，无需加锁

    void callback() {
        counter_++;           // 原子操作，线程安全
        int val = counter_.load();  // 原子读取
    }
};
```

### 4.3 unique_lock（灵活锁）

```cpp
#include <mutex>

void callback() {
    std::unique_lock<std::mutex> lock(mutex_);

    // ... 操作共享数据 ...

    // 可以提前释放锁
    lock.unlock();

    // 不涉及共享数据的操作（不需要锁）

    // 也可以重新加锁
    lock.lock();
}
```

### 4.4 condition_variable（条件变量）

```cpp
#include <mutex>
#include <condition_variable>

std::mutex mtx;
std::condition_variable cv;
bool data_ready = false;

// 等待端
void wait_for_data() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [] { return data_ready; });
    // 处理数据
}

// 通知端
void set_data_ready() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        data_ready = true;
    }
    cv.notify_one();
}
```

### 4.5 C++ vs Python 线程安全对照

| 需求 | Python | C++ |
|------|--------|-----|
| 互斥保护 | `with threading.Lock():` | `std::lock_guard<std::mutex>` |
| 原子计数 | `threading.Atomic`（无内置） | `std::atomic<int>` |
| 等待通知 | `threading.Event` / `Condition` | `std::condition_variable` |
| 线程安全队列 | `queue.Queue` | `std::queue` + `std::mutex` |
| 读写锁 | 无内置 | `std::shared_mutex` (C++17) |

---

## 5. C++ 特有注意事项

### 5.1 回调组必须用 SharedPtr 保持生命周期

```cpp
class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // ❌ 错误：回调组作为局部变量，会被销毁
        // auto group = this->create_callback_group(...);
        // auto timer = this->create_wall_timer(1s, cb, group);
        // group 在构造函数结束后仍然有效（因为被 timer 内部引用）

        // ✅ 正确：回调组作为成员变量，确保生命周期
        my_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_ = this->create_wall_timer(1s, std::bind(&MyNode::cb, this), my_group_);
    }

private:
    rclcpp::CallbackGroup::SharedPtr my_group_;  // 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### 5.2 std::bind 的占位符

```cpp
// 订阅回调（1个参数）
std::bind(&MyNode::sub_cb, this, std::placeholders::_1)

// 服务回调（2个参数）
std::bind(&MyNode::srv_cb, this, std::placeholders::_1, std::placeholders::_2)
```

### 5.3 Lambda 替代 std::bind

```cpp
// 使用 Lambda 替代 std::bind（更现代、更可读）
auto timer = this->create_wall_timer(1s,
    [this]() { this->timer_callback(); },
    my_group);

auto sub = this->create_subscription<std_msgs::msg::String>(
    "/topic", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        this->msg_callback(msg);
    },
    options);
```

### 5.4 create_wall_timer vs create_timer

```cpp
// create_wall_timer：使用系统墙钟时间，不受 ROS 时间仿真影响
auto timer1 = this->create_wall_timer(100ms, cb, group);

// create_timer：使用 ROS 时间，受 /clock 话题影响（仿真场景）
auto timer2 = this->create_timer(100ms, cb, group);
```

### 5.5 服务回调的线程模型

```cpp
// C++ 中服务回调在执行器的线程中执行
// 如果服务端在 MutuallyExclusive 回调组中，
// 同时只有一个线程能执行该回调

// 如果服务端在 Reentrant 回调组中，
// 多个服务请求可以并发处理（需要线程安全！）
```

### 5.6 executor.spin() 与信号处理

```cpp
// rclcpp::spin() 和 executor.spin() 会自动处理 SIGINT (Ctrl+C)
// 但如果使用手动循环，需要检查 rclcpp::ok()

while (rclcpp::ok()) {
    executor.spin_once(100ms);
}
```

---

## 6. 示例代码详解

### 6.1 示例对照表

| 编号 | C++ 文件 | Python 对应 | C++ 特有知识点 |
|------|---------|------------|---------------|
| ① | `default_behavior.cpp` | `default_behavior.py` | `get_default_callback_group()` |
| ② | `mutually_exclusive_demo.cpp` | `mutually_exclusive_demo.py` | `SubscriptionOptions.callback_group`, `std::ostringstream` 获取线程ID |
| ③ | `reentrant_demo.cpp` | `reentrant_demo.py` | `std::mutex` + `std::lock_guard`, `std::atomic` |
| ④ | `executor_types.cpp` | `executor_types.py` | `SingleThreadedExecutor`, `MultiThreadedExecutor` C++ API |
| ⑤ | `multi_node_executor.cpp` | `multi_node_executor.py` | C++ 多节点执行器 |
| ⑥ | `robot_sensor_fusion.cpp` | `robot_sensor_fusion.py` | `std::mutex` 跨回调组保护, 结构体 |
| ⑦ | `robot_nav_stack.cpp` | `robot_nav_stack.py` | C++ 导航消息类型, `std::lock_guard` |
| ⑧ | `deadlock_demo.cpp` | `deadlock_demo.py` | `async_send_request()`, 命令行参数 |

### 6.2 运行方式

```bash
# 编译
colcon build --packages-select callback_group_learning_cpp
source install/setup.bash

# 运行C++示例
ros2 run callback_group_learning_cpp default_behavior
ros2 run callback_group_learning_cpp mutually_exclusive_demo
ros2 run callback_group_learning_cpp reentrant_demo
ros2 run callback_group_learning_cpp executor_types
ros2 run callback_group_learning_cpp multi_node_executor
ros2 run callback_group_learning_cpp robot_sensor_fusion
ros2 run callback_group_learning_cpp robot_nav_stack
ros2 run callback_group_learning_cpp deadlock_demo        # 默认模式3
ros2 run callback_group_learning_cpp deadlock_demo 1      # 死锁模式

# 使用launch
ros2 launch callback_group_learning_cpp 01_default_behavior.launch.py
ros2 launch callback_group_learning_cpp 08_deadlock_demo.launch.py mode:=1
```

---

## 7. 快速参考

### 7.1 C++ 回调组速查

```cpp
// === 创建回调组 ===
auto exclusive_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
auto reentrant_group = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

// === 定时器绑定 ===
auto timer = node->create_wall_timer(100ms, callback, group);

// === 订阅绑定 ===
rclcpp::SubscriptionOptions opts;
opts.callback_group = group;
auto sub = node->create_subscription<MsgType>("/topic", 10, callback, opts);

// === 服务端绑定 ===
auto srv = node->create_service<SrvType>("/service", callback, qos_profile, group);

// === 客户端绑定 ===
auto client = node->create_client<SrvType>("/service", qos_profile, group);
```

### 7.2 C++ 执行器速查

```cpp
// 单线程
rclcpp::spin(node);

// 多线程
rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
executor.add_node(node);
executor.spin();

// 手动循环
while (rclcpp::ok()) {
    executor.spin_once(100ms);
}

// 多节点
executor.add_node(node_a);
executor.add_node(node_b);
executor.spin();
```

### 7.3 C++ 线程安全速查

```cpp
// 互斥锁
std::mutex mtx;
{
    std::lock_guard<std::mutex> lock(mtx);
    // 临界区
}

// 原子变量
std::atomic<int> counter{0};
counter++;
counter.load();

// 条件变量
std::condition_variable cv;
std::unique_lock<std::mutex> lock(mtx);
cv.wait(lock, []{ return ready; });
cv.notify_one();
```

### 7.4 Python ↔ C++ 迁移检查清单

从 Python 迁移到 C++ 时需要关注的差异：

- [ ] `MutuallyExclusiveCallbackGroup()` → `create_callback_group(CallbackGroupType::MutuallyExclusive)`
- [ ] `ReentrantCallbackGroup()` → `create_callback_group(CallbackGroupType::Reentrant)`
- [ ] 订阅的 `callback_group=` 参数 → `SubscriptionOptions.callback_group`
- [ ] 服务的 `callback_group=` 参数 → 构造函数最后一个参数
- [ ] `threading.Lock()` → `std::mutex` + `std::lock_guard`
- [ ] `threading.current_thread().name` → `std::this_thread::get_id()`
- [ ] `rclpy.spin(node, executor=exec)` → `executor.add_node(node); executor.spin()`
- [ ] 回调函数用 `std::bind` 或 Lambda 替代直接引用
- [ ] 回调组作为类成员变量保持生命周期（`CallbackGroup::SharedPtr`）
