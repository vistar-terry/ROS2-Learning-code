/**
 * @file static_executor.cpp
 * @brief ③ 静态单线程执行器 —— 零分配 + 确定性实时执行
 *
 * 本示例用同一份代码实证 StaticSingleThreadedExecutor 的四大特性及体现方式：
 *
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │ 1) 零分配（zero allocation）                                        │
 *  │    - 首次 spin 时一次性把已注册节点的所有实体（定时器 / 订阅 /       │
 *  │      服务 / 客户端 / waitable）登记进底层 wait set；                 │
 *  │    - 之后的循环【不再重建 wait set、不再为实体分配内存】，           │
 *  │      区别于 SingleThreadedExecutor 每轮都重新收集（见示例④）。      │
 *  │    - 体现：100Hz 控制循环长时间运行，内存占用稳定、调度开销恒定。    │
 *  │                                                                      │
 *  │ 2) 确定性 / 低抖动（deterministic, low jitter）                      │
 *  │    - 实体集合固定 + 单线程严格串行 → 延迟抖动稳定且最小。            │
 *  │    - 体现：实时测量定时器触发间隔相对期望(10ms)的抖动，并统计        │
 *  │      avg / max。                                                     │
 *  │    - 注意：在通用（非实时）操作系统上只能做到"相对最优"，           │
 *  │      真正的硬实时需要 RT 内核 + 线程优先级调度。                     │
 *  │                                                                      │
 *  │ 3) 严格串行、无数据竞争（serialized, data-race free）                │
 *  │    - 单个静态执行器线程内，所有回调（定时器 / 订阅）严格串行执行，   │
 *  │      天然没有并发，因此共享状态无需加锁。                            │
 *  │    - 体现：控制定时器与订阅回调共享 latest_sensor_value_，           │
 *  │      全程【故意不加锁】也安全（换成 MultiThreadedExecutor 就必须加   │
 *  │      std::mutex / std::atomic）。                                    │
 *  │                                                                      │
 *  │ 4) 限制：不支持运行时动态增删节点（no dynamic add/remove）          │
 *  │    - 实体集合在首次 spin 时固定；spin 之后再 add_node 的新节点       │
 *  │      不会被纳入调度（或至少不被保证）。                              │
 *  │    - 体现：演示中在 spin 启动 3 秒后追加一个"迟到节点"，            │
 *  │      观察其定时器计数器始终为 0，尽管它已出现在 get_all_nodes() 中。  │
 *  │    - 正确用法：所有节点必须在 spin 之前 add_node。                   │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 * 运行：
 *   ros2 run executor_learning_cpp static_executor
 * 或用 launch：
 *   ros2 launch executor_learning_cpp 03_static_executor.launch.py
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <atomic>
#include <thread>

using namespace std::chrono_literals;

// ================================================================
// 主控制节点 —— 体现 零分配 / 确定性 / 严格串行
// ================================================================
class RealtimeControlNode : public rclcpp::Node
{
public:
  RealtimeControlNode()
  : Node("realtime_control")
  {
    // ---- 共享状态（用于演示"严格串行 → 无需加锁"）----
    // subscription 回调写入 latest_sensor_value_，control 定时器回调读取它。
    // 二者在同一静态执行器内严格串行 → 这里【故意不加锁】也线程安全。
    // （若改用 MultiThreadedExecutor，则必须加 std::mutex / std::atomic）

    // 100Hz 控制定时器 —— 实时性核心
    control_timer_ = this->create_wall_timer(
      10ms,
      [this]()
      {
        double jitter = measure_jitter();   // 测量相对期望间隔的抖动
        record_jitter(jitter);             // 累计统计

        // 读取订阅回调写入的共享状态（无锁，安全）
        double sensor = latest_sensor_value_.load();

        loop_count_++;
        if (loop_count_ % 100 == 0)
        {
          RCLCPP_INFO(this->get_logger(),
                      "[Control 100Hz] #%d  sensor=%.3f  jitter(last)=%.3fms",
                      loop_count_, sensor, jitter);
        }
      });

    // 10Hz 状态监测
    monitor_timer_ = this->create_wall_timer(
      100ms,
      [this]()
      {
        double avg = (jitter_cnt_ > 0)
          ? (jitter_sum_ / static_cast<double>(jitter_cnt_))
          : 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "[Monitor 10Hz] loops=%d | jitter(ms): avg=%.3f max=%.3f (n=%ld)",
                    loop_count_, avg, jitter_max_, jitter_cnt_);
      });

    // 订阅 /sensor_data —— 与定时器共享状态，演示"串行无竞争"
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/sensor_data", 10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        // 写入共享状态（无锁）
        latest_sensor_value_ = std::strtod(msg->data.c_str(), nullptr);
        RCLCPP_INFO(this->get_logger(),
                    "sub /sensor_data: %s  (与 Control 共享状态，无需加锁)",
                    msg->data.c_str());
      });

    // 自刺激发布器：周期性发布，使上面的订阅回调真正被触发，
    // 以此证明"同一静态执行器内多种实体（定时器 + 订阅）被确定性调度"。
    pub_ = this->create_publisher<std_msgs::msg::String>("/sensor_data", 10);
    pub_timer_ = this->create_wall_timer(
      33ms,   // ~30Hz 模拟传感器采样
      [this]()
      {
        auto msg = std_msgs::msg::String();
        double t = std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - t0_).count();
        double v = 20.0 + 5.0 * std::sin(t);   // 一个平滑变化的"传感器读数"
        msg.data = std::to_string(v);
        pub_->publish(msg);
      });

    RCLCPP_INFO(this->get_logger(),
                "=== RealtimeControlNode ready (StaticSingleThreadedExecutor) ===");
  }

private:
  // 测量定时器触发间隔相对期望(10ms)的抖动（首次测量基准不定，返回 NaN 跳过）
  double measure_jitter()
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(now - last_time_).count();
    last_time_ = now;
    if (!first_done_)
    {
      first_done_ = true;
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::abs(elapsed - 10.0);
  }

  void record_jitter(double j)
  {
    if (std::isnan(j)) return;           // 跳过无效首值
    jitter_sum_ += j;
    jitter_cnt_ += 1;
    if (j > jitter_max_) jitter_max_ = j;
  }

  // ---- 以下成员仅在"同一静态执行器线程"内被回调访问（串行）----
  // 因此即便不是 atomic 也安全——这正是特性 3 的佐证。
  bool   first_done_{false};
  std::chrono::steady_clock::time_point last_time_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point t0_{std::chrono::steady_clock::now()};

  int    loop_count_{0};
  double jitter_sum_{0.0};
  long   jitter_cnt_{0};
  double jitter_max_{0.0};

  // 跨"定时器↔订阅"共享、无锁访问的状态（串行保证安全）
  std::atomic<double> latest_sensor_value_{0.0};

  rclcpp::TimerBase::SharedPtr                       control_timer_;
  rclcpp::TimerBase::SharedPtr                       monitor_timer_;
  rclcpp::TimerBase::SharedPtr                       pub_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   pub_;
};

// ================================================================
// 迟到节点 —— 体现 限制：spin 之后 add_node 不会被调度
// ================================================================
class LateNode : public rclcpp::Node
{
public:
  LateNode()
  : Node("late_node")
  {
    late_timer_ = this->create_wall_timer(
      200ms,
      [this]()
      {
        late_fires_++;   // 跨线程计数，用 atomic
        RCLCPP_INFO(this->get_logger(),
                    "[LateNode] timer fired #%d  <-- 若你看到此行，说明本 rclcpp 版本在 spin 后仍会重新收集实体",
                    late_fires_.load());
      });
  }

  int fires() const { return late_fires_.load(); }

private:
  std::atomic<int> late_fires_{0};
  rclcpp::TimerBase::SharedPtr late_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);   // 初始化 ROS2

  auto node = std::make_shared<RealtimeControlNode>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;  // 零分配静态执行器
  executor.add_node(node);   // ✅ 正确：spin 之前注册所有节点

  RCLCPP_INFO(node->get_logger(), "=== Using StaticSingleThreadedExecutor ===");
  RCLCPP_INFO(node->get_logger(), "Features: zero allocation, deterministic, serial(no-lock)");
  RCLCPP_INFO(node->get_logger(), "Limitation: runtime add/remove node NOT supported");

  // 在独立线程中驱动执行器，以便主线程模拟"运行时"场景
  std::thread spin_thread([&executor]() { executor.spin(); });

  // ---- 阶段一：让静态执行器先稳定跑 3 秒（体现零分配/确定性）----
  std::this_thread::sleep_for(3s);

  // ---- 阶段二：❌ 错误示范 —— spin 之后再 add_node ----
  auto late = std::make_shared<LateNode>();
  executor.add_node(late);    // 实体集合已在首次 spin 时固定，新节点通常不进入调度
  RCLCPP_INFO(node->get_logger(),
              "已 add_node(late) —— 但静态执行器在首次 spin 时即固定实体集合，该节点很可能不会被调度");
  RCLCPP_INFO(node->get_logger(),
              "get_all_nodes() 节点数 = %zu（已登记到节点表 ≠ 已被调度）",
              executor.get_all_nodes().size());

  // ---- 阶段三：再观察 3 秒，验证 late 节点的定时器是否真的触发 ----
  std::this_thread::sleep_for(3s);
  RCLCPP_INFO(node->get_logger(),
              "late 节点定时器实际触发次数 = %d  "
              "（0 ⇒ 证明静态执行器忽略 spin 后新增的节点；"
              ">0 ⇒ 当前 rclcpp 版本会在每次 spin 重新收集实体，"
              "但官方推荐/确定性保证的用法仍是 spin 前注册全部节点）",
              late->fires());

  executor.cancel();          // 优雅取消 spin
  spin_thread.join();
  rclcpp::shutdown();         // 清理 ROS2 资源
  return 0;
}
