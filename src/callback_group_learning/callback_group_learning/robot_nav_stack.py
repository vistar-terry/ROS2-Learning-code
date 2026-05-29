#!/usr/bin/env python3
"""
=============================================================================
 ⑦ 机器人导航栈 —— 实际应用案例
=============================================================================

场景描述：
  移动机器人导航系统包含多个功能模块：
  - 路径规划（低频，耗时计算）
  - 速度控制（高频，实时性要求高）
  - 地图更新（低频，耗时不紧急）
  - 避障检测（高频，紧急）
  - 里程计发布（高频）

回调组设计策略（优先级驱动）：
  ┌──────────────────┬─────────────┬──────────┬─────────────────────────┐
  │ 功能模块         │ 频率        │ 回调组   │ 原因                    │
  ├──────────────────┼─────────────┼──────────┼─────────────────────────┤
  │ 速度控制/里程计  │ 50Hz        │ 独立ME   │ 最高实时性，不能被阻塞  │
  │ 避障检测         │ 10Hz        │ 独立ME   │ 安全关键，不能被阻塞    │
  │ 路径规划         │ 1Hz         │ Reentrant│ 耗时，允许并发          │
  │ 地图更新         │ 0.5Hz       │ 独立ME   │ 耗时但低频，保护地图数据│
  └──────────────────┴─────────────┴──────────┴─────────────────────────┘

  ME = MutuallyExclusiveCallbackGroup

关键设计思路：
  1. 安全关键模块（速度控制、避障）放在独立回调组，保证不被阻塞
  2. 耗时计算（路径规划）用 Reentrant 允许并发
  3. 地图操作用 MutuallyExclusive 保护共享地图数据
  4. 使用 MultiThreadedExecutor 让所有回调组充分利用多核
=============================================================================
"""

import threading
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped, Odometry
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan


class RobotNavStackNode(Node):
    """机器人导航栈节点 —— 回调组优先级设计"""

    def __init__(self):
        super().__init__('robot_nav_stack')

        # ============================================================
        # 1. 回调组设计（按优先级）
        # ============================================================

        # 最高优先级：速度控制 + 里程计 (50Hz)
        # 独立回调组，确保永远不会被其他回调阻塞
        self.control_group = MutuallyExclusiveCallbackGroup()

        # 高优先级：避障检测 (10Hz)
        # 独立回调组，安全关键模块
        self.safety_group = MutuallyExclusiveCallbackGroup()

        # 低优先级：路径规划 (1Hz)
        # Reentrant：允许新规划请求到来时，旧规划仍在执行
        self.planning_group = ReentrantCallbackGroup()

        # 后台优先级：地图更新 (0.5Hz)
        # 独立 MutuallyExclusive：保护共享地图数据
        self.mapping_group = MutuallyExclusiveCallbackGroup()

        # ============================================================
        # 2. 发布者
        # ============================================================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # ============================================================
        # 3. 订阅者
        # ============================================================
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=self.safety_group,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10,
            callback_group=self.planning_group,
        )

        # ============================================================
        # 4. 定时器
        # ============================================================

        # 速度控制定时器 —— 50Hz，最高优先级
        self.control_timer = self.create_timer(
            0.02, self.control_callback,
            callback_group=self.control_group,
        )

        # 避障定时器 —— 10Hz
        self.safety_timer = self.create_timer(
            0.1, self.safety_callback,
            callback_group=self.safety_group,
        )

        # 路径规划定时器 —— 1Hz
        self.planning_timer = self.create_timer(
            1.0, self.planning_callback,
            callback_group=self.planning_group,
        )

        # 地图更新定时器 —— 0.5Hz
        self.mapping_timer = self.create_timer(
            2.0, self.mapping_callback,
            callback_group=self.mapping_group,
        )

        # ============================================================
        # 5. 状态变量
        # ============================================================
        self.lock = threading.Lock()

        # 速度控制状态
        self.current_speed = 0.0
        self.target_speed = 0.3
        self.angular_speed = 0.0

        # 避障状态
        self.min_obstacle_dist = float('inf')
        self.emergency_stop = False

        # 路径规划状态
        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.planning_in_progress = False

        # 里程计
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # 统计
        self.control_count = 0
        self.safety_count = 0
        self.planning_count = 0
        self.mapping_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('机器人导航栈启动')
        self.get_logger().info('  速度控制 → control_group (50Hz, MutuallyExclusive)')
        self.get_logger().info('  避障检测 → safety_group (10Hz, MutuallyExclusive)')
        self.get_logger().info('  路径规划 → planning_group (1Hz, Reentrant)')
        self.get_logger().info('  地图更新 → mapping_group (0.5Hz, MutuallyExclusive)')
        self.get_logger().info('=' * 60)

    # ================================================================
    # 最高优先级：速度控制 + 里程计
    # ================================================================

    def control_callback(self):
        """速度控制回调 —— 50Hz，在 control_group 中
        这是最高优先级的回调，独立回调组确保不被阻塞
        """
        self.control_count += 1

        # 更新里程计（简单积分）
        dt = 0.02  # 50Hz
        self.odom_x += self.current_speed * math.cos(self.odom_theta) * dt
        self.odom_y += self.current_speed * math.sin(self.odom_theta) * dt
        self.odom_theta += self.angular_speed * dt

        # 发布里程计
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        self.odom_pub.publish(odom)

        # 速度控制：平滑加减速
        if self.emergency_stop:
            self.current_speed = 0.0
            self.angular_speed = 0.0
        else:
            speed_diff = self.target_speed - self.current_speed
            self.current_speed += max(-0.1, min(0.1, speed_diff))  # 限幅

        # 发布速度指令
        cmd = Twist()
        cmd.linear.x = self.current_speed
        cmd.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(cmd)

    # ================================================================
    # 高优先级：避障检测
    # ================================================================

    def scan_callback(self, msg: LaserScan):
        """激光雷达回调 —— 在 safety_group 中"""
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            with self.lock:
                self.min_obstacle_dist = min(valid)

    def safety_callback(self):
        """避障回调 —— 10Hz，在 safety_group 中
        安全关键回调，独立组确保及时响应
        """
        self.safety_count += 1

        with self.lock:
            min_dist = self.min_obstacle_dist

        if min_dist < 0.3:
            self.emergency_stop = True
            self.get_logger().error(
                f'[避障] 紧急停止！障碍物距离: {min_dist:.2f}m'
            )
        elif min_dist < 0.5:
            self.target_speed = 0.1
            self.emergency_stop = False
            self.get_logger().warn(
                f'[避障] 减速！障碍物距离: {min_dist:.2f}m'
            )
        else:
            self.emergency_stop = False
            self.target_speed = 0.3

    # ================================================================
    # 低优先级：路径规划
    # ================================================================

    def goal_callback(self, msg: PoseStamped):
        """目标点回调 —— 在 planning_group (Reentrant) 中"""
        self.has_goal = True
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(
            f'[规划] 收到新目标: ({self.goal_x:.1f}, {self.goal_y:.1f})'
        )

    def planning_callback(self):
        """路径规划回调 —— 1Hz，在 planning_group (Reentrant) 中
        使用 Reentrant 允许并发的场景：
        如果规划算法耗时超过1秒，下一个规划周期到来时
        可以启动新的规划，而旧的规划仍在执行
        """
        self.planning_count += 1
        if not self.has_goal:
            return

        self.planning_in_progress = True
        self.get_logger().info('[规划] 开始路径规划...')

        # 模拟耗时规划算法
        time.sleep(0.5)

        # 发布规划路径
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        # 简单直线规划
        for i in range(10):
            pose = PoseStamped()
            pose.header = path.header
            t = i / 10.0
            pose.pose.position.x = self.odom_x + t * (self.goal_x - self.odom_x)
            pose.pose.position.y = self.odom_y + t * (self.goal_y - self.odom_y)
            path.poses.append(pose)

        self.path_pub.publish(path)
        self.planning_in_progress = False
        self.get_logger().info(f'[规划] 完成，路径 {len(path.poses)} 个点')

    # ================================================================
    # 后台优先级：地图更新
    # ================================================================

    def mapping_callback(self):
        """地图更新回调 —— 0.5Hz，在 mapping_group 中
        MutuallyExclusive 保护共享地图数据
        低频但耗时，放在独立组避免影响其他模块
        """
        self.mapping_count += 1

        # 模拟地图更新
        time.sleep(0.2)

        # 发布简单地图
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = 0.05
        map_msg.info.width = 100
        map_msg.info.height = 100
        map_msg.info.origin.position.x = -2.5
        map_msg.info.origin.position.y = -2.5
        map_msg.data = [0] * (100 * 100)  # 空地图
        self.map_pub.publish(map_msg)

        if self.mapping_count % 5 == 0:
            self.get_logger().info(f'[地图] 已更新 {self.mapping_count} 次')


def main(args=None):
    rclpy.init(args=args)

    node = RobotNavStackNode()

    # ============================================================
    # 使用 MultiThreadedExecutor
    # 4个回调组充分并发
    # - 速度控制(50Hz) 绝不会被路径规划(1Hz) 阻塞
    # - 避障检测不会被地图更新阻塞
    # ============================================================
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'\n统计:\n'
            f'  速度控制: {node.control_count} 次 (50Hz)\n'
            f'  避障检测: {node.safety_count} 次 (10Hz)\n'
            f'  路径规划: {node.planning_count} 次 (1Hz)\n'
            f'  地图更新: {node.mapping_count} 次 (0.5Hz)'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
