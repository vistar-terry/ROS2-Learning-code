#!/usr/bin/env python3
"""
worker_node.py
可重启的工作节点 — 用于演示 launch 中的 on_exit 事件和节点重启

功能：
- 模拟一个工作节点，运行指定次数后自动退出
- 用于演示 launch 中的节点重启、事件处理机制

参数：
- max_count: 最大工作次数后退出（默认 10，0表示无限运行）
- work_interval: 工作间隔秒数（默认 1.0）
- exit_code: 退出码（默认 0）

运行方式：
    ros2 run launch_learning worker_node
    ros2 run launch_learning worker_node --ros-args -p max_count:=5 -p exit_code:=1
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class WorkerNode(Node):
    """可重启的工作节点"""

    def __init__(self):
        super().__init__('worker_node')

        # 声明参数
        self.declare_parameter('max_count', 10)
        self.declare_parameter('work_interval', 1.0)
        self.declare_parameter('exit_code', 0)

        self.max_count = self.get_parameter('max_count').value
        work_interval = self.get_parameter('work_interval').value
        self.exit_code = self.get_parameter('exit_code').value

        # 创建发布者和定时器
        self.publisher = self.create_publisher(String, 'worker_status', 10)
        self.timer = self.create_timer(work_interval, self.work_callback)
        self.count = 0

        self.get_logger().info(
            f'工作节点启动: max_count={self.max_count}, '
            f'interval={work_interval}s, exit_code={self.exit_code}'
        )

    def work_callback(self):
        """定时器回调：执行工作"""
        self.count += 1

        # 发布工作状态
        msg = String()
        msg.data = f'working: step {self.count}/{self.max_count}'
        self.publisher.publish(msg)

        self.get_logger().info(f'执行第 {self.count}/{self.max_count} 步工作')

        # 达到最大次数后退出
        if self.max_count > 0 and self.count >= self.max_count:
            self.get_logger().info(
                f'工作完成！退出码={self.exit_code}'
            )
            raise SystemExit(self.exit_code)


def main(args=None):
    rclpy.init(args=args)
    node = WorkerNode()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        node.get_logger().info(f'节点退出，退出码={e.code}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
