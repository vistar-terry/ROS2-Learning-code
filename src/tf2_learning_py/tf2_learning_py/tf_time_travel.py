#!/usr/bin/env python3
"""
tf_time_travel.py
ROS2 TF时间旅行示例（Python版）

功能：演示如何查询特定时间点的TF变换
场景：回放bag数据时查询历史变换、多传感器时间同步等

运行方式：
    # 终端1：运行动态广播器
    ros2 run tf2_learning_py dynamic_tf_broadcaster_py
    # 终端2：运行时间旅行示例
    ros2 run tf2_learning_py tf_time_travel_py
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros import ExtrapolationException


class TfTimeTravel(Node):
    """TF时间旅行节点"""

    def __init__(self):
        super().__init__('tf_time_travel_py')
        self.count = 0

        # 创建TF缓冲区，缓存时间30秒
        self.tf_buffer = Buffer(duration=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 每3秒执行一次查询演示
        self.timer = self.create_timer(3.0, self.demo_time_travel)

        self.get_logger().info('TF时间旅行(Python)示例已启动')

    def demo_time_travel(self):
        """演示不同时间点的TF查询"""

        self.count += 1
        self.get_logger().info(f'\n========== 第{self.count}次查询 ==========')

        # ============================================================
        # 1. 查询最新变换
        # ============================================================
        try:
            latest_tf = self.tf_buffer.lookup_transform(
                'base_link', 'rotating_sensor', rclpy.time.Time())
            self.get_logger().info(
                f'[最新变换] x={latest_tf.transform.translation.x:.3f}, '
                f'y={latest_tf.transform.translation.y:.3f}'
            )
        except TransformException as ex:
            self.get_logger().warn(f'查询最新变换失败: {ex}')
            return

        # ============================================================
        # 2. 查询2秒前的变换
        # ============================================================
        try:
            now = self.get_clock().now()
            two_sec_ago = now - Duration(seconds=2)

            past_tf = self.tf_buffer.lookup_transform(
                'base_link', 'rotating_sensor', two_sec_ago)
            self.get_logger().info(
                f'[2秒前的变换] x={past_tf.transform.translation.x:.3f}, '
                f'y={past_tf.transform.translation.y:.3f}'
            )
        except ExtrapolationException as ex:
            self.get_logger().warn(f'2秒前的变换超出缓存范围: {ex}')
        except TransformException as ex:
            self.get_logger().warn(f'查询历史变换失败: {ex}')

        # ============================================================
        # 3. canTransform 带时间检查
        # ============================================================
        five_sec_ago = self.get_clock().now() - Duration(seconds=5)
        can_transform, error_msg = self.tf_buffer.can_transform(
            'base_link', 'rotating_sensor', five_sec_ago)
        if can_transform:
            self.get_logger().info('5秒前的变换可用')
        else:
            self.get_logger().info(f'5秒前的变换不可用: {error_msg}')


def main(args=None):
    rclpy.init(args=args)
    node = TfTimeTravel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
