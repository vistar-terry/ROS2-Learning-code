#!/usr/bin/env python3
"""
tf_listener.py
ROS2 坐标监听器示例（Python版）

功能：监听TF变换，查询两个坐标系之间的变换关系
场景：获取传感器数据在机器人坐标系下的位置

运行方式：
    # 终端1：运行动态广播器
    ros2 run tf2_learning_py dynamic_tf_broadcaster_py
    # 终端2：运行监听器
    ros2 run tf2_learning_py tf_listener_py
"""

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException


class TfListener(Node):
    """坐标监听器节点"""

    def __init__(self):
        super().__init__('tf_listener_py')

        # ============================================================
        # 1. 创建TF缓冲区和监听器
        #    Python版API与C++版类似
        # ============================================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ============================================================
        # 2. 创建定时器，1秒查询一次
        # ============================================================
        self.timer = self.create_timer(1.0, self.lookup_transform)

        self.get_logger().info('TF监听器(Python)已启动，正在查询 base_link -> rotating_sensor...')

    def lookup_transform(self):
        """定时器回调：查询并打印TF变换"""

        # ============================================================
        # 3. 检查变换是否可用
        # ============================================================
        if not self.tf_buffer.can_transform('base_link', 'rotating_sensor', rclpy.time.Time()):
            self.get_logger().warn('变换 base_link -> rotating_sensor 尚不可用，等待中...')
            return

        try:
            # ============================================================
            # 4. 查询最新变换
            #    rclpy.time.Time() 等价于 C++ 中的 tf2::TimePointZero
            # ============================================================
            t = self.tf_buffer.lookup_transform(
                'base_link',            # 目标坐标系
                'rotating_sensor',      # 源坐标系
                rclpy.time.Time()       # 获取最新变换
            )

            # ============================================================
            # 5. 提取并打印变换信息
            # ============================================================
            self.get_logger().info(
                f'变换 base_link -> rotating_sensor:\n'
                f'  平移: x={t.transform.translation.x:.3f}, '
                f'y={t.transform.translation.y:.3f}, '
                f'z={t.transform.translation.z:.3f}\n'
                f'  旋转: x={t.transform.rotation.x:.3f}, '
                f'y={t.transform.rotation.y:.3f}, '
                f'z={t.transform.rotation.z:.3f}, '
                f'w={t.transform.rotation.w:.3f}'
            )

        except TransformException as ex:
            # ============================================================
            # 6. 异常处理
            # ============================================================
            self.get_logger().warn(f'TF查询失败: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = TfListener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
