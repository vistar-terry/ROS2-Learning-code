#!/usr/bin/env python3
"""
tf_point_transform.py
ROS2 坐标点变换示例（Python版）

功能：将一个坐标系下的点变换到另一个坐标系下
场景：传感器数据坐标转换、目标检测后的位置映射等

运行方式：
    # 终端1：运行动态广播器
    ros2 run tf2_learning_py dynamic_tf_broadcaster_py
    # 终端2：运行点变换示例
    ros2 run tf2_learning_py tf_point_transform_py
"""

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401 - 注册PointStamped的doTransform


class TfPointTransform(Node):
    """坐标点变换节点"""

    def __init__(self):
        super().__init__('tf_point_transform_py')

        # 创建TF缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2秒查询一次
        self.timer = self.create_timer(2.0, self.transform_point)

        self.get_logger().info(
            '坐标点变换(Python)已启动: 将 rotating_sensor 中的点变换到 base_link 中'
        )

    def transform_point(self):
        """定时器回调：执行坐标点变换"""

        # ============================================================
        # 1. 构造源坐标系中的点
        #    假设在旋转传感器坐标系中，前方1米处有一个障碍物
        # ============================================================
        sensor_point = PointStamped()
        sensor_point.header.frame_id = 'rotating_sensor'
        sensor_point.header.stamp = self.get_clock().now().to_msg()
        sensor_point.point.x = 1.0    # 传感器前方1米
        sensor_point.point.y = 0.0
        sensor_point.point.z = 0.0

        try:
            # ============================================================
            # 2. 方法一：使用 do_transform_point 进行点变换（推荐）
            #    Python版的 do_transform_point 等价于 C++ 的 doTransform
            #    需要导入 tf2_geometry_msgs 模块
            # ============================================================
            transform = self.tf_buffer.lookup_transform(
                'base_link',                    # 目标坐标系
                'rotating_sensor',              # 源坐标系
                rclpy.time.Time()               # 最新变换
            )

            # 使用 tf2_geometry_msgs 提供的变换函数
            base_point = tf2_geometry_msgs.do_transform_point(sensor_point, transform)

            self.get_logger().info(
                f'方法一(do_transform_point):\n'
                f'  传感器坐标系中的点: ({sensor_point.point.x:.3f}, '
                f'{sensor_point.point.y:.3f}, {sensor_point.point.z:.3f})\n'
                f'  机器人坐标系中的点: ({base_point.point.x:.3f}, '
                f'{base_point.point.y:.3f}, {base_point.point.z:.3f})'
            )

            # ============================================================
            # 3. 方法二：手动计算
            #    从变换消息中提取旋转和平移，手动进行矩阵运算
            # ============================================================
            import math

            # 提取四元数
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # 计算旋转矩阵
            r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
            r01 = 2.0 * (qx * qy - qw * qz)
            r02 = 2.0 * (qx * qz + qw * qy)
            r10 = 2.0 * (qx * qy + qw * qz)
            r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
            r12 = 2.0 * (qy * qz - qw * qx)

            # 提取源点坐标
            xs = sensor_point.point.x
            ys = sensor_point.point.y

            # 计算：p_target = R * p_source + t
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y

            xt = r00 * xs + r01 * ys + tx
            yt = r10 * xs + r11 * ys + ty

            self.get_logger().info(
                f'方法二(手动矩阵):\n'
                f'  机器人坐标系中的点: ({xt:.3f}, {yt:.3f})'
            )

        except TransformException as ex:
            self.get_logger().warn(f'点变换失败: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = TfPointTransform()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
