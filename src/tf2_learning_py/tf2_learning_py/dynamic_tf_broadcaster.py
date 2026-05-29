#!/usr/bin/env python3
"""
dynamic_tf_broadcaster.py
ROS2 动态坐标广播器示例（Python版）

功能：发布一个随时间变化的坐标变换关系
场景：旋转的雷达、摆动的机械臂关节、移动的机器人等

运行方式：
    ros2 run tf2_learning_py dynamic_tf_broadcaster_py

验证方式：
    ros2 run tf2_ros tf2_echo base_link rotating_sensor
"""

import math

from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import rclpy


class DynamicTfBroadcaster(Node):
    """动态坐标广播器节点"""

    def __init__(self):
        super().__init__('dynamic_tf_broadcaster_py')

        # ============================================================
        # 1. 创建动态坐标广播器
        # ============================================================
        self.tf_broadcaster = TransformBroadcaster(self)

        # 初始化角度
        self.angle = 0.0

        # ============================================================
        # 2. 创建定时器，50ms = 20Hz 发布频率
        # ============================================================
        self.timer = self.create_timer(0.05, self.publish_transform)

        self.get_logger().info(
            '动态TF广播器(Python)已启动: base_link -> rotating_sensor (20Hz)'
        )

    def publish_transform(self):
        """定时器回调：发布动态变换"""

        # ============================================================
        # 3. 构造并发布动态变换
        # ============================================================
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'             # 父坐标系
        t.child_frame_id = 'rotating_sensor'         # 子坐标系

        # 模拟一个绕Z轴旋转的传感器
        radius = 0.5         # 旋转半径（米）
        height = 0.3         # 安装高度（米）
        angular_speed = 0.5  # 角速度（弧度/秒）

        # 计算当前位置（圆周运动）
        t.transform.translation.x = radius * math.cos(self.angle)
        t.transform.translation.y = radius * math.sin(self.angle)
        t.transform.translation.z = height

        # 计算当前姿态
        q = self.euler_to_quaternion(roll=0.0, pitch=0.0, yaw=self.angle)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 发布变换
        self.tf_broadcaster.sendTransform(t)

        # 更新角度
        self.angle += 0.05 * angular_speed  # 0.05s * 0.5 rad/s

        # 保持角度在 [-π, π]
        if self.angle > math.pi:
            self.angle -= 2 * math.pi

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float):
        """欧拉角转四元数（与静态广播器中相同）"""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
