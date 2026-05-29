#!/usr/bin/env python3
r"""
robot_tf_broadcaster.py
ROS2 机器人TF树广播器示例（Python版）

功能：模拟一个移动机器人的完整TF树
场景：实际机器人中各种传感器和执行器相对于底盘的坐标关系

TF树结构：
                    map
                     |
                  odom
                     |
                 base_link
                /    |    \     \
    base_footprint  |  imu_link  camera_link
                    |
                laser_link

运行方式：
    ros2 run tf2_learning_py robot_tf_broadcaster_py
"""

import math

from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import rclpy


class RobotTfBroadcaster(Node):
    """机器人TF树广播器节点"""

    def __init__(self):
        super().__init__('robot_tf_broadcaster_py')

        # 初始化机器人位姿
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ============================================================
        # 创建广播器
        # ============================================================
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # ============================================================
        # 发布所有静态变换
        # ============================================================
        self.publish_static_transforms()

        # ============================================================
        # 创建定时器，20Hz发布动态变换
        # ============================================================
        self.timer = self.create_timer(0.05, self.publish_dynamic_transforms)

        self.get_logger().info('机器人TF树广播器(Python)已启动，模拟机器人圆周运动')

    def publish_static_transforms(self):
        """发布所有静态变换（传感器安装位置）"""

        static_transforms = []

        # --- base_link -> base_footprint ---
        static_transforms.append(self.create_transform(
            'base_link', 'base_footprint',
            0.0, 0.0, -0.1, 0.0, 0.0, 0.0))

        # --- base_link -> imu_link ---
        static_transforms.append(self.create_transform(
            'base_link', 'imu_link',
            0.15, 0.0, 0.05, 0.0, 0.0, 0.0))

        # --- base_link -> laser_link ---
        static_transforms.append(self.create_transform(
            'base_link', 'laser_link',
            0.0, 0.0, 0.15, 0.0, 0.0, 0.0))

        # --- base_link -> camera_link ---
        static_transforms.append(self.create_transform(
            'base_link', 'camera_link',
            0.2, 0.0, 0.12, 0.0, -0.349, 0.0))  # pitch=-20°

        # 一次性发布所有静态变换
        self.tf_static_broadcaster.sendTransform(static_transforms)

        self.get_logger().info(
            f'已发布 {len(static_transforms)} 个静态变换: '
            'base_footprint, imu_link, laser_link, camera_link'
        )

    def publish_dynamic_transforms(self):
        """发布动态变换（模拟机器人运动）"""

        # 模拟参数
        linear_speed = 0.2     # 线速度 m/s
        angular_speed = 0.1    # 角速度 rad/s
        dt = 0.05              # 时间步长

        # 更新位姿
        self.x += linear_speed * math.cos(self.theta) * dt
        self.y += linear_speed * math.sin(self.theta) * dt
        self.theta += angular_speed * dt

        if self.theta > math.pi:
            self.theta -= 2 * math.pi

        # --- map -> odom 变换 ---
        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = self.get_clock().now().to_msg()
        tf_map_odom.header.frame_id = 'map'
        tf_map_odom.child_frame_id = 'odom'
        tf_map_odom.transform.translation.x = 0.0
        tf_map_odom.transform.translation.y = 0.0
        tf_map_odom.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0, 0, 0)
        tf_map_odom.transform.rotation.x = q[0]
        tf_map_odom.transform.rotation.y = q[1]
        tf_map_odom.transform.rotation.z = q[2]
        tf_map_odom.transform.rotation.w = q[3]

        # --- odom -> base_link 变换 ---
        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = self.get_clock().now().to_msg()
        tf_odom_base.header.frame_id = 'odom'
        tf_odom_base.child_frame_id = 'base_link'
        tf_odom_base.transform.translation.x = self.x
        tf_odom_base.transform.translation.y = self.y
        tf_odom_base.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.theta)
        tf_odom_base.transform.rotation.x = q[0]
        tf_odom_base.transform.rotation.y = q[1]
        tf_odom_base.transform.rotation.z = q[2]
        tf_odom_base.transform.rotation.w = q[3]

        # 同时发布两个动态变换
        self.tf_broadcaster.sendTransform([tf_map_odom, tf_odom_base])

    def create_transform(self, parent, child, x, y, z, roll, pitch, yaw):
        """辅助函数：创建TransformStamped消息"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = self.euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """欧拉角转四元数"""
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
    node = RobotTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
