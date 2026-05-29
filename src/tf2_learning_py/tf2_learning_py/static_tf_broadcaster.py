#!/usr/bin/env python3
"""
static_tf_broadcaster.py
ROS2 静态坐标广播器示例（Python版）

功能：发布一个固定的坐标变换关系
场景：传感器安装位置、机器人底盘与轮子的固定关系等

运行方式：
    ros2 run tf2_learning_py static_tf_broadcaster_py

验证方式：
    ros2 run tf2_ros tf2_echo base_link sensor_lidar
"""

import math

from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import rclpy


class StaticTfBroadcaster(Node):
    """静态坐标广播器节点"""

    def __init__(self):
        super().__init__('static_tf_broadcaster_py')

        # ============================================================
        # 1. 创建静态坐标广播器
        #    Python版的StaticTransformBroadcaster用法与C++版相同
        #    静态TF只需发布一次，之后会一直被缓存
        # ============================================================
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # 发布静态变换
        self.publish_static_transform()

        self.get_logger().info(
            '静态TF广播器(Python)已启动: base_link -> sensor_lidar '
            '[x=0.3, y=0.0, z=0.2, roll=0, pitch=0, yaw=0]'
        )

    def publish_static_transform(self):
        """发布一个从 base_link 到 sensor_lidar 的静态变换"""

        # ============================================================
        # 2. 构造 TransformStamped 消息
        # ============================================================
        t = TransformStamped()

        # --- 头信息 ---
        t.header.stamp = self.get_clock().now().to_msg()   # 时间戳
        t.header.frame_id = 'base_link'                     # 父坐标系
        t.child_frame_id = 'sensor_lidar'                   # 子坐标系

        # --- 平移部分 ---
        t.transform.translation.x = 0.3    # 前方0.3米
        t.transform.translation.y = 0.0    # 无左右偏移
        t.transform.translation.z = 0.2    # 上方0.2米

        # --- 旋转部分 ---
        # Python中四元数的构造：使用 from_euler 方法
        # 顺序：x(Roll), y(Pitch), y(Yaw)
        from tf2_ros import TransformException
        import transforms3d
        # 方法1：使用 transforms3d 库（推荐，更Pythonic）
        # quaternion = transforms3d.euler.euler2quat(roll, pitch, yaw, axes='sxyz')
        # 但transforms3d可能未安装，这里使用手动计算

        # 方法2：手动从欧拉角计算四元数
        # 对于无旋转的情况，四元数为 (0, 0, 0, 1)
        q = self.euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # ============================================================
        # 3. 发布静态变换
        # ============================================================
        self.tf_static_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float):
        """
        欧拉角转四元数

        参数：
            roll:  绕X轴旋转角度（弧度）
            pitch: 绕Y轴旋转角度（弧度）
            yaw:   绕Z轴旋转角度（弧度）

        返回：
            [x, y, z, w] 四元数列表

        数学原理：
            q_x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
            q_y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
            q_z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
            q_w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)
        """
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
    node = StaticTfBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
