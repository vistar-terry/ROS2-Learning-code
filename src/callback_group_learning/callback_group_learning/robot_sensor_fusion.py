#!/usr/bin/env python3
"""
=============================================================================
 ⑥ 机器人传感器融合 —— 实际应用案例
=============================================================================

场景描述：
  一个移动机器人配备了多种传感器：
  - 激光雷达 (10Hz)：用于避障
  - IMU (100Hz)：用于姿态估计
  - 相机 (30Hz)：用于目标检测

回调组设计考量：
  1. 激光雷达回调 —— 高频，但处理较快 → MutuallyExclusive（保护障碍物列表）
  2. IMU 回调 —— 最高频(100Hz)，不能被阻塞 → 独立 MutuallyExclusive
  3. 相机回调 —— 低频但处理耗时 → Reentrant（允许并发处理多帧）
  4. 融合回调 —— 定时融合所有传感器数据 → 独立 MutuallyExclusive

核心设计原则：
  - 高频传感器回调不能被低频耗时回调阻塞
  - 共享数据需要用 MutuallyExclusive 或锁保护
  - 耗时不紧急的处理用 Reentrant 允许并发
  - 各传感器回调分配到不同的回调组，最大化并发度
=============================================================================
"""

import threading
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu, Image


class RobotSensorFusionNode(Node):
    """机器人传感器融合节点 —— 回调组实际应用"""

    def __init__(self):
        super().__init__('robot_sensor_fusion')

        # ============================================================
        # 1. 回调组设计（核心！）
        #    每种传感器的回调放入独立的回调组
        #    避免高频传感器被低频传感器阻塞
        # ============================================================

        # 激光雷达回调组 —— MutuallyExclusive
        # 原因：需要保护共享的障碍物数据
        self.lidar_group = MutuallyExclusiveCallbackGroup()

        # IMU 回调组 —— 独立的 MutuallyExclusive
        # 原因：IMU 最高频(100Hz)，不能被任何其他回调阻塞
        # 独立组确保 IMU 回调可以与其他组的回调并发执行
        self.imu_group = MutuallyExclusiveCallbackGroup()

        # 相机回调组 —— Reentrant
        # 原因：图像处理耗时，允许多帧并发处理
        # 注意：如果处理结果写入共享变量，需要自行加锁！
        self.camera_group = ReentrantCallbackGroup()

        # 融合回调组 —— 独立 MutuallyExclusive
        # 原因：融合逻辑需要读取所有传感器数据，需要原子性
        self.fusion_group = MutuallyExclusiveCallbackGroup()

        # ============================================================
        # 2. 订阅传感器数据
        # ============================================================

        # 激光雷达订阅 —— 10Hz，可靠传输
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10,
            callback_group=self.lidar_group,
        )

        # IMU 订阅 —— 100Hz，最佳努力传输
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, imu_qos,
            callback_group=self.imu_group,
        )

        # 相机订阅 —— 30Hz
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10,
            callback_group=self.camera_group,
        )

        # ============================================================
        # 3. 融合定时器 —— 周期性融合所有传感器数据
        # ============================================================
        self.fusion_timer = self.create_timer(
            0.1,  # 10Hz 融合
            self.fusion_callback,
            callback_group=self.fusion_group,
        )

        # ============================================================
        # 4. 共享数据（各回调组独立，需要线程安全访问）
        # ============================================================
        self.lock = threading.Lock()

        # 激光数据（受 lidar_group 的 MutuallyExclusive 保护）
        self.min_distance = float('inf')
        self.obstacle_angles = []

        # IMU 数据（受 imu_group 的 MutuallyExclusive 保护）
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.imu_count = 0

        # 相机数据（Reentrant 回调组，需要锁保护）
        self.detected_objects = []
        self.camera_count = 0

        # 统计
        self.lidar_count = 0
        self.fusion_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('机器人传感器融合节点启动')
        self.get_logger().info('  激光雷达 → lidar_group (MutuallyExclusive)')
        self.get_logger().info('  IMU      → imu_group (MutuallyExclusive, 独立)')
        self.get_logger().info('  相机     → camera_group (Reentrant)')
        self.get_logger().info('  融合     → fusion_group (MutuallyExclusive, 独立)')
        self.get_logger().info('=' * 60)

    # ================================================================
    # 传感器回调
    # ================================================================

    def lidar_callback(self, msg: LaserScan):
        """激光雷达回调 —— 在 lidar_group 中
        由于 lidar_group 是 MutuallyExclusive，
        这个回调不会与自身并发，obstacle_angles 安全
        """
        self.lidar_count += 1

        # 提取最近障碍物距离
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            # 提取障碍物角度
            self.obstacle_angles = []
            for i, r in enumerate(msg.ranges):
                if r < 1.0 and r > msg.range_min:
                    angle = msg.angle_min + i * msg.angle_increment
                    self.obstacle_angles.append(angle)

    def imu_callback(self, msg: Imu):
        """IMU 回调 —— 在 imu_group 中（独立回调组）
        由于是独立的 MutuallyExclusive 组，
        这个回调不会被激光雷达或相机回调阻塞
        """
        self.imu_count += 1

        # 从四元数计算欧拉角（简化版）
        q = msg.orientation
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))  # clamp
        self.pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def camera_callback(self, msg: Image):
        """相机回调 —— 在 camera_group (Reentrant) 中
        使用 Reentrant 允许多帧并发处理
        共享数据 detected_objects 需要锁保护
        """
        self.camera_count += 1

        # 模拟图像处理耗时（50ms）
        time.sleep(0.05)

        # 模拟目标检测结果（需要锁保护，因为 Reentrant 允许并发）
        with self.lock:
            self.detected_objects = [
                {'class': 'person', 'confidence': 0.95, 'distance': 2.5},
                {'class': 'chair', 'confidence': 0.87, 'distance': 1.8},
            ]

    def fusion_callback(self):
        """融合回调 —— 在 fusion_group 中
        读取所有传感器的最新数据，做综合判断
        由于 fusion_group 是独立 MutuallyExclusive，
        融合逻辑不会被传感器回调阻塞
        """
        self.fusion_count += 1

        # 读取各传感器数据
        # 注意：跨回调组读取数据可能读到不一致的快照
        # 严格场景下需要加锁或使用更高级的同步机制
        with self.lock:
            min_dist = self.min_distance
            obstacles = list(self.obstacle_angles)
            detected = list(self.detected_objects)
            roll, pitch, yaw = self.roll, self.pitch, self.yaw
            imu_ok = self.imu_count > 0

        # 综合判断
        danger_level = '安全'
        if min_dist < 0.3:
            danger_level = '危险！紧急停止'
        elif min_dist < 0.5:
            danger_level = '警告！减速避障'
        elif min_dist < 1.0:
            danger_level = '注意'

        # 每10次融合输出一次日志（避免刷屏）
        if self.fusion_count % 10 == 0:
            self.get_logger().info(
                f'[融合 #{self.fusion_count}] '
                f'最近障碍: {min_dist:.2f}m, '
                f'障碍数: {len(obstacles)}, '
                f'姿态: R={math.degrees(roll):.1f}° P={math.degrees(pitch):.1f}° '
                f'Y={math.degrees(yaw):.1f}°, '
                f'检测目标: {len(detected)}, '
                f'状态: {danger_level}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = RobotSensorFusionNode()

    # ============================================================
    # 使用 MultiThreadedExecutor
    # 4个独立回调组可以充分并发
    # ============================================================
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'\n统计: 激光 {node.lidar_count} 帧, '
            f'IMU {node.imu_count} 帧, '
            f'相机 {node.camera_count} 帧, '
            f'融合 {node.fusion_count} 次'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
