"""④ 机器人实战案例 —— 命名日志器与多模块日志分层"""
import random

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import Temperature


class RobotLoggingDemoNode(Node):
    """机器人日志实战演示节点"""

    def __init__(self):
        super().__init__('robot_logging_demo')

        # ================================================================
        # 为各子系统创建命名日志器
        # ================================================================
        self._sensor_logger = self.get_logger().get_child('sensor')
        self._safety_logger = self.get_logger().get_child('safety')

        # 安全模块只看 WARN 以上，传感器模块看 DEBUG
        self._safety_logger.set_level(LoggingSeverity.WARN)
        self._sensor_logger.set_level(LoggingSeverity.DEBUG)

        self.get_logger().info('=== Robot Logging Demo Started ===')

        # 发布器和定时器
        self._temp_pub = self.create_publisher(Temperature, 'robot/temperature', 10)
        self._sensor_timer = self.create_timer(0.2, self._sensor_callback)
        self._safety_timer = self.create_timer(1.0, self._safety_callback)

        # 状态变量
        self._temperature = 25.0
        self._sensor_count = 0
        self._emergency = False

    def _sensor_callback(self):
        self._sensor_count += 1

        # 模拟温度数据
        self._temperature = 25.0 + random.gauss(0, 2.0)
        if self._sensor_count % 20 == 0:
            self._temperature = 95.0 + random.gauss(0, 2.0)

        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temperature = self._temperature
        msg.variance = 4.0
        self._temp_pub.publish(msg)

        self._sensor_logger.debug(f'Temperature reading: {self._temperature:.2f} C')

        if self._temperature > 80.0:
            self._sensor_logger.warning(f'High temperature: {self._temperature:.1f} C')
        if self._temperature > 100.0:
            self._sensor_logger.error(f'Critical temperature: {self._temperature:.1f} C!')
            self._emergency = True

    def _safety_callback(self):
        if self._emergency:
            self._safety_logger.error('EMERGENCY STOP! Temperature critical')
            self._emergency = False
        else:
            self._safety_logger.debug('Safety check passed')


def main(args=None):
    rclpy.init(args=args)
    node = RobotLoggingDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
