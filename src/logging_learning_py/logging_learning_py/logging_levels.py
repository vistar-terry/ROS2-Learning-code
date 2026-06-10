"""② 日志级别控制 —— 运行时动态调整与命名日志器"""
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity


class LoggingLevelsNode(Node):
    """日志级别控制演示节点"""

    def __init__(self):
        super().__init__('logging_levels')

        # ================================================================
        # 通过参数设置日志级别
        # ================================================================
        self.declare_parameter('log_level', 'info')
        log_level_str = self.get_parameter('log_level').get_parameter_value().string_value
        level = self._level_from_string(log_level_str)
        self.get_logger().set_level(level)
        self.get_logger().info(f'Log level set via parameter: {log_level_str}')

        # ================================================================
        # 命名日志器
        # ================================================================
        self._sensor_logger = self.get_logger().get_child('sensor')
        self._motor_logger = self.get_logger().get_child('motor')

        self._sensor_logger.set_level(LoggingSeverity.DEBUG)
        self._motor_logger.set_level(LoggingSeverity.WARN)

        self._sensor_logger.info('Sensor logger initialized (DEBUG level)')
        self._sensor_logger.debug('Sensor debug info visible')
        self._motor_logger.info('Motor INFO invisible (WARN level)')
        self._motor_logger.warning('Motor warning is visible')

        # ================================================================
        # 定时器演示
        # ================================================================
        self._count = 0
        self._timer = self.create_timer(2.0, self._timer_callback)

    def _timer_callback(self):
        self._count += 1

        # 每 10 次切换级别
        if self._count % 10 == 0:
            new_level = (
                LoggingSeverity.DEBUG
                if (self._count // 10) % 2 == 0
                else LoggingSeverity.INFO
            )
            self.get_logger().set_level(new_level)
            level_name = 'DEBUG' if new_level == LoggingSeverity.DEBUG else 'INFO'
            self.get_logger().info(f'Log level switched to {level_name}')

        self.get_logger().debug(f'Callback #{self._count} (DEBUG)')
        self.get_logger().info(f'Callback #{self._count} (INFO)')
        self.get_logger().warning(f'Callback #{self._count} (WARN)')

    @staticmethod
    def _level_from_string(level_str: str):
        """字符串转日志级别"""
        levels = {
            'debug': LoggingSeverity.DEBUG,
            'info': LoggingSeverity.INFO,
            'warn': LoggingSeverity.WARN,
            'error': LoggingSeverity.ERROR,
            'fatal': LoggingSeverity.FATAL,
        }
        return levels.get(level_str.lower(), LoggingSeverity.INFO)


def main(args=None):
    rclpy.init(args=args)
    node = LoggingLevelsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
