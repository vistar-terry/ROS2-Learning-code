"""① 基础日志使用 —— 五种日志级别与基本方法"""
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity


class LoggingBasicNode(Node):
    """基础日志演示节点"""

    def __init__(self):
        super().__init__('logging_basic')
        self.get_logger().info('=== Logging Basic Demo ===')

        # ================================================================
        # 五种日志级别的基本使用
        # ================================================================

        # DEBUG 级别：调试信息，默认不输出
        self.get_logger().debug('This is a DEBUG message (invisible by default)')

        # INFO 级别：一般信息
        self.get_logger().info('This is an INFO message')

        # WARN 级别：警告信息
        self.get_logger().warning('This is a WARN message')

        # ERROR 级别：错误信息
        self.get_logger().error('This is an ERROR message')

        # FATAL 级别：致命错误
        self.get_logger().fatal('This is a FATAL message')

        # ================================================================
        # 格式化输出 —— Python f-string
        # ================================================================
        count = 42
        value = 3.14159
        name = 'sensor'
        self.get_logger().info(
            f'Integer: {count}, Double: {value:.2f}, String: {name}'
        )

        # ================================================================
        # 获取和设置日志级别
        # ================================================================
        current_level = self.get_logger().get_effective_level()
        self.get_logger().info(f'Current log level: {current_level}')

        # 设置日志级别为 DEBUG
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().debug('Now DEBUG messages are visible!')

        # 设置日志级别为 WARN
        self.get_logger().set_level(LoggingSeverity.WARN)
        self.get_logger().info('This INFO will NOT be shown')
        self.get_logger().warning('This WARN WILL be shown')

        # 恢复 INFO
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info('Back to INFO level')


def main(args=None):
    rclpy.init(args=args)
    node = LoggingBasicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
