"""③ 高级日志 —— 限流日志与条件日志（Python 手动实现）"""
import time

import rclpy
from rclpy.node import Node


class LoggingAdvancedNode(Node):
    """高级日志演示节点"""

    def __init__(self):
        super().__init__('logging_advanced')
        self.get_logger().info('=== Logging Advanced Demo ===')

        self._count = 0
        self._once_logged = False
        self._throttle_timestamps = {}  # 记录各 key 上次输出时间

        self._timer = self.create_timer(0.5, self._timer_callback)

    def _throttled_log(self, key: str, interval_sec: float, log_func, message: str):
        """手动实现限流日志

        Args:
            key: 限流标识（不同 key 独立计时）
            interval_sec: 限流间隔（秒）
            log_func: 日志函数（如 self.get_logger().info）
            message: 日志消息
        """
        now = time.time()
        last = self._throttle_timestamps.get(key, 0.0)
        if now - last >= interval_sec:
            log_func(message)
            self._throttle_timestamps[key] = now

    def _timer_callback(self):
        self._count += 1

        # ================================================================
        # ONCE：仅输出一次
        # ================================================================
        if not self._once_logged:
            self.get_logger().info(
                f'[ONCE] This appears only once (count={self._count})'
            )
            self._once_logged = True

        # ================================================================
        # THROTTLE：限流输出（手动实现）
        # ================================================================
        self._throttled_log(
            'status', 3.0,
            self.get_logger().info,
            f'[THROTTLE 3s] Periodic status (count={self._count})'
        )

        # ================================================================
        # EXPRESSION：条件表达式日志
        # ================================================================
        if self._count % 5 == 0:
            self.get_logger().info(
                f'[EXPRESSION] Count is multiple of 5 (count={self._count})'
            )

        if self._count > 10:
            self.get_logger().warning(
                f'[EXPRESSION] Count exceeded 10 (count={self._count})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LoggingAdvancedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
