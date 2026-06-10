"""数据显示组件 —— 订阅滤波后的温度数据并打印"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class DataDisplayComponent(Node):
    """数据显示组件"""

    def __init__(self, **kwargs):
        super().__init__('data_display', **kwargs)

        # 声明参数
        self.declare_parameter('input_topic', 'filtered_temperature')
        input_topic = self.get_parameter('input_topic').value

        self._count = 0
        self._sub = self.create_subscription(
            Temperature, input_topic, self._on_filtered_temperature, 10)

        self.get_logger().info(
            f'DataDisplayComponent initialized, sub={input_topic}'
        )

    def _on_filtered_temperature(self, msg):
        self._count += 1
        self.get_logger().info(
            f'[{self._count:4d}] Filtered temperature: '
            f'{msg.temperature:.2f} C (variance: {msg.variance:.4f})'
        )


def main(args=None):
    """独立运行入口"""
    rclpy.init(args=args)
    node = DataDisplayComponent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
