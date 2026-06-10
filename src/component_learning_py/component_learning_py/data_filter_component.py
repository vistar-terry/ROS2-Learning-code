"""数据滤波组件 —— 对传感器数据进行滑动平均滤波"""
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class DataFilterComponent(Node):
    """数据滤波组件"""

    def __init__(self, **kwargs):
        super().__init__('data_filter', **kwargs)

        # 声明参数
        self.declare_parameter('window_size', 5)
        self.declare_parameter('input_topic', 'raw_temperature')
        self.declare_parameter('output_topic', 'filtered_temperature')

        self._window_size = self.get_parameter('window_size').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._buffer = deque()
        self._sub = self.create_subscription(
            Temperature, input_topic, self._on_temperature, 10)
        self._pub = self.create_publisher(Temperature, output_topic, 10)

        self.get_logger().info(
            f'DataFilterComponent initialized, '
            f'window_size={self._window_size}, sub={input_topic}, pub={output_topic}'
        )

    def _on_temperature(self, msg):
        self._buffer.append(msg.temperature)
        if len(self._buffer) > self._window_size:
            self._buffer.popleft()

        avg = sum(self._buffer) / len(self._buffer)

        out = Temperature()
        out.header = msg.header
        out.temperature = avg
        out.variance = 0.0

        self._pub.publish(out)


def main(args=None):
    """独立运行入口"""
    rclpy.init(args=args)
    node = DataFilterComponent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
