"""传感器数据源组件 —— 模拟温度传感器发布数据"""
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


class SensorSourceComponent(Node):
    """传感器数据源组件"""

    def __init__(self, **kwargs):
        # ★ 组件必须传递 **kwargs 给父类
        super().__init__('sensor_source', **kwargs)

        # 声明参数
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('base_temperature', 25.0)
        self.declare_parameter('noise_amplitude', 2.0)

        self._temperature = self.get_parameter('base_temperature').value
        rate = self.get_parameter('publish_rate').value

        # 创建发布器
        self._pub = self.create_publisher(Temperature, 'raw_temperature', 10)

        # 创建定时器
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._publish_temperature)

        self.get_logger().info(
            f'SensorSourceComponent initialized, '
            f'rate={rate:.1f} Hz, base_temp={self._temperature:.1f}'
        )

    def _publish_temperature(self):
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'

        noise_amp = self.get_parameter('noise_amplitude').value
        msg.temperature = self._temperature + random.gauss(0, noise_amp)
        msg.variance = noise_amp * noise_amp

        self._pub.publish(msg)


def main(args=None):
    """独立运行入口"""
    rclpy.init(args=args)
    node = SensorSourceComponent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
