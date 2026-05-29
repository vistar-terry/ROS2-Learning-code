# sensor_processor/sensor_processor/my_node.py

import rclpy
from rclpy.node import Node
from sensor_processor.sensor_data_processor import SensorDataProcessor
from sensor_processor.actuator_controller import ActuatorController
from std_msgs.msg import Int32

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.sensor_processor = SensorDataProcessor()
        self.actuator_controller = ActuatorController()
        self.actuator_controller.activate()  # 初始化时激活执行器

        # 创建一个订阅者来接收传感器数据
        self.subscription = self.create_subscription(
            Int32,  # 假设传感器数据是整数类型
            'sensor_data',  # 主题名
            self.listener_callback,  # 当接收到数据时调用的回调函数
            10  # QoS设置（队列大小）
        )
        self.subscription  # 防止被垃圾回收

    def listener_callback(self, msg):
        # 当接收到传感器数据时调用此函数
        self.sensor_processor.process_data(msg.data)
        # 根据处理后的数据执行其他操作（例如，根据数据值切换执行器状态）
        # 这里只是一个简单的示例，不根据数据值做决定
        self.actuator_controller.toggle()

    def spin(self):
        # 重写spin方法以添加初始化逻辑（通常不需要这样做，但这里为了示例）
        print("Node is spinning...")
        super().spin()  # 调用父类的spin方法来启动事件循环

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)  # 启动节点的事件循环
    node.destroy_node()  # 清理并关闭节点
    rclpy.shutdown()  # 关闭ROS 2

if __name__ == '__main__':
    main()