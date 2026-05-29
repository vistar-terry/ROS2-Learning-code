import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.count_ = 0
        self.publisher_ = self.create_publisher(String, '/hello_world', 10)
        timer_period = 1.0  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello, world! {self.count_}"
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.publisher_.publish(msg)
        self.count_ += 1


def main(args=None):
    rclpy.init(args=args)
    hello_world_publisher_node = HelloWorldPublisher()
    rclpy.spin(hello_world_publisher_node) # 启动节点的事件循环
    hello_world_publisher_node.destroy_node() # 清理并关闭节点
    rclpy.shutdown() # 关闭ROS2


if __name__ == '__main__':
    main()