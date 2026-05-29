import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(String, '/hello_world_topic', self.listener_callback, 10)
        self.subscription # 避免未使用变量的警告

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    listener_node = Listener()
    rclpy.spin(listener_node) # 启动节点的事件循环
    listener_node.destroy_node() # 显式清理并关闭节点（可选）
    rclpy.shutdown() # 关闭ROS2


if __name__ == '__main__':
    main()