import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.count_ = 0
        self.publisher_ = self.create_publisher(String, '/hello_world_topic', 10)
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
    talker_node = Talker()
    rclpy.spin(talker_node) # 启动节点的事件循环
    talker_node.destroy_node() # 清理并关闭节点
    rclpy.shutdown() # 关闭ROS2


if __name__ == '__main__':
    main()