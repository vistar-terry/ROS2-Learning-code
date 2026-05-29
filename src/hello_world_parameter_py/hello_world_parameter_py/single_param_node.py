import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time


class SingleParamNode(Node):
    def __init__(self):
        super().__init__('single_param_node')
        
        self.declare_parameter('my_parameter', 'world')
        
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        """定时器回调"""
        my_param = self.get_parameter('my_parameter').value
        self.get_logger().info(f"Hello {my_param}!")
        
        # 设置参数为新值
        self.set_parameters([Parameter('my_parameter', value='lllll')])


def main(args=None):
    rclpy.init(args=args)
    single_node = SingleParamNode()
    
    try:
        rclpy.spin(single_node)
    except KeyboardInterrupt:
        pass
    finally:
        single_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
