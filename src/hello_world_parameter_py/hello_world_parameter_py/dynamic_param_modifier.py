import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import Parameter
import time


class DynamicParamModifier(Node):
    def __init__(self):
        super().__init__('dynamic_param_modifier')
        
        self.counter = 0
        
        # 创建参数客户端
        self.param_client = AsyncParameterClient(
            self, 'param_manager_node')
        
        # 创建定时器，定期修改参数
        self.timer = self.create_timer(3.0, self.modify_parameters)
        self.started = False
    
    def modify_parameters(self):
        """动态修改参数"""
        self.counter += 1
        
        # 动态修改参数
        new_speed = 1.0 + (self.counter % 5) * 0.5
        new_name = f"robot_{self.counter}"
        
        self.get_logger().info(
            f"Modifying parameters: speed={new_speed:.2f}, name={new_name}")
        
        # 异步设置参数
        set_future = self.param_client.set_parameters([
            Parameter('robot_speed', value=new_speed),
            Parameter('robot_name', value=new_name)
        ])
        
        # 添加完成回调
        def handle_set_result(f):
            try:
                response = f.result()
                for result in response.results:
                    if result.successful:
                        if not self.started:
                            self.started = True
                            self.get_logger().info("Successfully connected to parameter service")
                    else:
                        self.get_logger().error(
                            f"Failed to set parameter: {result.reason}")
            except Exception as e:
                self.get_logger().error(f"Error setting parameters: {e}")
        
        set_future.add_done_callback(handle_set_result)


def main(args=None):
    rclpy.init(args=args)
    modifier = DynamicParamModifier()
    
    try:
        rclpy.spin(modifier)
    except KeyboardInterrupt:
        pass
    finally:
        modifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
