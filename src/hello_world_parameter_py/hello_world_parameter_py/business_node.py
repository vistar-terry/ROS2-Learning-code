import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterEvent


class BusinessNode(Node):
    def __init__(self):
        super().__init__('business_node')
        self.param_client = AsyncParameterClient(self, 'param_manager_node')
        self.timer = self.create_timer(3.0, self.operate_on_parameters)
        self.operation_count = 0
        self.param_names = ['robot_name', 'robot_speed']
        
        self.param_event_sub = self.create_subscription(
            ParameterEvent, '/parameter_events',
            self.parameter_event_callback, 10
        )
    
    def _extract_parameter_value(self, param_value):
        """从 ParameterValue 对象提取实际值"""
        return (param_value.string_value or param_value.integer_value or 
                param_value.double_value or param_value.bool_value or param_value)
    
    def _handle_get_result(self, f):
        """处理获取参数的结果"""
        try:
            param_values = f.result()
            for name, value in zip(self.param_names, param_values.values):
                self.get_logger().info(f"Got remote parameter '{name}': {value.string_value}")
        except Exception as e:
            self.get_logger().error(f"Error getting parameters: {e}")
    
    def _handle_set_result(self, f):
        """处理设置参数的结果"""
        try:
            response = f.result()
            for result in response.results:
                msg = "Successfully set parameter" if result.successful else f"Failed to set parameter: {result.reason}"
                log_func = self.get_logger().info if result.successful else self.get_logger().error
                log_func(msg)
        except Exception as e:
            self.get_logger().error(f"Error setting parameters: {e}")
    
    def operate_on_parameters(self):
        """定期获取和设置参数"""
        self.operation_count += 1
        
        if self.operation_count == 1:
            self.get_logger().info("Getting remote parameters...")
            future = self.param_client.get_parameters(self.param_names)
            future.add_done_callback(self._handle_get_result)
        
        elif self.operation_count == 2:
            self.get_logger().info("Setting remote parameters...")
            future = self.param_client.set_parameters([
                Parameter('robot_name', value='new_robot_from_client'),
                Parameter('robot_speed', value=2.5)
            ])
            future.add_done_callback(self._handle_set_result)
    
    def parameter_event_callback(self, msg):
        """处理参数事件"""
        for changed_param in msg.changed_parameters:
            param_value = self._extract_parameter_value(changed_param.value)
            self.get_logger().info(
                f"[{msg.node}] Parameter '{changed_param.name}' changed to {param_value}"
            )


def main(args=None):
    rclpy.init(args=args)
    business_node = BusinessNode()
    try:
        rclpy.spin(business_node)
    except KeyboardInterrupt:
        pass
    finally:
        business_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

