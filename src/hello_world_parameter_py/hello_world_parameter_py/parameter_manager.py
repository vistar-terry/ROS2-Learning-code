import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import threading
import time


class ParamManager(Node):
    def __init__(self):
        super().__init__('param_manager_node')
        
        # 1. 声明参数
        self.declare_parameter('robot_speed', 1.0)
        self.declare_parameter('robot_name', 'robot_alpha')
        self.declare_parameter('enable_debug', False)
        self.declare_parameter('waypoints', [1.0, 2.0, 3.0])
        
        # 2. 添加参数回调
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # 3. 创建定时器，定期打印参数值
        self.timer = self.create_timer(2.0, self.print_parameters)
        
        # 4. 初始打印参数
        self.print_parameters()
    
    def print_parameters(self):
        """获取并打印所有参数"""
        robot_speed = self.get_parameter('robot_speed').value
        robot_name = self.get_parameter('robot_name').value
        enable_debug = self.get_parameter('enable_debug').value
        waypoints = self.get_parameter('waypoints').value
        
        self.get_logger().info("=== Current Parameters ===")
        self.get_logger().info(f"Robot Speed: {robot_speed:.2f}")
        self.get_logger().info(f"Robot Name: {robot_name}")
        self.get_logger().info(f"Enable Debug: {enable_debug}")
        self.get_logger().info(f"Waypoints: {waypoints}")
        self.get_logger().info("==========================")
    
    def parameters_callback(self, parameters):
        """参数变化回调"""
        result = SetParametersResult()
        result.successful = True
        
        for param in parameters:
            self.get_logger().info(
                f"Parameter '{param.name}' changed to: {param.value}")
            
            # 参数验证
            if param.name == 'robot_speed':
                speed = param.value
                if speed < 0.0 or speed > 10.0:
                    result.successful = False
                    result.reason = "Robot speed must be between 0 and 10"
                    self.get_logger().warn(
                        f"Invalid speed value: {speed}")
                    return result
            
            if param.name == 'robot_name':
                name = param.value
                if not name:
                    result.successful = False
                    result.reason = "Robot name cannot be empty"
                    return result
        
        return result


def main(args=None):
    rclpy.init(args=args)
    param_manager = ParamManager()
    
    try:
        rclpy.spin(param_manager)
    except KeyboardInterrupt:
        pass
    finally:
        param_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
