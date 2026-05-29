#!/usr/bin/env python3
# server.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):

        super().__init__('add_two_ints_server')

        # 创建服务
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.handle_add_two_ints
        )
        
        self.get_logger().info("AddTwoInts 服务端已启动...")

    def handle_add_two_ints(self, request, response):
        """处理加法请求的回调函数"""
        self.get_logger().info(f"收到请求: {request.a} + {request.b}")
        response.sum = request.a + request.b
        self.get_logger().info(f"返回结果: {response.sum}")
        return response


def main(args=None):
    rclpy.init(args=args)
    
    # 创建服务节点
    server = AddTwoIntsServer()
    
    # 运行节点
    rclpy.spin(server)
    
    # 清理资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()