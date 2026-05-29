#!/usr/bin/env python3
# client.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys
import time


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        # 创建客户端
        self.client = self.create_client(AddTwoInts, '/add_two_ints')
        
    def send_request(self, a, b):
        """发送请求到服务端"""
        # 等待服务端可用
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('等待服务端...')
        
        # 创建请求
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        self.get_logger().info(f'发送请求: {a} + {b}')
        
        # 异步发送请求
        future = self.client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f'计算结果: {response.sum}')
                return True
            except Exception as e:
                self.get_logger().error(f'请求失败: {e}')
                return False
        else:
            self.get_logger().error('请求未完成')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    # 创建客户端节点
    client = AddTwoIntsClient()
    
    # 从命令行参数获取数字
    if len(sys.argv) >= 3:
        try:
            a = int(sys.argv[1])
            b = int(sys.argv[2])
            client.get_logger().info(f'请求参数: {a} + {b}')
            client.send_request(a, b)
        except ValueError:
            client.get_logger().error('参数必须是整数')
    else:
        # 如果没有提供参数，使用默认值
        client.get_logger().info('使用默认参数: 5 + 3')
        client.send_request(5, 3)
    
    # 清理资源
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()