#!/usr/bin/env python3
import sys
import asyncio
import threading
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientAsync(Node):
    def __init__(self):
        super().__init__("add_two_ints_client_async")
        self.client = self.create_client(AddTwoInts, "add_two_ints")
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("服务未启动，等待中...")
            if not rclpy.ok():
                self.get_logger().error("节点被中断，退出")
                return
        
        self.get_logger().info("服务已连接！")
    
    async def send_request(self, a: int, b: int):
        """异步发送请求并等待响应"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # 调用服务（返回ROS2 Future）
        ros_future = self.client.call_async(request)
        self.get_logger().info(f"发送请求: {a} + {b}")
        
        # 将ROS2 Future转换为asyncio Future
        loop = asyncio.get_running_loop()
        asyncio_future = loop.create_future()
        
        def done_callback(rf):
            """ROS2服务完成时的回调"""
            try:
                result = rf.result()
                loop.call_soon_threadsafe(asyncio_future.set_result, result)
            except Exception as e:
                loop.call_soon_threadsafe(asyncio_future.set_exception, e)
        
        # 添加回调
        ros_future.add_done_callback(done_callback)
        
        # 异步等待结果
        response = await asyncio_future
        self.get_logger().info(f"收到响应: {response.sum}")
        return response.sum


async def async_main(args=None):
    """异步主函数"""
    rclpy.init(args=args)
    
    # 检查参数
    if len(sys.argv) != 3:
        print("用法: ros2 run 包名 add_two_ints_client_async <a> <b>")
        rclpy.shutdown()
        return
    
    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print("参数必须是整数！")
        rclpy.shutdown()
        return
    
    # 创建客户端
    client = AddTwoIntsClientAsync()
    
    # 在后台线程中运行ROS2 spin
    def spin():
        rclpy.spin(client)
    
    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    
    try:
        # 异步调用服务
        result = await client.send_request(a, b)
        print(f"\n最终结果: {a} + {b} = {result}")
    finally:
        rclpy.shutdown()


def main():
    asyncio.run(async_main())


if __name__ == "__main__":
    main()