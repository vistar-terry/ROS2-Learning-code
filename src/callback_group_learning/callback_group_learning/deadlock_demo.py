#!/usr/bin/env python3
"""
=============================================================================
 ⑧ 死锁演示与解决 —— 回调组常见陷阱
=============================================================================

核心知识点：
  - 死锁：两个或多个回调互相等待，导致都无法继续执行
  - ROS2 中常见的3种死锁模式：
    1. 同一 MutuallyExclusive 回调组内的服务调用死锁
    2. 跨节点服务调用 + MutualExclusiveCallbackGroup 死锁
    3. 单线程执行器中长时间阻塞回调

死锁场景1（经典）：
  - 节点A有一个服务端和一个定时器
  - 定时器回调中调用节点A自己的服务
  - 如果服务端和定时器在同一个 MutuallyExclusive 回调组中
  - 定时器回调持有组锁，等待服务响应
  - 服务回调需要获取组锁才能执行
  - 结果：死锁！

解决方案：
  1. 将服务端和调用方放在不同的回调组
  2. 使用 ReentrantCallbackGroup
  3. 避免在回调中同步调用自己的服务（改用直接函数调用）

死锁场景2（跨节点）：
  - 节点A在回调中调用节点B的服务
  - 节点B在回调中调用节点A的服务
  - 如果双方都在 MutuallyExclusive 回调组中
  - 单线程执行器下必定死锁

本示例：
  - 演示死锁场景1（同节点服务调用）
  - 提供正确的解决方案
=============================================================================
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from std_msgs.msg import String
from example_interfaces.srv import Trigger


# ================================================================
# 死锁版本 —— 演示问题
# ================================================================

class DeadlockNode(Node):
    """死锁演示节点 —— 服务端和调用方在同一回调组"""

    def __init__(self):
        super().__init__('deadlock_node')

        # ============================================================
        # 错误设计：服务端和定时器在同一个 MutuallyExclusive 回调组
        # 这会导致定时器回调中调用服务时产生死锁！
        # ============================================================
        self.bad_group = MutuallyExclusiveCallbackGroup()

        # 服务端 —— 在 bad_group 中
        self.srv = self.create_service(
            Trigger, '/deadlock_trigger', self.service_callback,
            callback_group=self.bad_group,  # 与定时器同组
        )

        # 定时器 —— 也在 bad_group 中
        self.timer = self.create_timer(
            2.0, self.timer_callback,
            callback_group=self.bad_group,  # 与服务端同组
        )

        self.get_logger().warn('=' * 60)
        self.get_logger().warn('死锁演示节点启动')
        self.get_logger().warn('  服务端和定时器在同一 MutuallyExclusive 回调组')
        self.get_logger().warn('  定时器回调中调用自己的服务 → 会死锁！')
        self.get_logger().warn('=' * 60)

    def service_callback(self, request, response):
        """服务回调 —— 需要 bad_group 的锁才能执行"""
        self.get_logger().info('[服务] 收到请求，处理中...')
        time.sleep(0.5)
        response.success = True
        response.message = '服务处理完成'
        return response

    def timer_callback(self):
        """定时器回调 —— 持有 bad_group 的锁"""
        self.get_logger().info('[定时器] 调用自己的服务...')

        # ============================================================
        # 死锁发生点！
        # 1. timer_callback 获取了 bad_group 的互斥锁
        # 2. 调用服务需要服务回调执行
        # 3. 服务回调也在 bad_group 中，需要获取同一个互斥锁
        # 4. 但互斥锁已被 timer_callback 持有
        # 5. 死锁！
        # ============================================================

        # 使用 call_async 而非 call，避免同步阻塞
        # 但在同一个 MutuallyExclusive 回调组中，
        # 即使是 call_async，服务回调也无法执行（组锁被持有）
        future = self.srv.call_async(Trigger.Request())
        self.get_logger().info('[定时器] 服务调用已发送，等待响应...')

        # 注意：在 SingleThreadedExecutor 中，
        # 服务回调永远不会被执行（因为组锁被持有）
        # 在 MultiThreadedExecutor 中也是一样！
        # 因为 MutuallyExclusive 回调组的锁被定时器回调持有


# ================================================================
# 正确版本 —— 解决死锁
# ================================================================

class FixedNode(Node):
    """正确设计的节点 —— 服务端和调用方在不同回调组"""

    def __init__(self):
        super().__init__('fixed_node')

        # ============================================================
        # 正确设计1：服务端和定时器使用不同的回调组
        # ============================================================
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()

        # 服务端 —— 在 service_group 中
        self.srv = self.create_service(
            Trigger, '/fixed_trigger', self.service_callback,
            callback_group=self.service_group,  # 独立回调组
        )

        # 定时器 —— 在 timer_group 中
        self.timer = self.create_timer(
            2.0, self.timer_callback,
            callback_group=self.timer_group,  # 另一个回调组
        )

        # 订阅示例 —— 也在独立回调组
        self.sub_group = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription(
            String, '/status', self.status_callback, 10,
            callback_group=self.sub_group,
        )

        self.call_count = 0
        self.get_logger().info('=' * 60)
        self.get_logger().info('正确设计节点启动')
        self.get_logger().info('  服务端 → service_group (独立)')
        self.get_logger().info('  定时器 → timer_group (独立)')
        self.get_logger().info('  订阅   → sub_group (独立)')
        self.get_logger().info('  使用 MultiThreadedExecutor → 不会死锁')
        self.get_logger().info('=' * 60)

    def service_callback(self, request, response):
        """服务回调 —— 在 service_group 中，可以与定时器并发"""
        self.get_logger().info('[服务] 收到请求，处理中...')
        time.sleep(0.5)
        response.success = True
        response.message = f'服务处理完成，第 {self.call_count} 次调用'
        return response

    def timer_callback(self):
        """定时器回调 —— 在 timer_group 中"""
        self.call_count += 1
        self.get_logger().info(f'[定时器] 调用服务 (第{self.call_count}次)...')

        # 使用 call_async 异步调用
        # 服务端在不同的回调组中，可以并发执行
        future = self.srv.call_async(Trigger.Request())

        # 添加完成回调（不要在此阻塞等待结果）
        def on_done(fut):
            try:
                result = fut.result()
                self.get_logger().info(
                    f'[定时器] 服务响应: success={result.success}, '
                    f'msg="{result.message}"'
                )
            except Exception as e:
                self.get_logger().error(f'[定时器] 服务调用失败: {e}')

        future.add_done_callback(on_done)

    def status_callback(self, msg: String):
        """状态订阅回调 —— 在 sub_group 中"""
        self.get_logger().info(f'[订阅] 状态: {msg.data}')


# ================================================================
# 最佳实践版本 —— 避免回调中调用服务
# ================================================================

class BestPracticeNode(Node):
    """最佳实践 —— 避免在回调中调用自己的服务"""

    def __init__(self):
        super().__init__('best_practice_node')

        # 只需要一个回调组
        self.group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            2.0, self.timer_callback,
            callback_group=self.group,
        )

        # 核心逻辑封装在普通方法中
        self.get_logger().info('=' * 60)
        self.get_logger().info('最佳实践节点启动')
        self.get_logger().info('  不在回调中调用自己的服务')
        self.get_logger().info('  而是直接调用内部方法')
        self.get_logger().info('=' * 60)

    def do_something(self):
        """核心逻辑封装在普通方法中（而非服务回调中）"""
        self.get_logger().info('[核心逻辑] 执行处理...')
        time.sleep(0.5)
        self.get_logger().info('[核心逻辑] 处理完成')
        return True, '处理成功'

    def timer_callback(self):
        """定时器回调 —— 直接调用内部方法，不走服务"""
        # ✅ 最佳实践：直接调用方法，避免服务调用的开销和死锁风险
        success, message = self.do_something()
        self.get_logger().info(f'[定时器] 结果: success={success}, msg="{message}"')


def main(args=None):
    rclpy.init(args=args)

    print('\n' + '=' * 60)
    print('选择演示模式:')
    print('  1 - 死锁演示（SingleThreadedExecutor）')
    print('  2 - 正确版本（MultiThreadedExecutor + 独立回调组）')
    print('  3 - 最佳实践（直接方法调用）')
    print('=' * 60)

    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else '3'

    if mode == '1':
        # ============================================================
        # 死锁演示
        # ============================================================
        node = DeadlockNode()
        executor = MultiThreadedExecutor(num_threads=4)
        # 即使使用 MultiThreadedExecutor，
        # 同一 MutuallyExclusive 回调组中的服务回调也无法执行
        # 因为定时器回调持有组锁
        try:
            rclpy.spin(node, executor=executor)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    elif mode == '2':
        # ============================================================
        # 正确版本：不同回调组 + MultiThreadedExecutor
        # ============================================================
        node = FixedNode()
        executor = MultiThreadedExecutor(num_threads=4)
        try:
            rclpy.spin(node, executor=executor)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    else:
        # ============================================================
        # 最佳实践：避免回调中调用自己的服务
        # ============================================================
        node = BestPracticeNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
