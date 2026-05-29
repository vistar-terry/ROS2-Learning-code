# setuptools 用于简化包的构建、分发和安装过程
from setuptools import find_packages, setup

# 包名称
package_name = 'callback_group_learning'

# 调用setup配置包信息
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        ('share/' + package_name + '/launch', [
            'launch/01_default_behavior.launch.py',
            'launch/02_mutually_exclusive.launch.py',
            'launch/03_reentrant.launch.py',
            'launch/04_executor_types.launch.py',
            'launch/05_multi_node_executor.launch.py',
            'launch/06_robot_sensor_fusion.launch.py',
            'launch/07_robot_nav_stack.launch.py',
            'launch/08_deadlock_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vistar',
    maintainer_email='vistar@todo.todo',
    description='ROS2 回调组（Callback Group）学习包',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 定义入口点 —— 8个演示节点
    entry_points={
        'console_scripts': [
            # ① 默认行为演示：单线程执行器下的回调阻塞
            'default_behavior = callback_group_learning.default_behavior:main',
            # ② MutuallyExclusive 回调组演示
            'mutually_exclusive_demo = callback_group_learning.mutually_exclusive_demo:main',
            # ③ Reentrant 回调组演示
            'reentrant_demo = callback_group_learning.reentrant_demo:main',
            # ④ 执行器类型对比
            'executor_types = callback_group_learning.executor_types:main',
            # ⑤ 多节点单执行器
            'multi_node_executor = callback_group_learning.multi_node_executor:main',
            # ⑥ 机器人传感器融合（实际案例）
            'robot_sensor_fusion = callback_group_learning.robot_sensor_fusion:main',
            # ⑦ 机器人导航栈（实际案例）
            'robot_nav_stack = callback_group_learning.robot_nav_stack:main',
            # ⑧ 死锁演示与解决
            'deadlock_demo = callback_group_learning.deadlock_demo:main',
        ],
    },
)
