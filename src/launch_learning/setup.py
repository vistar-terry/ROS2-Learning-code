from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'launch_learning'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    # 安装所有launch文件（包括子目录）
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch目录下所有.launch.py文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 安装子目录中的launch文件
        (os.path.join('share', package_name, 'launch', 'advanced'),
            glob('launch/advanced/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'sub_launch'),
            glob('launch/sub_launch/*.launch.py')),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vistar',
    maintainer_email='vistar@todo.todo',
    description='ROS2 Launch 系统学习包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 演示用节点：带参数的发布者
            'param_talker = launch_learning.param_talker:main',
            # 演示用节点：带命名空间的订阅者
            'ns_listener = launch_learning.ns_listener:main',
            # 演示用节点：可重启的工作节点
            'worker_node = launch_learning.worker_node:main',
        ],
    },
)
