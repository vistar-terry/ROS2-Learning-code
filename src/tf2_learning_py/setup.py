from setuptools import find_packages, setup

package_name = 'tf2_learning_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (package_name + '/launch', ['launch/tf2_demo_py.launch.py']),
        # 安装rviz配置
        (package_name + '/rviz', ['rviz/robot_tf_py.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vistar',
    maintainer_email='vistar@todo.todo',
    description='ROS2 TF2 坐标变换学习包 - Python实现',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 静态坐标广播器
            'static_tf_broadcaster_py = tf2_learning_py.static_tf_broadcaster:main',
            # 动态坐标广播器
            'dynamic_tf_broadcaster_py = tf2_learning_py.dynamic_tf_broadcaster:main',
            # 坐标监听器
            'tf_listener_py = tf2_learning_py.tf_listener:main',
            # 坐标点变换
            'tf_point_transform_py = tf2_learning_py.tf_point_transform:main',
            # 机器人TF树广播器
            'robot_tf_broadcaster_py = tf2_learning_py.robot_tf_broadcaster:main',
            # TF时间旅行
            'tf_time_travel_py = tf2_learning_py.tf_time_travel:main',
        ],
    },
)
