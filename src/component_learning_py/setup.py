from setuptools import find_packages, setup

package_name = 'component_learning_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vistar',
    maintainer_email='vistar@todo.todo',
    description='ROS2 component mechanism learning package (Python)',
    license='Apache-2.0',
    # ★ 同时注册为独立可执行节点
    entry_points={
        'console_scripts': [
            'sensor_source = component_learning_py.sensor_source_component:main',
            'data_filter = component_learning_py.data_filter_component:main',
            'data_display = component_learning_py.data_display_component:main',
        ],
    },
)
