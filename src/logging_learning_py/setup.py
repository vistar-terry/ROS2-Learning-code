from setuptools import find_packages, setup

package_name = 'logging_learning_py'

setup(
    name=package_name,
    version='0.1.0',
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
    description='ROS2 logging system learning package (Python)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'logging_basic = logging_learning_py.logging_basic:main',
            'logging_levels = logging_learning_py.logging_levels:main',
            'logging_advanced = logging_learning_py.logging_advanced:main',
            'logging_robot_demo = logging_learning_py.logging_robot_demo:main',
        ],
    },
)
