from setuptools import find_packages, setup

package_name = 'hello_world_parameter_py'

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
    maintainer_email='1055311345@qq.com',
    description='Python implementation of parameter management in ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            "parameter_manager = " + package_name + ".parameter_manager:main",
            "business_node = " + package_name + ".business_node:main",
            "single_param_node = " + package_name + ".single_param_node:main",
            "dynamic_param_modifier = " + package_name + ".dynamic_param_modifier:main",
        ],
    },
)
