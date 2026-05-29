# setuptools 用于简化包的构建、分发和安装过程
from setuptools import find_packages, setup

# 包名称
package_name = 'hello_world_py'

# 调用setup配置包信息
setup(
    name=package_name, # 包名称
    version='0.0.0', # 版本号，主版本号.次版本号.修订号
    # 自动发现项目中的包。exclude=['test']参数告诉find_packages忽略名为test的目录
    packages=find_packages(exclude=['test']), 
    # 指定与包一起安装的数据文件
    # 元组第一项：目标安装路径
    # 元组第二项：要安装到目标路径的文件路径列表。这些路径是相对于setup.py文件所在目录的。
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], # 包安装时所需的依赖项
    zip_safe=True, # 指定包是否可以作为zip文件安全地安装和运行
    maintainer='vistar', # 指定包的维护者姓名
    maintainer_email='vistar@todo.todo', # 指定维护者的电子邮件地址
    description='TODO: Package description', # 包的简短描述
    license='Apache-2.0', # 包的许可证类型
    # tests_require=['pytest'], # 指定运行测试所需的依赖项
    # 定义入口点，即将包内的函数或类暴露为命令行工具
    entry_points={
        'console_scripts': [
            # 定义一个名为talker的命令行工具，映射到hello_world_py.publisher模块中的main函数
            "talker = " + package_name + ".publisher:main",
        ],
    },
)