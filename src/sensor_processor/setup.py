from setuptools import find_packages, setup

package_name = 'sensor_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vistar',
    maintainer_email='1055311345@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = sensor_processor.my_node:main',
        ],
    },
)
