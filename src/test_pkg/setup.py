from setuptools import find_packages, setup

package_name = 'test_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "publisher = " + package_name + ".publisher:main",
            "subscriber = " + package_name + ".subscriber:main",
            "service = " + package_name + ".service:main",
            "client = " + package_name + ".client:main",
        ],
    },
)
