"""⑧ 机器人传感器驱动 —— 生命周期控制的 IMU 驱动"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle_learning_cpp',
            executable='robot_sensor_driver',
            name='imu_driver',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_path': '/dev/imu0',
                'baudrate': 115200,
                'publish_rate': 100,
                'frame_id': 'imu_link',
            }],
        ),
    ])
