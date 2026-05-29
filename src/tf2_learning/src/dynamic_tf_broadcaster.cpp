/**
 * @file dynamic_tf_broadcaster.cpp
 * @brief ROS2 动态坐标广播器示例
 * 
 * 功能：发布一个随时间变化的坐标变换关系
 * 场景：旋转的雷达、摆动的机械臂关节、移动的机器人等
 * 
 * 知识点：
 * - TransformBroadcaster 的使用（动态广播器）
 * - 定时器驱动的TF发布
 * - 欧拉角与四元数的动态转换
 * - TF发布的最佳实践（频率选择）
 * 
 * 运行方式：
 *   ros2 run tf2_learning dynamic_tf_broadcaster
 * 
 * 验证方式：
 *   ros2 run tf2_ros tf2_echo base_link rotating_sensor
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class DynamicTfBroadcaster : public rclcpp::Node
{
public:
    DynamicTfBroadcaster() : Node("dynamic_tf_broadcaster"), angle_(0.0)
    {
        // ============================================================
        // 1. 创建动态坐标广播器
        //    动态广播器每次调用sendTransform都会发布新的TF
        //    与静态广播器的区别：动态TF不会持久缓存，需要持续发布
        // ============================================================
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // ============================================================
        // 2. 创建定时器，周期性发布TF
        //    推荐频率：10~100Hz
        //    - 10Hz：低频场景（缓慢移动的物体）
        //    - 50Hz：常用频率（导航、SLAM等）
        //    - 100Hz：高精度控制场景
        //    注意：频率越高，CPU开销越大
        // ============================================================
        // 50ms = 20Hz 发布频率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DynamicTfBroadcaster::publish_transform, this));

        RCLCPP_INFO(this->get_logger(),
            "动态TF广播器已启动: base_link -> rotating_sensor (20Hz)");
    }

private:
    void publish_transform()
    {
        // ============================================================
        // 3. 构造并发布动态变换
        //    每次调用都会覆盖之前发布的同一对坐标系的变换
        // ============================================================
        geometry_msgs::msg::TransformStamped transform_stamped;

        // 设置时间戳（重要！TF2依赖时间戳进行插值）
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "base_link";           // 父坐标系
        transform_stamped.child_frame_id = "rotating_sensor";      // 子坐标系

        // ============================================================
        // 模拟一个绕Z轴旋转的传感器
        // 旋转半径0.5m，高度0.3m，角速度0.5rad/s
        // ============================================================
        double radius = 0.5;      // 旋转半径（米）
        double height = 0.3;      // 安装高度（米）
        double angular_speed = 0.5; // 角速度（弧度/秒）

        // 计算当前位置（圆周运动）
        transform_stamped.transform.translation.x = radius * cos(angle_);
        transform_stamped.transform.translation.y = radius * sin(angle_);
        transform_stamped.transform.translation.z = height;

        // 计算当前姿态（传感器始终朝向圆心外侧）
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, angle_);   // yaw角随时间变化
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // 发布变换
        tf_broadcaster_->sendTransform(transform_stamped);

        // 更新角度（时间步进 * 角速度）
        // 50ms定时器 = 0.05s步进
        angle_ += 0.05 * angular_speed;

        // 保持角度在 [-π, π] 范围内，避免数值溢出
        if (angle_ > M_PI) {
            angle_ -= 2 * M_PI;
        }
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;   // 动态TF广播器
    rclcpp::TimerBase::SharedPtr timer_;                                // 定时器
    double angle_;                                                       // 当前旋转角度
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTfBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
