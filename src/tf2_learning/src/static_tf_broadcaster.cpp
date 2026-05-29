/**
 * @file static_tf_broadcaster.cpp
 * @brief ROS2 静态坐标广播器示例
 * 
 * 功能：发布一个固定的坐标变换关系（不会随时间变化）
 * 场景：传感器安装位置、机器人底盘与轮子的固定关系等
 * 
 * 知识点：
 * - StaticTransformBroadcaster 的使用
 * - TransformStamped 消息的构造
 * - 四元数与欧拉角的关系
 * 
 * 运行方式：
 *   ros2 run tf2_learning static_tf_broadcaster
 * 
 * 验证方式：
 *   ros2 run tf2_ros tf2_echo base_link sensor_lidar
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class StaticTfBroadcaster : public rclcpp::Node
{
public:
    StaticTfBroadcaster() : Node("static_tf_broadcaster")
    {
        // ============================================================
        // 1. 创建静态坐标广播器
        //    静态广播器只会发送一次TF，之后TF系统会一直缓存这个变换
        //    即使节点退出，TF仍然有效（直到ROS2关闭）
        // ============================================================
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

        // 发布一个从 base_link 到 sensor_lidar 的静态变换
        // 模拟：激光雷达安装在机器人前方0.3m，上方0.2m，无旋转
        publish_static_transform();

        RCLCPP_INFO(this->get_logger(),
            "静态TF广播器已启动: base_link -> sensor_lidar "
            "[x=0.3, y=0.0, z=0.2, roll=0, pitch=0, yaw=0]");
    }

private:
    void publish_static_transform()
    {
        // ============================================================
        // 2. 构造 TransformStamped 消息
        //    这是TF2中最核心的消息类型，描述了两个坐标系之间的变换
        // ============================================================
        geometry_msgs::msg::TransformStamped transform_stamped;

        // --- 头信息 ---
        transform_stamped.header.stamp = this->now();              // 时间戳
        transform_stamped.header.frame_id = "base_link";           // 父坐标系（参考坐标系）
        transform_stamped.child_frame_id = "sensor_lidar";         // 子坐标系（被描述的坐标系）

        // --- 平移部分 ---
        // 表示子坐标系原点在父坐标系中的位置
        transform_stamped.transform.translation.x = 0.3;           // 前方0.3米
        transform_stamped.transform.translation.y = 0.0;           // 左右偏移0
        transform_stamped.transform.translation.z = 0.2;           // 上方0.2米

        // --- 旋转部分（四元数） ---
        // 四元数是描述旋转的数学工具，避免万向锁问题
        // 这里从欧拉角(roll, pitch, yaw)转换为四元数
        // roll: 绕X轴旋转; pitch: 绕Y轴旋转; yaw: 绕Z轴旋转
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);  // 无旋转
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // ============================================================
        // 3. 发布静态变换
        //    与动态广播器不同，静态广播器只需发布一次
        // ============================================================
        tf_static_broadcaster_->sendTransform(transform_stamped);
    }

    // 静态坐标广播器智能指针
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTfBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
