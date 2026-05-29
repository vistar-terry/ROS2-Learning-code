/**
 * @file robot_tf_broadcaster.cpp
 * @brief ROS2 机器人TF树广播器示例
 * 
 * 功能：模拟一个移动机器人的完整TF树
 * 场景：实际机器人中，各种传感器和执行器相对于底盘的坐标关系
 * 
 * TF树结构：
 *                          map
 *                           |
 *                        odom (通常由轮式里程计发布)
 *                           |
 *                       base_link (机器人底盘中心)
 *                      /    |    \     \
 *               base_footprint  |   imu_link  camera_link
 *                              |
 *                          laser_link
 * 
 * 知识点：
 * - 多级TF树的设计
 * - 静态+动态混合TF发布
 * - 机器人常用坐标系命名规范
 * - REP-105 坐标系约定
 * 
 * 运行方式：
 *   ros2 run tf2_learning robot_tf_broadcaster
 * 
 * 可视化：
 *   ros2 run rviz2 rviz2 -d <path_to_rviz_config>
 *   或查看TF树：ros2 run tf2_tools view_frames
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class RobotTfBroadcaster : public rclcpp::Node
{
public:
    RobotTfBroadcaster() : Node("robot_tf_broadcaster"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // ============================================================
        // 创建广播器
        // ============================================================
        // 动态广播器：用于发布随时间变化的变换（机器人位姿）
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        // 静态广播器：用于发布固定的变换（传感器安装位置）
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

        // ============================================================
        // 发布所有静态变换（传感器安装位置，不会变化）
        // 这些变换只需要发布一次
        // ============================================================
        publish_static_transforms();

        // ============================================================
        // 创建定时器，周期性发布动态变换（机器人运动）
        // 20Hz发布频率，适合导航场景
        // ============================================================
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RobotTfBroadcaster::publish_dynamic_transforms, this));

        RCLCPP_INFO(this->get_logger(),
            "机器人TF树广播器已启动，模拟机器人圆周运动");
    }

private:
    /**
     * 发布所有静态变换
     * 模拟一个差速驱动机器人的传感器安装位置
     */
    void publish_static_transforms()
    {
        // ============================================================
        // REP-105 坐标系约定（ROS标准）：
        // - base_link: 机器人底盘中心，Z轴朝上
        // - base_footprint: base_link在地面的投影（Z=0）
        // - imu_link: IMU传感器安装位置
        // - laser_link: 激光雷达安装位置
        // - camera_link: 摄像头安装位置
        // ============================================================

        std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

        // --- base_link -> base_footprint ---
        // base_footprint是base_link在地面上的投影
        // 底盘高度0.1m
        auto tf_base_footprint = create_transform(
            "base_link", "base_footprint",
            0.0, 0.0, -0.1,   // 向下0.1m（地面）
            0.0, 0.0, 0.0);   // 无旋转
        static_transforms.push_back(tf_base_footprint);

        // --- base_link -> imu_link ---
        // IMU安装在底盘中心偏前
        auto tf_imu = create_transform(
            "base_link", "imu_link",
            0.15, 0.0, 0.05,  // 前方0.15m，上方0.05m
            0.0, 0.0, 0.0);
        static_transforms.push_back(tf_imu);

        // --- base_link -> laser_link ---
        // 激光雷达安装在底盘顶部中心
        auto tf_laser = create_transform(
            "base_link", "laser_link",
            0.0, 0.0, 0.15,   // 正上方0.15m
            0.0, 0.0, 0.0);
        static_transforms.push_back(tf_laser);

        // --- base_link -> camera_link ---
        // 摄像头安装在前方，稍微朝下倾斜
        auto tf_camera = create_transform(
            "base_link", "camera_link",
            0.2, 0.0, 0.12,   // 前方0.2m，上方0.12m
            0.0, -0.349, 0.0);  // pitch向下倾斜20度（0.349 rad ≈ 20°）
        static_transforms.push_back(tf_camera);

        // 一次性发布所有静态变换
        tf_static_broadcaster_->sendTransform(static_transforms);

        RCLCPP_INFO(this->get_logger(),
            "已发布 %zu 个静态变换: base_footprint, imu_link, laser_link, camera_link",
            static_transforms.size());
    }

    /**
     * 发布动态变换（模拟机器人运动）
     * 这里模拟机器人在地图上做圆周运动
     */
    void publish_dynamic_transforms()
    {
        // ============================================================
        // 模拟机器人运动参数
        // ============================================================
        double linear_speed = 0.2;      // 线速度 0.2 m/s
        double angular_speed = 0.1;     // 角速度 0.1 rad/s
        double dt = 0.05;               // 时间步长（50ms）

        // ============================================================
        // 更新机器人位姿（简单的差速运动学模型）
        // dx = v * cos(θ) * dt
        // dy = v * sin(θ) * dt
        // dθ = ω * dt
        // ============================================================
        x_ += linear_speed * cos(theta_) * dt;
        y_ += linear_speed * sin(theta_) * dt;
        theta_ += angular_speed * dt;

        // 保持角度在 [-π, π]
        if (theta_ > M_PI) theta_ -= 2 * M_PI;

        // ============================================================
        // 发布 map -> odom 变换
        // 在实际机器人中，这个变换通常由定位节点发布（如AMCL）
        // 这里我们简化为静态变换（假设无定位漂移）
        // ============================================================
        geometry_msgs::msg::TransformStamped tf_map_odom;
        tf_map_odom.header.stamp = this->now();
        tf_map_odom.header.frame_id = "map";
        tf_map_odom.child_frame_id = "odom";
        tf_map_odom.transform.translation.x = 0.0;
        tf_map_odom.transform.translation.y = 0.0;
        tf_map_odom.transform.translation.z = 0.0;
        tf2::Quaternion q_map;
        q_map.setRPY(0, 0, 0);
        // 手动赋值四元数字段（tf2::Quaternion → geometry_msgs::msg::Quaternion）
        tf_map_odom.transform.rotation.x = q_map.x();
        tf_map_odom.transform.rotation.y = q_map.y();
        tf_map_odom.transform.rotation.z = q_map.z();
        tf_map_odom.transform.rotation.w = q_map.w();

        // ============================================================
        // 发布 odom -> base_link 变换
        // 在实际机器人中，这个变换通常由轮式里程计发布
        // 这里使用模拟的运动数据
        // ============================================================
        geometry_msgs::msg::TransformStamped tf_odom_base;
        tf_odom_base.header.stamp = this->now();
        tf_odom_base.header.frame_id = "odom";
        tf_odom_base.child_frame_id = "base_link";
        tf_odom_base.transform.translation.x = x_;
        tf_odom_base.transform.translation.y = y_;
        tf_odom_base.transform.translation.z = 0.0;   // 2D运动，z始终为0
        tf2::Quaternion q_base;
        q_base.setRPY(0, 0, theta_);
        // 手动赋值四元数字段
        tf_odom_base.transform.rotation.x = q_base.x();
        tf_odom_base.transform.rotation.y = q_base.y();
        tf_odom_base.transform.rotation.z = q_base.z();
        tf_odom_base.transform.rotation.w = q_base.w();

        // 同时发布两个动态变换
        tf_broadcaster_->sendTransform({tf_map_odom, tf_odom_base});
    }

    /**
     * 辅助函数：创建TransformStamped消息
     */
    geometry_msgs::msg::TransformStamped create_transform(
        const std::string& parent, const std::string& child,
        double x, double y, double z,
        double roll, double pitch, double yaw)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        // 手动赋值四元数字段
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        return t;
    }

    // 广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 机器人位姿状态
    double x_;       // X坐标（米）
    double y_;       // Y坐标（米）
    double theta_;   // 朝向角（弧度）
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotTfBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
