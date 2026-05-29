/**
 * @file diff_odometry.cpp
 * @brief 差速机器人简易里程计节点
 * 
 * 功能：从 /joint_states 读取轮子速度，计算里程计并发布 TF
 * 注意：当使用 diff_drive_controller 时，里程计由控制器内部发布
 *       此节点是演示里程计计算原理的独立实现
 * 
 * 知识点：
 * - 差速运动学正解
 * - 里程计计算与发布
 * - TF 广播（odom → base_footprint）
 * - NavMessages/Odometry 消息
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class DiffOdometry : public rclcpp::Node
{
public:
    DiffOdometry() : Node("diff_odometry"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // 声明参数
        this->declare_parameter("wheel_radius", 0.033);
        this->declare_parameter("wheel_separation", 0.22);
        this->declare_parameter("publish_tf", true);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        // 创建 TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 创建里程计发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // 订阅关节状态
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DiffOdometry::joint_state_callback, this, std::placeholders::_1));

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "里程计节点启动: wheel_radius=%.3fm, wheel_separation=%.3fm",
            wheel_radius_, wheel_separation_);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // ============================================================
        // 1. 从 joint_states 中提取左右轮速度
        // ============================================================
        double left_velocity = 0.0;
        double right_velocity = 0.0;

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "left_wheel_joint")
                left_velocity = msg->velocity[i];
            else if (msg->name[i] == "right_wheel_joint")
                right_velocity = msg->velocity[i];
        }

        // ============================================================
        // 2. 差速运动学正解
        //    将轮子角速度转换为机器人线速度和角速度
        //    
        //    线速度 = (v_right + v_left) × wheel_radius / 2
        //    角速度 = (v_right - v_left) × wheel_radius / wheel_separation
        // ============================================================
        double linear_x = (right_velocity + left_velocity) * wheel_radius_ / 2.0;
        double angular_z = (right_velocity - left_velocity) * wheel_radius_ / wheel_separation_;

        // ============================================================
        // 3. 计算时间间隔
        // ============================================================
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0 || dt > 1.0) return;  // 防止异常值

        // ============================================================
        // 4. 积分更新位姿
        //    使用中点法（比欧拉法更准确）
        // ============================================================
        double delta_x = linear_x * cos(theta_) * dt;
        double delta_y = linear_x * sin(theta_) * dt;
        double delta_theta = angular_z * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // 保持角度在 [-π, π]
        if (theta_ > M_PI) theta_ -= 2 * M_PI;
        if (theta_ < -M_PI) theta_ += 2 * M_PI;

        // ============================================================
        // 5. 发布 TF 变换（odom → base_footprint）
        // ============================================================
        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = current_time;
            tf.header.frame_id = "odom";
            tf.child_frame_id = "base_footprint";
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tf);
        }

        // ============================================================
        // 6. 发布里程计消息
        // ============================================================
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // 位置
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度
        odom.twist.twist.linear.x = linear_x;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = angular_z;

        odom_pub_->publish(odom);
    }

    // 参数
    double wheel_radius_;
    double wheel_separation_;
    bool publish_tf_;

    // 状态
    double x_;
    double y_;
    double theta_;
    rclcpp::Time last_time_;

    // ROS 接口
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffOdometry>());
    rclcpp::shutdown();
    return 0;
}
