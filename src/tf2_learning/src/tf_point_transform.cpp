/**
 * @file tf_point_transform.cpp
 * @brief ROS2 坐标点变换示例
 * 
 * 功能：将一个坐标系下的点变换到另一个坐标系下
 * 场景：传感器数据坐标转换、目标检测后的位置映射等
 * 
 * 知识点：
 * - tf2::fromMsg / tf2::toMsg 消息转换
 * - tf2::doTransform 点变换
 * - PointStamped 的使用
 * - 高级变换查询（带时间戳的变换）
 * 
 * 运行方式：
 *   # 终端1：运行动态广播器
 *   ros2 run tf2_learning dynamic_tf_broadcaster
 *   # 终端2：运行点变换示例
 *   ros2 run tf2_learning tf_point_transform
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // PointStamped的doTransform特化
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.h>

class TfPointTransform : public rclcpp::Node
{
public:
    TfPointTransform() : Node("tf_point_transform")
    {
        // 创建TF缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2秒查询一次
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TfPointTransform::transform_point, this));

        RCLCPP_INFO(this->get_logger(),
            "坐标点变换示例已启动: 将 rotating_sensor 中的点变换到 base_link 中");
    }

private:
    void transform_point()
    {
        // ============================================================
        // 1. 构造源坐标系中的点
        //    假设在旋转传感器坐标系中，前方1米处有一个障碍物
        // ============================================================
        geometry_msgs::msg::PointStamped sensor_point;
        sensor_point.header.frame_id = "rotating_sensor";    // 源坐标系
        sensor_point.header.stamp = this->now();              // 当前时间
        sensor_point.point.x = 1.0;   // 传感器前方1米
        sensor_point.point.y = 0.0;   // 无左右偏移
        sensor_point.point.z = 0.0;   // 无上下偏移

        try {
            // ============================================================
            // 2. 方法一：使用 doTransform 进行点变换（推荐）
            //    这是TF2提供的模板化变换函数
            //    支持的类型：PointStamped, PoseStamped, Vector3Stamped等
            //    底层自动进行坐标变换的矩阵运算
            // ============================================================
            geometry_msgs::msg::PointStamped base_point;
            tf2::doTransform(sensor_point, base_point,
                tf_buffer_->lookupTransform(
                    "base_link",            // 目标坐标系
                    "rotating_sensor",      // 源坐标系
                    sensor_point.header.stamp,  // 与点时间戳匹配
                    std::chrono::milliseconds(100)  // 超时等待100ms
                ));

            RCLCPP_INFO(this->get_logger(),
                "方法一(doTransform):\n"
                "  传感器坐标系中的点: (%.3f, %.3f, %.3f)\n"
                "  机器人坐标系中的点: (%.3f, %.3f, %.3f)",
                sensor_point.point.x, sensor_point.point.y, sensor_point.point.z,
                base_point.point.x, base_point.point.y, base_point.point.z);

            // ============================================================
            // 3. 方法二：手动获取变换矩阵，自行计算
            //    适用于需要批量变换多个点的场景
            //    获取变换后可以复用，避免重复查询Buffer
            // ============================================================
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform("base_link", "rotating_sensor", tf2::TimePointZero);

            // 手动计算（演示原理，实际开发推荐用doTransform）
            // 变换公式: p_target = R * p_source + t
            // 其中 R 是旋转矩阵，t 是平移向量
            double x_s = sensor_point.point.x;
            double y_s = sensor_point.point.y;
            double z_s = sensor_point.point.z;

            // 从四元数提取旋转矩阵（简化版，仅绕Z轴旋转的情况）
            double qx = transform.transform.rotation.x;
            double qy = transform.transform.rotation.y;
            double qz = transform.transform.rotation.z;
            double qw = transform.transform.rotation.w;

            // 旋转矩阵 R = I + 2*[q]x*[q]x + 2*qw*[q]x
            // 完整的3x3旋转矩阵展开：
            double r00 = 1.0 - 2.0*(qy*qy + qz*qz);  // R(0,0)
            double r01 = 2.0*(qx*qy - qw*qz);          // R(0,1)
            double r02 = 2.0*(qx*qz + qw*qy);          // R(0,2)
            double r10 = 2.0*(qx*qy + qw*qz);          // R(1,0)
            double r11 = 1.0 - 2.0*(qx*qx + qz*qz);   // R(1,1)
            double r12 = 2.0*(qy*qz - qw*qx);          // R(1,2)
            double r20 = 2.0*(qx*qz - qw*qy);          // R(2,0)
            double r21 = 2.0*(qy*qz + qw*qx);          // R(2,1)
            double r22 = 1.0 - 2.0*(qx*qx + qy*qy);   // R(2,2)

            // p_target = R * p_source + t
            double x_t = r00*x_s + r01*y_s + r02*z_s + transform.transform.translation.x;
            double y_t = r10*x_s + r11*y_s + r12*z_s + transform.transform.translation.y;
            double z_t = r20*x_s + r21*y_s + r22*z_s + transform.transform.translation.z;

            RCLCPP_INFO(this->get_logger(),
                "方法二(手动矩阵):\n"
                "  机器人坐标系中的点: (%.3f, %.3f, %.3f)",
                x_t, y_t, z_t);

        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "点变换失败: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfPointTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
