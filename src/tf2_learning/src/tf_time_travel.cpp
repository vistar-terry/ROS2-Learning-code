/**
 * @file tf_time_travel.cpp
 * @brief ROS2 TF时间旅行示例
 * 
 * 功能：演示如何查询特定时间点的TF变换
 * 场景：回放bag数据时查询历史变换、多传感器时间同步等
 * 
 * 知识点：
 * - TF2的时间插值机制
 * - 带时间戳的lookupTransform
 * - 高级变换API：advanced API（指定源时间和目标时间）
 * - ExtrapolationException异常处理
 * 
 * 运行方式：
 *   # 终端1：运行动态广播器
 *   ros2 run tf2_learning dynamic_tf_broadcaster
 *   # 终端2：运行时间旅行示例
 *   ros2 run tf2_learning tf_time_travel
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <chrono>
#include <builtin_interfaces/msg/time.hpp>

class TfTimeTravel : public rclcpp::Node
{
public:
    TfTimeTravel() : Node("tf_time_travel"), count_(0)
    {
        // 创建TF缓冲区，缓存时间设为30秒（比默认10秒更长，以便查询更早的变换）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock(),
            std::chrono::seconds(30),  // 缓存30秒
            this);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 每3秒执行一次查询演示
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&TfTimeTravel::demo_time_travel, this));

        RCLCPP_INFO(this->get_logger(), "TF时间旅行示例已启动");
    }

private:
    void demo_time_travel()
    {
        count_++;
        RCLCPP_INFO(this->get_logger(), "\n========== 第%d次查询 ==========", count_);

        // ============================================================
        // 1. 查询最新变换（最常用的方式）
        //    tf2::TimePointZero 表示获取缓冲区中最新的变换
        // ============================================================
        try {
            auto latest_tf = tf_buffer_->lookupTransform(
                "base_link", "rotating_sensor", tf2::TimePointZero);
            RCLCPP_INFO(this->get_logger(),
                "[最新变换] x=%.3f, y=%.3f",
                latest_tf.transform.translation.x,
                latest_tf.transform.translation.y);
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "查询最新变换失败: %s", ex.what());
            return;
        }

        // ============================================================
        // 2. 查询2秒前的变换
        //    TF2会在缓存中进行时间插值
        //    使用 rclcpp::Time 进行时间计算
        // ============================================================
        try {
            rclcpp::Time now = this->now();
            rclcpp::Time two_sec_ago = now - rclcpp::Duration(2, 0);  // 2秒前

            auto past_tf = tf_buffer_->lookupTransform(
                "base_link", "rotating_sensor", two_sec_ago);
            RCLCPP_INFO(this->get_logger(),
                "[2秒前的变换] x=%.3f, y=%.3f",
                past_tf.transform.translation.x,
                past_tf.transform.translation.y);
        }
        catch (const tf2::ExtrapolationException& ex) {
            // 时间超出缓存范围时抛出此异常
            RCLCPP_WARN(this->get_logger(),
                "2秒前的变换超出缓存范围: %s", ex.what());
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "查询历史变换失败: %s", ex.what());
        }

        // ============================================================
        // 3. 高级API：指定源时间和目标时间的变换
        //    这在多传感器融合中非常有用
        //    例如：获取"在时间t1时，从frame_A到frame_B的变换，
        //          但使用时间t2时的frame_A作为参考"
        //    
        //    API: lookupTransform(target_frame, target_time,
        //                        source_frame, source_time,
        //                        fixed_frame, timeout)
        //    
        //    含义：在fixed_frame坐标系下，
        //          将source_frame在source_time时刻的位置
        //          变换到target_frame在target_time时刻的位置
        // ============================================================
        try {
            rclcpp::Time now = this->now();
            rclcpp::Time one_sec_ago = now - rclcpp::Duration(1, 0);

            auto advanced_tf = tf_buffer_->lookupTransform(
                "base_link",           // 目标坐标系
                now,                   // 目标时间
                "rotating_sensor",     // 源坐标系
                one_sec_ago,           // 源时间（1秒前）
                "odom",               // 固定参考坐标系
                std::chrono::milliseconds(100));  // 超时

            RCLCPP_INFO(this->get_logger(),
                "[高级API变换] 在odom参考系下，将1秒前rotating_sensor中的点"
                "变换到当前base_link中: x=%.3f, y=%.3f",
                advanced_tf.transform.translation.x,
                advanced_tf.transform.translation.y);
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "高级API查询失败: %s", ex.what());
        }

        // ============================================================
        // 4. canTransform 带时间检查
        //    在执行lookupTransform之前，先检查变换是否可用
        //    注意：canTransform 的错误信息版本需要使用 tf2::TimePoint
        // ============================================================
        rclcpp::Time five_sec_ago = this->now() - rclcpp::Duration(5, 0);
        std::string error_msg;
        // 将 rclcpp::Time 转换为 tf2::TimePoint
        tf2::TimePoint tf2_five_sec_ago = tf2_ros::fromRclcpp(five_sec_ago);
        if (tf_buffer_->canTransform("base_link", "rotating_sensor",
            tf2_five_sec_ago, &error_msg)) {
            RCLCPP_INFO(this->get_logger(), "5秒前的变换可用");
        } else {
            RCLCPP_INFO(this->get_logger(),
                "5秒前的变换不可用: %s", error_msg.c_str());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfTimeTravel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
