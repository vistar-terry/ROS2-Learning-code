/**
 * @file tf_listener.cpp
 * @brief ROS2 坐标监听器示例
 * 
 * 功能：监听TF变换，查询两个坐标系之间的变换关系
 * 场景：获取传感器数据在机器人坐标系下的位置、路径规划中的坐标转换等
 * 
 * 知识点：
 * - TransformListener 的使用
 * - BufferCore 缓冲区的工作机制
 * - lookupTransform 查询变换
 * - canTransform 检查变换是否可用
 * - 超时等待机制
 * - TF异常处理
 * 
 * 运行方式：
 *   # 终端1：运行动态广播器
 *   ros2 run tf2_learning dynamic_tf_broadcaster
 *   # 终端2：运行监听器
 *   ros2 run tf2_learning tf_listener
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

class TfListener : public rclcpp::Node
{
public:
    TfListener() : Node("tf_listener")
    {
        // ============================================================
        // 1. 创建TF缓冲区和监听器
        //    - Buffer：存储所有接收到的TF变换，支持时间插值
        //      默认缓存时间为10秒，可通过参数调整
        //    - Listener：自动订阅 /tf 和 /tf_static 话题，
        //      将接收到的变换存入Buffer
        // ============================================================
        
        // 创建TF缓冲区，设置缓存时间为10秒（默认值）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        // 创建TF监听器，自动订阅TF话题并填充缓冲区
        // 监听器的生命周期必须与缓冲区一致
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ============================================================
        // 2. 创建定时器，周期性查询TF变换
        //    这里用1Hz的频率查询，实际应用中根据需求调整
        // ============================================================
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TfListener::lookup_transform, this));

        RCLCPP_INFO(this->get_logger(), "TF监听器已启动，正在查询 base_link -> rotating_sensor 变换...");
    }

private:
    void lookup_transform()
    {
        // ============================================================
        // 3. 检查变换是否可用（非阻塞方式）
        //    canTransform 只检查当前缓冲区中是否有变换数据
        //    不会等待数据到达
        // ============================================================
        if (!tf_buffer_->canTransform("base_link", "rotating_sensor", tf2::TimePointZero)) {
            RCLCPP_WARN(this->get_logger(), 
                "变换 base_link -> rotating_sensor 尚不可用，等待中...");
            return;
        }

        try {
            // ============================================================
            // 4. 查询最新变换
            //    lookupTransform 是TF2的核心API
            //    参数说明：
            //      - target_frame: 目标坐标系（要变换到的坐标系）
            //      - source_frame: 源坐标系（要变换的坐标系）  
            //      - time: 查询时间点，tf2::TimePointZero表示获取最新
            //    返回值：TransformStamped消息，包含平移和旋转
            // ============================================================
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("base_link", "rotating_sensor", tf2::TimePointZero);

            // ============================================================
            // 5. 提取变换信息
            // ============================================================
            RCLCPP_INFO(this->get_logger(),
                "变换 base_link -> rotating_sensor:\n"
                "  平移: x=%.3f, y=%.3f, z=%.3f\n"
                "  旋转: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);

        }
        catch (const tf2::TransformException& ex) {
            // ============================================================
            // 6. 异常处理
            //    常见异常类型：
            //    - tf2::LookupException: 坐标系不存在
            //    - tf2::ConnectivityException: 坐标系之间没有连接路径
            //    - tf2::ExtrapolationException: 请求的时间超出缓存范围
            //    - tf2::InvalidArgumentException: 参数无效
            // ============================================================
            RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                       // TF缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;           // TF监听器
    rclcpp::TimerBase::SharedPtr timer_;                                 // 定时器
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
