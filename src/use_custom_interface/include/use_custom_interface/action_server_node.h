#ifndef __HELLO_WORLD_INTERFACE_ACTION_SERVER_H__
#define __HELLO_WORLD_INTERFACE_ACTION_SERVER_H__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <hello_world_interface/action/navigate_to_goal.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

// 位姿结构体
struct Pose2D
{
    double x;
    double y;
    double theta; // 朝向角度（弧度）

    Pose2D(double x = 0, double y = 0, double theta = 0)
        : x(x), y(y), theta(theta) {}

    // 打印位姿信息
    void print() const
    {
        RCLCPP_INFO(rclcpp::get_logger("action_server_node"), "(%.3lf, %.3lf, %.1lf°)", x, y, theta * 180 / M_PI);
    }

    // 计算两点之间的距离
    double distanceTo(const Pose2D &other) const
    {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }
};

class CActionServerNode : public rclcpp::Node
{
public:
    using NavigateToGoal = hello_world_interface::action::NavigateToGoal;
    using GoalHandleNavigateToGoal = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

    explicit CActionServerNode();
    virtual ~CActionServerNode();

private:
    rclcpp_action::GoalResponse addAction(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateToGoal::Goal> goal);

    rclcpp_action::CancelResponse cancelAction(
        const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle);

    void executeAction(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle);

    void execute(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle);

private:
    Pose2D m_goalPose;
    Pose2D m_currentPose;
    rclcpp_action::Server<NavigateToGoal>::SharedPtr m_actionServer;
};

// 路径规划类，模拟机器人直线运动
class LinearPathPlanner
{
public:
    LinearPathPlanner(const Pose2D &start, const Pose2D &end, double duration = 10.0)
        : m_start(start), m_end(end), m_duration(duration)
    {
        m_totalDistance = m_start.distanceTo(m_end);

        // 计算朝向角度（指向目标点的方向）
        double dx = m_end.x - m_start.x;
        double dy = m_end.y - m_start.y;
        m_start.theta = std::atan2(dy, dx);
        m_end.theta = m_start.theta; // 保持相同朝向
    }

    // 根据时间t（0到duration）计算当前位置
    Pose2D getPoseAtTime(double t)
    {
        if (t <= 0)
            return m_start;
        if (t >= m_duration)
            return m_end;

        // 线性插值
        double ratio = t / m_duration;
        double x = m_start.x + (m_end.x - m_start.x) * ratio;
        double y = m_start.y + (m_end.y - m_start.y) * ratio;
        m_current = Pose2D(x, y, m_start.theta);

        return m_current;
    }

    double getDuration() const { return m_duration; }
    double getTotalDistance() const { return m_totalDistance; }
    double getRemainDistance() const
    {
        double dx = m_current.x - m_end.x;
        double dy = m_current.y - m_end.y;

        return std::sqrt(dx * dx + dy * dy);
    }

    void printPathInfo() const
    {
        RCLCPP_INFO(rclcpp::get_logger("action_server_node"), "start: ");
        m_start.print();
        RCLCPP_INFO(rclcpp::get_logger("action_server_node"), "end: ");
        m_end.print();
        RCLCPP_INFO(rclcpp::get_logger("action_server_node"), "Total Distance: %.3lfm", m_totalDistance);
        RCLCPP_INFO(rclcpp::get_logger("action_server_node"), "Total Time: %.1lfs", m_duration);
        RCLCPP_INFO(rclcpp::get_logger("action_server_node"), "Avg Speed: %.3lfm/s", m_totalDistance / m_duration);
    }

private:
    Pose2D m_start;
    Pose2D m_end;
    Pose2D m_current;
    double m_totalDistance;
    double m_duration; // 总运动时间（秒）
};

#endif // __HELLO_WORLD_INTERFACE_ACTION_SERVER_H__