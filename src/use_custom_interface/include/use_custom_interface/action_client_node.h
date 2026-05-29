#ifndef __HELLO_WORLD_INTERFACE_ACTION_CLIENT_H__
#define __HELLO_WORLD_INTERFACE_ACTION_CLIENT_H__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <hello_world_interface/action/navigate_to_goal.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

using namespace std::placeholders;

class CActionClientNode : public rclcpp::Node
{
public:
    using NavigateToGoal = hello_world_interface::action::NavigateToGoal;
    using GoalHandleNavigateToGoal = rclcpp_action::ClientGoalHandle<NavigateToGoal>;

    CActionClientNode();
    ~CActionClientNode();

    bool isGoalDone() const;
    void sendGoal();

private:
    void goalResponseCallback(const GoalHandleNavigateToGoal::SharedPtr future);

    void feedbackCallback(
        GoalHandleNavigateToGoal::SharedPtr,
        const std::shared_ptr<const NavigateToGoal::Feedback> feedback);

    void resultCallback(const GoalHandleNavigateToGoal::WrappedResult &result);

private:
    rclcpp_action::Client<NavigateToGoal>::SharedPtr m_actionClient;
    rclcpp::TimerBase::SharedPtr m_timer;
    bool m_goalDone;
};

#endif // __HELLO_WORLD_INTERFACE_ACTION_CLIENT_H__