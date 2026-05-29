#include "use_custom_interface/action_client_node.h"

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

using namespace std::placeholders;

CActionClientNode::CActionClientNode()
    : Node("action_client_node")
    , m_goalDone(false)
{
    this->m_actionClient = rclcpp_action::create_client<NavigateToGoal>(this, "/useInterface/action/navigateToGoal");

    this->m_timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CActionClientNode::sendGoal, this));
}

CActionClientNode::~CActionClientNode()
{
}

bool CActionClientNode::isGoalDone() const
{
    return this->m_goalDone;
}

void CActionClientNode::sendGoal()
{
    this->m_timer->cancel();
    this->m_goalDone = false;

    if (!this->m_actionClient->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->m_goalDone = true;
        return;
    }

    auto goalMsg = NavigateToGoal::Goal();
    goalMsg.target_pose.x = 10.5;
    goalMsg.target_pose.y = 6.7;
    goalMsg.target_pose.theta = 3.14;

    RCLCPP_INFO(this->get_logger(), "Sending goal...");

    auto sendGoalOptions = rclcpp_action::Client<NavigateToGoal>::SendGoalOptions();
    sendGoalOptions.goal_response_callback = std::bind(&CActionClientNode::goalResponseCallback, this, _1);
    sendGoalOptions.feedback_callback = std::bind(&CActionClientNode::feedbackCallback, this, _1, _2);
    sendGoalOptions.result_callback = std::bind(&CActionClientNode::resultCallback, this, _1);
    this->m_actionClient->async_send_goal(goalMsg, sendGoalOptions);
}

void CActionClientNode::goalResponseCallback(const GoalHandleNavigateToGoal::SharedPtr future)
{
    auto goalHandle = future.get();
    if (!goalHandle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void CActionClientNode::feedbackCallback(
    GoalHandleNavigateToGoal::SharedPtr,
    const std::shared_ptr<const NavigateToGoal::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Current Pose:(%.3lf, %.3lf, %.1lf°), To Goal: %.3fm, Complete: %d%%",
            feedback->current_pose.x, feedback->current_pose.y, feedback->current_pose.theta * 180 / M_PI,
            feedback->distance_to_goal, feedback->percent_complete);
}

void CActionClientNode::resultCallback(const GoalHandleNavigateToGoal::WrappedResult &result)
{
    this->m_goalDone = true;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was succeed");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Final Pose:(%lf, %lf, %lf), Execution Time: %ds",
        result.result.get()->final_pose.x, result.result.get()->final_pose.y, result.result.get()->final_pose.theta * 180 / M_PI,
        result.result.get()->execution_time);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto actionClient = std::make_shared<CActionClientNode>();

    while (!actionClient->isGoalDone())
    {
        rclcpp::spin_some(actionClient);
    }

    rclcpp::shutdown();
    return 0;
}