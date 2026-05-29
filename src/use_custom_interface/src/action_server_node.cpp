#include "use_custom_interface/action_server_node.h"

using namespace std::placeholders;

CActionServerNode::CActionServerNode()
    : Node("action_server_node")
    , m_goalPose(Pose2D())
    , m_currentPose(Pose2D())
{
    this->m_actionServer = rclcpp_action::create_server<NavigateToGoal>(
        this,
        "/useInterface/action/navigateToGoal",
        std::bind(&CActionServerNode::addAction, this, _1, _2),  // 添加action任务
        std::bind(&CActionServerNode::cancelAction, this, _1),   // 取消action任务
        std::bind(&CActionServerNode::executeAction, this, _1)); // 添加action成功后，执行action

    RCLCPP_INFO(this->get_logger(), "NavigateToGoal Action Server has been started.");
}

CActionServerNode::~CActionServerNode()
{
    RCLCPP_INFO(this->get_logger(), "NavigateToGoal Action Server is shutting down.");
}

rclcpp_action::GoalResponse CActionServerNode::addAction(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateToGoal::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal:(%.3lf, %.3lf, %.1lf°)",
                goal->target_pose.x, goal->target_pose.y, goal->target_pose.theta * 180.0/M_PI);

    m_goalPose.x = goal->target_pose.x;
    m_goalPose.y = goal->target_pose.y;
    m_goalPose.theta = goal->target_pose.theta;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CActionServerNode::cancelAction(
    const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    // 执行取消action操作 ...

    return rclcpp_action::CancelResponse::ACCEPT;
}

void CActionServerNode::executeAction(
    const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
    std::thread{std::bind(&CActionServerNode::execute, this, _1), goal_handle}.detach();
}

void CActionServerNode::execute(
    const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // 模拟运动
    LinearPathPlanner path(m_currentPose, m_goalPose);
    path.printPathInfo();
    double duration = path.getDuration();
    double time_step = 1.0; //1s检测一次
    int total_steps = static_cast<int>(std::ceil(duration / time_step));

    auto result = std::make_shared<NavigateToGoal::Result>();
    auto feedback = std::make_shared<NavigateToGoal::Feedback>();

    for (int step = 0; step <= total_steps; ++step)
    {
        double current_time = step * time_step;
        if (current_time > duration)
        {
            current_time = duration;
        }

        m_currentPose = path.getPoseAtTime(current_time);
        feedback->current_pose.x = m_currentPose.x;
        feedback->current_pose.y = m_currentPose.y;
        feedback->current_pose.theta = m_currentPose.theta;
        feedback->distance_to_goal = path.getRemainDistance();
        feedback->percent_complete = (current_time / duration) * 100;
        goal_handle->publish_feedback(feedback);

        m_currentPose.print();

        if (step < total_steps)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(time_step * 1000)));
        }
    }

    result->final_pose.x = m_currentPose.x;
    result->final_pose.y = m_currentPose.y;
    result->final_pose.theta = m_currentPose.theta;
    result->execution_time = duration;
    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Move was Completed!");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto actionServer = std::make_shared<CActionServerNode>();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Action Server started");

    rclcpp::spin(actionServer);
    rclcpp::shutdown();

    return 0;
}