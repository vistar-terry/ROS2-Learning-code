#include <rclcpp/rclcpp.hpp>
#include <thread>

class DynamicParamModifier : public rclcpp::Node
{
public:
    DynamicParamModifier() : Node("dynamic_param_modifier"), counter_(0)
    {
        // 创建参数客户端
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "param_manager_node");

        // 等待参数服务器可用
        while (!param_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Parameter service not available, waiting...");
        }

        // 创建定时器，定期修改参数
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&DynamicParamModifier::modifyParameters, this));

    }

private:
    void modifyParameters()
    {
        counter_++;

        // 动态修改参数
        double new_speed = 1.0 + (counter_ % 5) * 0.5;
        std::string new_name = "robot_" + std::to_string(counter_);

        RCLCPP_INFO(this->get_logger(),
                    "Modifying parameters: speed=%.2f, name=%s",
                    new_speed, new_name.c_str());

        // 异步设置参数
        auto results_future = param_client_->set_parameters({rclcpp::Parameter("robot_speed", new_speed),
                                                             rclcpp::Parameter("robot_name", new_name)});

        // 在后台线程等待结果，避免在回调中阻塞或重复将节点添加到执行器
        auto node_shared = this->shared_from_this();
        std::thread([node_shared, results_future]() mutable
        {
            if (results_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
            {
                auto results = results_future.get();
                for (const auto &result : results)
                {
                    if (!result.successful)
                    {
                        RCLCPP_ERROR(node_shared->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(node_shared->get_logger(), "Setting parameters timed out");
            } 
        }).detach();
    }

    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicParamModifier>());
    rclcpp::shutdown();
    return 0;
}