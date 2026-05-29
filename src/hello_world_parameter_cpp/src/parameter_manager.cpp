#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class ParamManager : public rclcpp::Node
{
public:
    ParamManager() : Node("param_manager_node")
    {
        // 1. 声明参数
        this->declare_parameter("robot_speed", 1.0);
        this->declare_parameter("robot_name", std::string("robot_alpha"));
        this->declare_parameter("enable_debug", false);

        // 声明数组参数
        std::vector<double> default_waypoints = {1.0, 2.0, 3.0};
        this->declare_parameter("waypoints", default_waypoints);

        // 2. 添加参数回调
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParamManager::parametersCallback, this, std::placeholders::_1));

        // 3. 创建定时器，定期打印参数值
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ParamManager::printParameters, this));

        // 4. 初始打印参数
        printParameters();
    }

private:
    void printParameters()
    {
        // 获取并打印所有参数
        double robot_speed = this->get_parameter("robot_speed").as_double();
        std::string robot_name = this->get_parameter("robot_name").as_string();
        bool enable_debug = this->get_parameter("enable_debug").as_bool();
        auto waypoints = this->get_parameter("waypoints").as_double_array();

        RCLCPP_INFO(this->get_logger(), "=== Current Parameters ===");
        RCLCPP_INFO(this->get_logger(), "Robot Speed: %.2f", robot_speed);
        RCLCPP_INFO(this->get_logger(), "Robot Name: %s", robot_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Enable Debug: %s", enable_debug ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Waypoints: [%.1f, %.1f, %.1f]",
                    waypoints[0], waypoints[1], waypoints[2]);
        RCLCPP_INFO(this->get_logger(), "==========================");
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Parameter '%s' changed to: %s",
                        param.get_name().c_str(),
                        param.value_to_string().c_str());

            // 参数验证
            if (param.get_name() == "robot_speed")
            {
                double speed = param.as_double();
                if (speed < 0.0 || speed > 10.0)
                {
                    result.successful = false;
                    result.reason = "Robot speed must be between 0 and 10";
                    RCLCPP_WARN(this->get_logger(),
                                "Invalid speed value: %.2f", speed);
                    return result;
                }
            }

            if (param.get_name() == "robot_name")
            {
                std::string name = param.as_string();
                if (name.empty())
                {
                    result.successful = false;
                    result.reason = "Robot name cannot be empty";
                    return result;
                }
            }
        }

        return result;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamManager>());
    rclcpp::shutdown();

    return 0;
}