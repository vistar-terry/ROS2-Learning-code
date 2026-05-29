#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class BusinessNode : public rclcpp::Node
{
public:
    BusinessNode() : Node("business_node")
    {
        // 启动一个同步客户端，连接服务端节点：param_server_node
        sync_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "param_manager_node");

        // 等待服务上线
        while (!sync_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for parameter service...");
        }

        // 获取远程参数
        auto robot_name_param = sync_client_->get_parameter<std::string>("robot_name");
        RCLCPP_INFO(this->get_logger(), "Got remote robot_name: %s", robot_name_param.c_str());

        // 设置远程参数
        sync_client_->set_parameters({rclcpp::Parameter("robot_name", "new_robot_from_client")});
        RCLCPP_INFO(this->get_logger(), "Set remote parameter robot_name to 'new_robot_from_client'");

        // 验证设置是否成功
        auto updated_name = sync_client_->get_parameter<std::string>("robot_name");
        RCLCPP_INFO(this->get_logger(), "Verified updated robot_name: %s", updated_name.c_str());

        // 订阅参数事件，监听参数变化
        auto event_cb = [this](const rcl_interfaces::msg::ParameterEvent & event)
        {
            // 过滤只关注 param_manager_node 的参数事件
            // if (event.node != "param_manager_node") 
            // {
            //     return;
            // }

            // 遍历所有发生变化的参数
            for (const auto &p : event.changed_parameters)
            {
                rclcpp::Parameter param;
                try
                {
                    param = rclcpp::Parameter::from_parameter_msg(p);
                    RCLCPP_INFO(this->get_logger(), "[%s] '%s' convert to: %s", 
                        event.node.c_str(), p.name.c_str(), param.value_to_string().c_str());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error converting parameter '%s': %s", p.name.c_str(), e.what());
                }
            }
        };
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        event_cb_handle_ = param_subscriber_->add_parameter_event_callback(event_cb);
    }

private:
    std::shared_ptr<rclcpp::SyncParametersClient> sync_client_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> event_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BusinessNode>());
    rclcpp::shutdown();

    return 0;
}