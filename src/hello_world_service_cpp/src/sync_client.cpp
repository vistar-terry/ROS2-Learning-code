#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "hello_world_service_cpp/sync_client.h"

AddTwoIntsSyncClient::AddTwoIntsSyncClient() : Node("sync_add_two_ints_client")
{
    client_ = this->create_client<AddTwoInts>("/add_two_ints");
    // 开启自省机制
    client_->configure_introspection(
        this->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);
}

AddTwoIntsSyncClient::~AddTwoIntsSyncClient()
{
}

bool AddTwoIntsSyncClient::add_two_ints(int a, int b, int& sum)
{
    // 等待服务端
    if (!client_->wait_for_service(5s))
    {
        RCLCPP_ERROR(this->get_logger(), "等待服务端超时");

        return false;
    }

    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    RCLCPP_INFO(this->get_logger(), "发送请求: %ld + %ld", request->a, request->b);

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        sum = response->sum;
        RCLCPP_INFO(this->get_logger(), "计算结果: %ld", response->sum);

        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "请求失败");
    }

    return false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsSyncClient>();

    if (argc >= 3)
    {
        int a = std::stoi(argv[1]);
        int b = std::stoi(argv[2]);
        RCLCPP_INFO(node->get_logger(), "请求参数: %d + %d", a, b);
        int sum;
        if (node->add_two_ints(a, b, sum))
        {
            RCLCPP_INFO(node->get_logger(), "返回结果: %d", sum);
        }
    }

    rclcpp::shutdown();

    return 0;
}