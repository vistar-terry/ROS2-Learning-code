#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "hello_world_service_cpp/async_client.h"

using namespace std::chrono_literals;

AddTwoIntsAsyncClient::AddTwoIntsAsyncClient() : Node("async_add_two_ints_client")
{
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints");
}

AddTwoIntsAsyncClient::~AddTwoIntsAsyncClient()
{
}

bool AddTwoIntsAsyncClient::add_two_ints(int a, int b)
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

    auto result = client_->async_send_request(request,
            std::bind(&AddTwoIntsAsyncClient::response_callback, this, std::placeholders::_1));

    return true;
}

void AddTwoIntsAsyncClient::response_callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
{
    try
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "异步回调收到响应: %ld", response->sum);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "服务调用失败: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsAsyncClient>();

    if (argc >= 3)
    {
        int a = std::stoi(argv[1]);
        int b = std::stoi(argv[2]);
        node->add_two_ints(a, b);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}