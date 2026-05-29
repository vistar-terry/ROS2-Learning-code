#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "hello_world_service_cpp/server.h"

AddTwoIntsServer::AddTwoIntsServer() : Node("add_two_ints_server")
{
    // 创建服务
    service_ = this->create_service<AddTwoInts>(
        "/add_two_ints",
        std::bind(&AddTwoIntsServer::handle_add_two_ints, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 开启自省机制
    service_->configure_introspection(
        this->get_clock(), rclcpp::SystemDefaultsQoS(), RCL_SERVICE_INTROSPECTION_CONTENTS);

    RCLCPP_INFO(this->get_logger(), "AddTwoInts 服务端已启动...");
}

void AddTwoIntsServer::handle_add_two_ints(
    const std::shared_ptr<AddTwoInts::Request> request,
    std::shared_ptr<AddTwoInts::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到请求: %ld + %ld", request->a, request->b);
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "返回结果: %ld", response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(server);
    rclcpp::shutdown();

    return 0;
}