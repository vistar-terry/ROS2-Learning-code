#pragma once
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsServer : public rclcpp::Node
{
public:
    AddTwoIntsServer();

private:
    void handle_add_two_ints(
        const std::shared_ptr<AddTwoInts::Request> request,
        std::shared_ptr<AddTwoInts::Response> response);

    rclcpp::Service<AddTwoInts>::SharedPtr service_;
};
