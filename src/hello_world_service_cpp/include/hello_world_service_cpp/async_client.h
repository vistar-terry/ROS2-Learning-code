#pragma once
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsAsyncClient : public rclcpp::Node
{
public:
    AddTwoIntsAsyncClient();
    ~AddTwoIntsAsyncClient();
    bool add_two_ints(int a, int b);
    void response_callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future);

private:
    rclcpp::Client<AddTwoInts>::SharedPtr client_;
};
