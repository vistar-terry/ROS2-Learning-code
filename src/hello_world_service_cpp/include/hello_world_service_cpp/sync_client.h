#pragma once
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsSyncClient : public rclcpp::Node
{
public:
    AddTwoIntsSyncClient();
    ~AddTwoIntsSyncClient();
    bool add_two_ints(int a, int b, int& sum);

private:
    rclcpp::Client<AddTwoInts>::SharedPtr client_;
};
