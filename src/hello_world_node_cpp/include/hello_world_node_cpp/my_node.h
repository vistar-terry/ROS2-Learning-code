#ifndef __HELLO_WORLD_NODE_MYNODE_H__
#define __HELLO_WORLD_NODE_MYNODE_H__

#include "rclcpp/rclcpp.hpp"

class CMyNode : public rclcpp::Node
{

public:
    CMyNode();
    ~CMyNode();

private:
    /* data */
};


#endif // __HELLO_WORLD_NODE_MYNODE_H__