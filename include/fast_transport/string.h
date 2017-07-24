#pragma once

#include <std_msgs/String.h>

template<> template<>
std_msgs::String fast_transport::Publisher<std_msgs::String>::GetMsg(const std::string &data, const std_msgs::Header &header) {
    std_msgs::String msg;
    msg.data =data;
    return msg;
}
