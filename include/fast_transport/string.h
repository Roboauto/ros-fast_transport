#pragma once

#include <std_msgs/String.h>

template<> template<>
std_msgs::String fast_transport<std_msgs::String>::Publisher::GetMsg(const std::string &data, const std_msgs::Header &header) {
    std_msgs::String msg;
    msg.data =data;
    return msg;
}
