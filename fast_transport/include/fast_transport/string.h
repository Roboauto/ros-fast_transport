#pragma once

#include <std_msgs/String.h>

template<> template<>
std_msgs::String fast_transport<std_msgs::String>::Publisher::GetMsg(const std::string &data, const std_msgs::Header &header) {
    std_msgs::String msg;
    msg.data =data;
    return msg;
}

template<> template<>
fast_transport_msgs::FastMsg fast_transport<std_msgs::String>::Publisher::GetFastMsg(const std::string &data, const std_msgs::Header &header) {
    fast_transport_msgs::FastMsg msg;
    msg.header = header;
    return msg;
}

