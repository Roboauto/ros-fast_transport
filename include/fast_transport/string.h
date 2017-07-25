#pragma once

#include <std_msgs/String.h>

template <>
struct fast_transport::helper<std_msgs::String,std::string> {
    static std_msgs::String get_msg(const std::string &data, const std_msgs::Header &header) {
        std_msgs::String msg;
        msg.data =data;
        return msg;
    }
    static std::string get_data(const std_msgs::String&str) {
        return str.data;
    }
};