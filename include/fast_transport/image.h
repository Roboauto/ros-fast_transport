#pragma once

#include "../fast_transport.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

template <>
struct fast_transport::helper<sensor_msgs::Image,cv::Mat> {
    static sensor_msgs::Image get_msg(const cv::Mat& data, const std_msgs::Header &header, const std::string encoding = "" ) {
        auto bridge = cv_bridge::CvImage(header, encoding, data);
        sensor_msgs::Image msg;
        bridge.toImageMsg(msg);
        return msg;
    }

    static cv::Mat get_data(const sensor_msgs::Image &msg) {
        return cv_bridge::toCvCopy(msg)->image;
    }

    static cv::Mat get_shared_data(const boost::interprocess::mapped_region &region) {
        cv::Mat* addr = static_cast<cv::Mat*>(region.get_address());
        cv::Mat data;
        addr->copyTo(data);
        return data;
    }
};