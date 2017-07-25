#pragma once

#include "../fast_transport.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

template<>
struct fast_transport::helper<sensor_msgs::Image, cv::Mat> {
    static sensor_msgs::Image
    get_msg(const cv::Mat &data, const std_msgs::Header &header, const std::string encoding = "") {
        auto bridge = cv_bridge::CvImage(header, encoding, data);
        sensor_msgs::Image msg;
        bridge.toImageMsg(msg);
        return msg;
    }

    static cv::Mat get_data(const sensor_msgs::Image &msg) {
        return cv_bridge::toCvCopy(msg)->image;
    }

    static std::size_t get_shared_size(const cv::Mat &data) {
        return sizeof(cv::Mat) + data.rows * data.cols * data.elemSize();
    }

    static void set_shared_data(const cv::Mat &data, boost::interprocess::mapped_region &region) {
	std::memcpy(region.get_address(),&data,sizeof(cv::Mat));
        cv::Mat *frame = static_cast<cv::Mat *>(region.get_address());
        frame->data = static_cast<uchar *>(region.get_address() + sizeof(cv::Mat));
        std::memcpy(frame->data, data.data, data.rows * data.cols * data.elemSize());
    }

    static cv::Mat get_shared_data(const boost::interprocess::mapped_region &region) {
        cv::Mat data = cv::Mat(static_cast<cv::Mat *>(region.get_address())->rows,
                               static_cast<cv::Mat *>(region.get_address())->cols,
                               static_cast<cv::Mat *>(region.get_address())->type(),
                               static_cast<uchar *>(region.get_address() + sizeof(cv::Mat)));
        return data;
    }
};
