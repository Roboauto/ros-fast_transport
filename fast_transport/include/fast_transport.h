#pragma once

#include <string>

#include <ros/ros.h>

#include <fast_transport_msgs/FastMsg.h>

template<class T>
class fast_transport {
public:

    ros::Subscriber fast_subscriber;

    struct Publisher {
        ros::Publisher pub;
        ros::Publisher fast_pub;

        template<class D>
        T GetMsg(const D& data, const std_msgs::Header &header);

        template<class D>
        fast_transport_msgs::FastMsg GetFastMsg(const D& data, const std_msgs::Header &header);

        template<class D>
        void publish(const D& data, const std_msgs::Header &header);
    };

    static ros::Subscriber subscribe<T>() {
        return {};
    }

    static Publisher advertise(ros::NodeHandle&n, const std::string & topic) {
        Publisher pub;
        pub.pub = n.advertise<T>(topic, 1);
        pub.fast_pub = n.advertise<T>("/fast_transport/"+topic, 1);
        return pub;
    }


protected:
    //static std::string prefix = "/fast_transport/";
private:
};


template<class T>  template <class D>
void fast_transport<T>::Publisher::publish(const D &data, const std_msgs::Header &header) {
    if(pub.getNumSubscribers() != 0) {
        pub.publish(GetMsg(data,header));
    }
    if(fast_pub.getNumSubscribers() != 0) {
        fast_transport_msgs::FastMsg msg;
        msg.header = header;
        msg.data = "TEST";
        fast_pub.publish(GetFastMsg(data,header));
    }
}