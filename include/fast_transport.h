#pragma once

#include <std_msgs/Int64.h>
#include <std_msgs/Header.h>

#include <ros/ros.h>

#include <string>

class fast_transport {
public:

    static class unique_id {
        public:
        int GetId() {
            //TODO: implement
            return 1;
        }
    } unique_id;

    struct Subscriber {
        Subscriber(ros::NodeHandle&n, const std::string & topic, std::size_t queue_size) {
            sub_info = n.subscribe(topic,1, &Subscriber::OnInfo, this);
        }

        ros::Subscriber sub;
        ros::Subscriber sub_info;
        ros::Subscriber sub_fast;

        void OnInfo(const std_msgs::Int64&x) {
            ROS_DEBUG_STREAM("ID OF PUBLISHER IS:" << x);
        }
    };

    template<class T>
    struct Publisher {
        ros::Publisher pub;
        ros::Publisher pub_info;
        ros::Publisher fast_pub;

        template<class D>
        T GetMsg(const D& data, const std_msgs::Header &header);

        template<class D>
        void publish(const D& data, const std_msgs::Header &header);
    };

    static std::shared_ptr<Subscriber> subscribe(ros::NodeHandle&n, const std::string & topic, std::size_t queue_size) {
        return std::make_shared<Subscriber>(n,topic,queue_size);
    }

    template<class T>
    static Publisher<T> advertise(ros::NodeHandle&n, const std::string & topic) {
        Publisher<T> pub;
        pub.pub = n.advertise<T>(topic, 1);
        pub.fast_pub = n.advertise<std_msgs::Header>("/fast_transport/"+topic, 1);
        pub.pub_info = n.advertise<std_msgs::Int64>("/fast_transport/info/"+topic, 1,true);
        std_msgs::Int64 msg;
        msg.data = unique_id.GetId();
        pub.pub_info.publish(msg);
        return pub;
    }


protected:
    //static std::string prefix = "/fast_transport/";
private:
};


template<class T>  template <class D>
void fast_transport::Publisher<T>::publish(const D &data, const std_msgs::Header &header) {
    if(pub.getNumSubscribers() != 0) {
        pub.publish(GetMsg(data,header));
    }
    if(fast_pub.getNumSubscribers() != 0) {
        fast_pub.publish(header);
    }
}