#pragma once

#include <std_msgs/Int64.h>
#include <std_msgs/Header.h>

#include <ros/ros.h>

#include <string>

//TODO: remove!
#undef ROS_DEBUG_STREAM
#define ROS_DEBUG_STREAM(x) std::cerr << x << std::endl

class fast_transport {
public:

    template<class T, class D>
    struct helper;

    static class unique_id {
        public:
        int GetId() {
            //TODO: implement
            return 1;
        }
    } unique_id;

    template<class T, class D>
    struct Subscriber {
        template<class X>
        Subscriber(ros::NodeHandle&n, const std::string & topic_, std::size_t queue_size_,
                   const X& fun_):
                node(n),topic(topic_), queue_size(queue_size_), fun(fun_) {
            sub_info = n.subscribe("/fast_transport/info"+topic,1, &Subscriber<T,D>::OnInfo, this);
        }

        ros::Subscriber sub;
        ros::Subscriber sub_info;

        ros::NodeHandle &node;
        std::string topic;
        std::size_t queue_size;

        std::function<void(const D&, const std_msgs::Header&) > fun;

        void OnFast(const std_msgs::Header &header) {

        }

        void OnNormal(const T &header) {

        }

        void OnInfo(const std_msgs::Int64&x) {

            ROS_DEBUG_STREAM("ID OF PUBLISHER IS:" << x.data);

            if(x.data == unique_id.GetId()) {
                ROS_DEBUG_STREAM("SUBSCRIBING TO FAST TRANSPORT");
                sub.shutdown();
                sub = node.subscribe("/fast_transport"+topic,1, &Subscriber::OnFast, this);
            } else {
                ROS_DEBUG_STREAM("SUBSCRIBING TO SLOW TRANSPORT");
                sub.shutdown();
                sub = node.subscribe<T>(topic,1, [this](typename T::ConstPtr x) {
                    fun(helper<T,D>::get_data(*x.get()), x->header);
                });
            }
        }
    };

    template<class T>
    struct Publisher {
        ros::Publisher pub;
        ros::Publisher pub_info;
        ros::Publisher fast_pub;

        template<class D, typename ...X>
        void publish(const D& data, const std_msgs::Header &header, X...);
    };

    template<class T, class D, class X>
    static std::shared_ptr<Subscriber<T,D>> subscribe(ros::NodeHandle&n, const std::string & topic, std::size_t queue_size, const X& fun) {
        return std::make_shared<Subscriber<T,D>>(n,topic,queue_size,fun);
    }

    template<class T>
    static Publisher<T> advertise(ros::NodeHandle&n, const std::string & topic) {
        Publisher<T> pub;
        pub.pub = n.advertise<T>(topic, 1);
        pub.fast_pub = n.advertise<std_msgs::Header>("/fast_transport"+topic, 1);
        pub.pub_info = n.advertise<std_msgs::Int64>("/fast_transport/info"+topic, 1,true);
        std_msgs::Int64 msg;
        msg.data = unique_id.GetId();
        pub.pub_info.publish(msg);
        return pub;
    }


protected:
    //static std::string prefix = "/fast_transport/";
private:
};


template<class T>  template <class D, class ...X>
void fast_transport::Publisher<T>::publish(const D &data, const std_msgs::Header &header, X... args) {
    if(pub.getNumSubscribers() != 0) {
        pub.publish(helper<T,D>::get_msg(data,header, args...));
    }

    if(fast_pub.getNumSubscribers() != 0) {
        fast_pub.publish(header);
    }
}