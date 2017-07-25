#pragma once

#include <std_msgs/Int64.h>
#include <std_msgs/Header.h>

#include <ros/ros.h>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <string>

#include "fast_transport/queue.h"

//TODO: remove!
#undef ROS_DEBUG_STREAM
#define ROS_DEBUG_STREAM(x) std::cerr << x << std::endl

namespace bip = boost::interprocess;

class fast_transport {
public:

    template<class T, class D>
    struct helper;

    static class unique_id_c {
        public:
            unique_id_c() {
                try {
                    bip::shared_memory_object shm (bip::open_only, "fast_transport_id", bip::read_only);
                    bip::mapped_region region(shm, bip::read_only);
                    id = *static_cast<int*>(region.get_address());
                    ROS_DEBUG_STREAM("LOADED identification with ID:" << id);
                } catch (const boost::interprocess::interprocess_exception & except) {
                    srand(time(NULL));
                    const int newID = rand();
                    bip::shared_memory_object::remove("fast_transport_id");
                    bip::shared_memory_object shm (bip::create_only, "fast_transport_id", bip::read_write);

                    shm.truncate(sizeof(int));
                    bip::mapped_region region(shm, bip::read_write);
                    *static_cast<int*>(region.get_address()) = newID;
                    id = newID;
                    ROS_DEBUG_STREAM("CREATED identification with ID:" << newID);
                }
            }

            int GetId() {
                return id;
            }
        private:
            int id;
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

        void OnInfo(const std_msgs::Int64&x) {
            ROS_DEBUG_STREAM("ID OF PUBLISHER IS:" << x.data);

            if(x.data == unique_id.GetId()) {
                ROS_DEBUG_STREAM("SUBSCRIBING TO FAST TRANSPORT");
                sub.shutdown();
                sub = node.subscribe<std_msgs::Header>("/fast_transport"+topic,1, [this](std_msgs::Header::ConstPtr x) {
                    try {
                        const std::string memoryName = topic + "/" + std::to_string(x->seq);
                        bip::shared_memory_object shm (bip::open_only, memoryName.c_str(), bip::read_only);
                        bip::mapped_region region(shm, bip::read_only);

                        fun(helper<T,D>::get_shared_data(region), *x);
                    } catch (const boost::interprocess::interprocess_exception & except) {
                        ROS_ERROR_STREAM( "Exception while opening shared memory on fast_transport: "<< except.what());
                    }
                });
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
        Publisher(const std::string& topic, std::size_t queueSize = 10) : queue(queueSize), topic(topic), remover(topic) {
        }

        ros::Publisher pub;
        ros::Publisher pub_info;
        ros::Publisher fast_pub;
        Queue queue;
        std::string topic;

        struct shm_remove {
            shm_remove(const std::string& topicName) : topic(topicName) {
                bip::shared_memory_object::remove(topic.c_str());

            }
            ~shm_remove(){ bip::shared_memory_object::remove(topic.c_str()); }
            std::string topic;
        } remover;

        template<class D, typename ...X>
        void publish(const D& data, const std_msgs::Header &header, X...);
    };

    template<class T, class D, class X>
    static std::shared_ptr<Subscriber<T,D>> subscribe(ros::NodeHandle&n, const std::string & topic, std::size_t queue_size, const X& fun) {
        return std::make_shared<Subscriber<T,D>>(n,topic,queue_size,fun);
    }

    template<class T>
    static Publisher<T> advertise(ros::NodeHandle&n, const std::string & topic) {
        Publisher<T> pub(topic);
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
    auto id{header.seq};
    bip::shared_memory_object shm(bip::create_only, std::string(topic + std::to_string(id)).c_str(), bip::read_write);
    shm.truncate(sizeof(D));

    bip::mapped_region region{shm, bip::read_write};
    auto shdata = static_cast<D*>(region.get_address());
    *shdata = data;
    if(pub.getNumSubscribers() != 0) {
        pub.publish(helper<T,D>::get_msg(data,header, args...));
    }

    if(fast_pub.getNumSubscribers() != 0) {
        fast_pub.publish(header);
    }
}