#ifndef UAVCAN_ROS_BRIDGE_H
#define UAVCAN_ROS_BRIDGE_H

#include <uavcan/uavcan.hpp>
#include <ros/console.h>
#include <ros/ros.h>

namespace uav_to_ros {

ros::Time convert_timestamp(const uavcan::Timestamp& uav_time)
{
    ros::Time ros_time;
    ros_time.fromNSec(1000*uint64_t(uav_time.usec));
    return ros_time;
}
 
template <typename UAVMSG, typename ROSMSG>
bool convert(const UAVMSG&, ROSMSG&)
{
    //ROS_WARN("Can't find conversion for uavcan type %s", uav_msg.getDataTypeFullName());
    static_assert(sizeof(UAVMSG) == -1 || sizeof(ROSMSG) == -1, "ERROR: You need to supply a convert specialization for the UAVCAN -> ROS msg types provided");
    return false;
}

template <typename UAVMSG, typename ROSMSG>
bool convert(const uavcan::ReceivedDataStructure<UAVMSG>& uav_msg, ROSMSG& ros_msg, unsigned char uid)
{
    uavcan::NodeID node_id = uav_msg.getSrcNodeID();
    if (uid == 255 || uid == node_id.get()) {
        return convert((const UAVMSG&)uav_msg, ros_msg);
    }
    return false;
}

template <typename UAVMSG, typename ROSMSG, unsigned NodeMemoryPoolSize=16384>
class ConversionServer {
public:
    typedef uavcan::ReceivedDataStructure<UAVMSG> ReceivedDataStructure;
    typedef uavcan::Node<NodeMemoryPoolSize> UavNode;
    typedef uavcan::MethodBinder<ConversionServer*, void (ConversionServer::*)(const UAVMSG&) const> UavMsgCallbackBinder;
    typedef uavcan::MethodBinder<ConversionServer*, void (ConversionServer::*)(const ReceivedDataStructure&) const> UavMsgExtendedBinder;

    //uavcan::Subscriber<UAVMSG, UavMsgCallbackBinder> uav_sub;
    uavcan::Subscriber<UAVMSG, UavMsgExtendedBinder> uav_sub;
    ros::Publisher ros_pub;
    unsigned char uid;

    ConversionServer(UavNode& uav_node, ros::NodeHandle& ros_node, const std::string& ros_topic, unsigned char uid=255) : uav_sub(uav_node), uid(uid)
    {
        ros_pub = ros_node.advertise<ROSMSG>(ros_topic, 10);

        //const int uav_sub_start_res = uav_sub.start(UavMsgCallbackBinder(this, &ConversionServer::conversion_callback));
        const int uav_sub_start_res = uav_sub.start(UavMsgExtendedBinder(this, &ConversionServer::conversion_callback));
        if (uav_sub_start_res < 0) {
            ROS_ERROR("Failed to start the uav subscriber, error: %d", uav_sub_start_res);
        }
    }

    void conversion_callback(const ReceivedDataStructure& uav_msg) const
    {
        ROSMSG ros_msg;
        bool success = convert(uav_msg, ros_msg, uid);
        if (success) {
            ros_pub.publish(ros_msg);
        }
        /*
        else {
            ROS_WARN("There was an error trying to convert uavcan type %s", uav_msg.getDataTypeFullName());
        }
        */
    }
};

}

namespace ros_to_uav {

uavcan::Timestamp convert_timestamp(const ros::Time& ros_time)
{
    uavcan::Timestamp uav_time;
    uint64_t nsec = ros_time.toNSec();
    uav_time.usec = nsec / 1000;
    return uav_time;
}

template <typename ROSMSG, typename UAVMSG>
bool convert(const ROSMSG&, UAVMSG&)
{
    //ROS_WARN("Can't find conversion for uavcan type %s", uav_msg.getDataTypeFullName());
    static_assert(sizeof(UAVMSG) == -1 || sizeof(ROSMSG) == -1, "ERROR: You need to supply a convert specialization for the ROS -> UAVCAN msg types provided");
    return false;
}

template <typename ROSMSG, typename UAVMSG>
bool convert(const ROSMSG& ros_msg, UAVMSG& uav_msg, unsigned char)
{
    return convert(ros_msg, uav_msg);
}

template <typename UAVMSG, typename ROSMSG, unsigned NodeMemoryPoolSize=16384>
class ConversionServer {
public:
    typedef uavcan::Node<NodeMemoryPoolSize> UavNode;

    uavcan::Publisher<UAVMSG> uav_pub;
    ros::Subscriber ros_sub;
    unsigned char uid;

    ConversionServer(UavNode& uav_node, ros::NodeHandle& ros_node, const std::string& ros_topic, unsigned char uid=0) : uav_pub(uav_node), uid(uid)
    {
        const int uav_pub_init_res = uav_pub.init();
        if (uav_pub_init_res < 0) {
            ROS_ERROR("Failed to start the uav publisher, error: %d, type: %s", uav_pub_init_res, UAVMSG::getDataTypeFullName());
        }
        ros_sub = ros_node.subscribe(ros_topic, 10, &ConversionServer::conversion_callback, this);
    }

    void conversion_callback(const ROSMSG& ros_msg)
    {
        UAVMSG uav_msg;
        bool success = convert(ros_msg, uav_msg, uid);
        if (success) {
            const int pub_res = uav_pub.broadcast(uav_msg);
            if (pub_res < 0) {
                ROS_WARN("There was an error broadcasting the uavcan type %s", uav_msg.getDataTypeFullName());
            }
        }
        else {
            ROS_WARN("There was an error trying to convert uavcan type %s", uav_msg.getDataTypeFullName());
        }
    }
};

}

#endif // UAVCAN_ROS_BRIDGE
