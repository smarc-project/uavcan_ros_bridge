#ifndef UAVCAN_ROS_BRIDGE_H
#define UAVCAN_ROS_BRIDGE_H

#include <uavcan/uavcan.hpp>
#include <ros/console.h>
#include <ros/ros.h>

namespace uav_to_ros {
 
template <typename UAVMSG, typename ROSMSG>
bool convert(const UAVMSG& uav_msg, ROSMSG& ros_msg)
{
    ROS_WARN("Can't find conversion for uavcan type %s", uav_msg.getDataTypeFullName());
    return false;
}

}

namespace ros_to_uav {

template <typename ROSMSG, typename UAVMSG>
bool convert(const ROSMSG& ros_msg, UAVMSG& uav_msg)
{
    ROS_WARN("Can't find conversion for uavcan type %s", uav_msg.getDataTypeFullName());
    return false;
}

}

template <typename UAVMSG, typename ROSMSG, unsigned NodeMemoryPoolSize=16384>
class UavRosConversionServer {
public:
    typedef uavcan::Node<NodeMemoryPoolSize> UavNode;
    typedef uavcan::MethodBinder<UavRosConversionServer*, void (UavRosConversionServer::*)(const UAVMSG&) const> UavMsgCallbackBinder;

    uavcan::Subscriber<UAVMSG, UavMsgCallbackBinder> uav_sub;
    ros::Publisher ros_pub;

    UavRosConversionServer(UavNode& uav_node, ros::NodeHandle& ros_node, const std::string& ros_topic) : uav_sub(uav_node)
    {
        ros_pub = ros_node.advertise<ROSMSG>(ros_topic, 10);

        const int uav_sub_start_res = uav_sub.start(UavMsgCallbackBinder(this, &UavRosConversionServer::conversion_callback));
        if (uav_sub_start_res < 0) {
            ROS_ERROR("Failed to start the uav subscriber, error: %d", uav_sub_start_res);
        }
    }

    void conversion_callback(const UAVMSG& uav_msg) const
    {
        ROSMSG ros_msg;
        bool success = uav_to_ros::convert(uav_msg, ros_msg);
        if (success) {
            ros_pub.publish(ros_msg);
        }
        else {
            ROS_WARN("There was an error trying to convert uavcan type %s", uav_msg.getDataTypeFullName());
        }
    }
};

template <typename UAVMSG, typename ROSMSG, unsigned NodeMemoryPoolSize=16384>
class RosUavConversionServer {
public:
    typedef uavcan::Node<NodeMemoryPoolSize> UavNode;

    uavcan::Publisher<UAVMSG> uav_pub;
    ros::Subscriber ros_sub;

    RosUavConversionServer(UavNode& uav_node, ros::NodeHandle& ros_node, const std::string& ros_topic) : uav_pub(uav_node)
    {
        const int uav_pub_init_res = uav_pub.init();
        if (uav_pub_init_res < 0) {
            ROS_ERROR("Failed to start the uav publisher, error: %d, type: %s", uav_pub_init_res, UAVMSG::getDataTypeFullName());
        }
        ros_sub = ros_node.subscribe(ros_topic, 10, &RosUavConversionServer::conversion_callback, this);
    }

    void conversion_callback(const ROSMSG& ros_msg)
    {
        UAVMSG uav_msg;
        bool success = ros_to_uav::convert(ros_msg, uav_msg);
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
#endif // UAVCAN_ROS_BRIDGE
