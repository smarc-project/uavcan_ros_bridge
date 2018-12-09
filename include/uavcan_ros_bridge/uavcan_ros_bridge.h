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

template <typename ROSREQ, typename UAVREQ>
bool convert_request(const ROSREQ&, UAVREQ&)
{
    static_assert(sizeof(UAVREQ) == -1 || sizeof(ROSREQ) == -1, "ERROR: You need to supply a convert specialization for the ROS -> UAVCAN service request");
    return false;
}

template <typename UAVRES, typename ROSRES>
bool convert_response(const UAVRES&, ROSRES&)
{
    static_assert(sizeof(UAVRES) == -1 || sizeof(ROSRES) == -1, "ERROR: You need to supply a convert specialization for the UAVCAN -> ROS service response");
    return false;
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

template <typename UAVSRV, typename ROSSRV, unsigned NodeMemoryPoolSize=16384>
class ServiceConversionServer {
public:
    typedef uavcan::Node<NodeMemoryPoolSize> UavNode;

    ros::ServiceServer ros_service;
	UavNode& uav_node;
	uavcan::ServiceClient<UAVSRV> uav_client;

    ServiceConversionServer(UavNode& uav_node, ros::NodeHandle& ros_node, const std::string& ros_service_name) : uav_node(uav_node), uav_client(uav_node) //, node_id(node_id)
    {
		const int uav_client_init_res = uav_client.init();
		if (uav_client_init_res < 0) {
            ROS_ERROR("Failed to start the uav publisher, error: %d, type: %s", uav_client_init_res, UAVSRV::getDataTypeFullName());
		}
		uav_client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(200));
		uav_client.setPriority(uavcan::TransferPriority::OneHigherThanLowest);

		ros_service = ros_node.advertiseService(ros_service_name, &ServiceConversionServer::service_callback, this);
    }

    bool service_callback(typename ROSSRV::Request& ros_req, typename ROSSRV::Response& ros_res)
    {
        typename UAVSRV::Request uav_req;
        bool success = convert_request(ros_req, uav_req);
        if (!success) {
            ROS_WARN("There was an error trying to convert uavcan service %s", UAVSRV::getDataTypeFullName());
            return false;
        }

		uav_client.setCallback([&](const uavcan::ServiceCallResult<UAVSRV>& uav_res) {
            if (uav_res.isSuccessful()) {
                success = convert_response((const typename UAVSRV::Response&)uav_res.getResponse(), ros_res);
            }
            else {
                ROS_WARN("There was an error call the uavcan service %s on node id %d", UAVSRV::getDataTypeFullName(), static_cast<int>(uav_res.getCallID().server_node_id.get()));
                success = false;
			}
        });

        const int uav_call_res = uav_client.call(ros_req.node_id, uav_req);
		if (uav_call_res < 0) {
            ROS_WARN("Unable to perform service call: %d", uav_call_res);
            return false;
		}

        uav_node.setModeOperational();
        while (uav_client.hasPendingCalls()) {
            const int res = uav_node.spin(uavcan::MonotonicDuration::fromMSec(10));
            if (res < 0) {
                ROS_WARN("Transient failure: %d", res);
                success = false;
                break;
            }
        }

        return success;
    }
};

}

#endif // UAVCAN_ROS_BRIDGE
