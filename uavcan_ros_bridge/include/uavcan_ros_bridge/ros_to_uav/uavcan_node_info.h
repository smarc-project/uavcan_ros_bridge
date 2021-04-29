#ifndef UAVCAN_NODE_INFO_H
#define UAVCAN_NODE_INFO_H

#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/UavcanGetNodeInfo.h>

namespace ros_to_uav {

template <>
bool convert_request(const uavcan_ros_msgs::UavcanGetNodeInfo::Request& ros_request, uavcan::protocol::GetNodeInfo::Request& uav_request);

template <>
bool convert_response(const uavcan::protocol::GetNodeInfo::Response& uav_response, uavcan_ros_msgs::UavcanGetNodeInfo::Response& ros_response);

}

#endif
