#ifndef UAVCAN_TRANSPORT_STATS_H
#define UAVCAN_TRANSPORT_STATS_H

#include <uavcan/protocol/GetTransportStats.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/UavcanGetTransportStats.h>

namespace ros_to_uav {

template <>
bool convert_request(const uavcan_ros_msgs::UavcanGetTransportStats::Request& ros_request, uavcan::protocol::GetTransportStats::Request& uav_request);

template <>
bool convert_response(const uavcan::protocol::GetTransportStats::Response& uav_response, uavcan_ros_msgs::UavcanGetTransportStats::Response& ros_response);

}

#endif
