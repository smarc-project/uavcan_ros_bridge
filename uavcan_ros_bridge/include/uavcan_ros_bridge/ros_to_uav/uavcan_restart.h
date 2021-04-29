#ifndef UAVCAN_RESTART_H
#define UAVCAN_RESTART_H

#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/UavcanRestartNode.h>

namespace ros_to_uav {

template <>
bool convert_request(const uavcan_ros_msgs::UavcanRestartNode::Request& ros_request, uavcan::protocol::RestartNode::Request& uav_request);

template <>
bool convert_response(const uavcan::protocol::RestartNode::Response& uav_response, uavcan_ros_msgs::UavcanRestartNode::Response& ros_response);

}

#endif
