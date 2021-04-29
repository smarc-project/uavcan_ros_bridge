#include <uavcan_ros_bridge/ros_to_uav/uavcan_restart.h>

namespace ros_to_uav {

#pragma GCC diagnostic ignored "-Wunused-parameter"
template <>
bool convert_request(const uavcan_ros_msgs::UavcanRestartNode::Request& ros_request, uavcan::protocol::RestartNode::Request& uav_request)
{
    uav_request.magic_number = uav_request.MAGIC_NUMBER;
    return true;
}

template <>
bool convert_response(const uavcan::protocol::RestartNode::Response& uav_response, uavcan_ros_msgs::UavcanRestartNode::Response& ros_response)
{
    return true;
}
#pragma GCC diagnostic pop

}
