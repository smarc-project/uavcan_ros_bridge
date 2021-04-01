#include <uavcan_ros_bridge/ros_to_uav/uavcan_transport_stats.h>

namespace ros_to_uav {

template <>
bool convert_request(const uavcan_ros_bridge::UavcanGetTransportStats::Request& ros_request, uavcan::protocol::GetTransportStats::Request& uav_request)
{
    return true;
}

template <>
bool convert_response(const uavcan::protocol::GetTransportStats::Response& uav_response, uavcan_ros_bridge::UavcanGetTransportStats::Response& ros_response)
{
    ros_response.transfers_tx = uav_response.transfers_tx;
    ros_response.transfers_rx = uav_response.transfers_rx;
    ros_response.transfer_errors = uav_response.transfer_errors;

    return true;
}

}
