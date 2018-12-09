#include <uavcan_ros_bridge/ros_to_uav/uavcan_node_info.h>

namespace ros_to_uav {

template <>
bool convert_request(const uavcan_ros_bridge::UavcanGetNodeInfo::Request& ros_request, uavcan::protocol::GetNodeInfo::Request& uav_request)
{
    return true;
}

template <>
bool convert_response(const uavcan::protocol::GetNodeInfo::Response& uav_response, uavcan_ros_bridge::UavcanGetNodeInfo::Response& ros_response)
{
    ros_response.name.resize(uav_response.name.size()); //= std::string((char*)&uav_response.name[0], 4); //uav_response.name.size());
    for (int i = 0; i < uav_response.name.size(); ++i) {
        ros_response.name[i] = uav_response.name[i];
    }

	ros_response.status.uptime_sec = uav_response.status.uptime_sec;
	ros_response.status.mode = uav_response.status.mode;
	ros_response.status.sub_mode = uav_response.status.sub_mode;
	ros_response.status.vendor_specific_status_code = uav_response.status.vendor_specific_status_code;
    return true;
}

}
