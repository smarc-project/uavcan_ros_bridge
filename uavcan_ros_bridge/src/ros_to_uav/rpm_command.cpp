#include <uavcan_ros_bridge/ros_to_uav/rpm_command.h>

namespace ros_to_uav {

template <>
bool convert(const std_msgs::Int32& ros_msg, uavcan::equipment::esc::RPMCommand& uav_msg)
{
    uav_msg.rpm.resize(1);
    uav_msg.rpm[0] = ros_msg.data;
    return true;
}

}
