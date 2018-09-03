#include <uavcan_ros_bridge/ros_to_uav/command.h>

namespace ros_to_uav {

template <>
bool convert(const std_msgs::Float32& ros_msg, uavcan::equipment::actuator::Command& uav_msg)
{
    ROS_INFO("Successfully converted the command to uavcan");
    uav_msg.command_value = ros_msg.data;
    return true;
}

}
