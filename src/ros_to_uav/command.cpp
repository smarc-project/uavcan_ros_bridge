#include <uavcan_ros_bridge/ros_to_uav/command.h>

namespace ros_to_uav {

template <>
bool convert(const std_msgs::Float32& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg)
{
    uav_msg.commands.resize(1);
    uav_msg.commands[0].command_value = ros_msg.data;
    return true;
}

}
