#include <uavcan_ros_bridge/ros_to_uav/array_command.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::ArrayCommand& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg)
{
    uav_msg.commands.resize(ros_msg.commands.size());
    for (int i = 0; i < ros_msg.commands.size(); ++i) {
        uav_msg.commands[i].actuator_id = ros_msg.commands[i].actuator_id;
        uav_msg.commands[i].command_value = ros_msg.commands[i].command_value;
        uav_msg.commands[i].command_type = ros_msg.commands[i].command_type;
    }
    return true;
}

}
