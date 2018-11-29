#include <uavcan_ros_bridge/ros_to_uav/percent_stamped.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::PercentStamped& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg, unsigned char uid)
{

    uav_msg.commands.resize(1);
    uav_msg.commands[0].actuator_id = uid;
    uav_msg.commands[0].command_value = ros_msg.value;
    uav_msg.commands[0].command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_POSITION;

    return true;
}

}
