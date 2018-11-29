#include <uavcan_ros_bridge/ros_to_uav/ballast_angles.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::BallastAngles& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg, unsigned char uid)
{

    uav_msg.commands.resize(2);

    uav_msg.commands[0].actuator_id = uid;
    uav_msg.commands[0].command_value = ros_msg.weight_1_offset_radians;
    uav_msg.commands[0].command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_POSITION;

    uav_msg.commands[1].actuator_id = uid + 1;
    uav_msg.commands[1].command_value = ros_msg.weight_2_offset_radians;
    uav_msg.commands[1].command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_POSITION;

    return true;
}

}
