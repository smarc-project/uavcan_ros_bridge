#ifndef ROS_TO_UAV_ARRAY_COMMAND_H
#define ROS_TO_UAV_ARRAY_COMMAND_H

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/ArrayCommand.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::ArrayCommand& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg);

}

#endif
