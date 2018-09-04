#ifndef ROS_TO_UAV_COMMAND_H
#define ROS_TO_UAV_COMMAND_H

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <std_msgs/Float32.h>

namespace ros_to_uav {

template <>
bool convert(const std_msgs::Float32& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg);

}

#endif
