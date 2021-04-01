#ifndef ROS_TO_UAV_RPM_COMMAND_H
#define ROS_TO_UAV_RPM_COMMAND_H

#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <std_msgs/Int32.h>

namespace ros_to_uav {

// TODO: convert ros msg to sam_msgs/ThrusterRPMs
template <>
bool convert(const std_msgs::Int32& ros_msg, uavcan::equipment::esc::RPMCommand& uav_msg);

}

#endif // ROS_TO_UAV_RPM_COMMAND_H
