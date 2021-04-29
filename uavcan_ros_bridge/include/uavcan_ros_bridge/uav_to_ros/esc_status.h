#ifndef UAV_TO_ROS_ESC_STATUS_H
#define UAV_TO_ROS_ESC_STATUS_H

#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/ESCStatus.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>& uav_msg, uavcan_ros_msgs::ESCStatus& ros_msg, unsigned char uid);

}

#endif
