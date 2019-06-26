#ifndef UAV_TO_ROS_ESC_STATUS_H
#define UAV_TO_ROS_ESC_STATUS_H

#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/ESCStatus.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::esc::Status& uav_msg, uavcan_ros_bridge::ESCStatus& ros_msg);

}

#endif
