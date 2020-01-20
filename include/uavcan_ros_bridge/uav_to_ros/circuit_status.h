#ifndef UAV_TO_ROS_CIRCUIT_STATUS_H
#define UAV_TO_ROS_CIRCUIT_STATUS_H

#include <uavcan/equipment/power/CircuitStatus.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/CircuitStatus.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::CircuitStatus& uav_msg, uavcan_ros_bridge::CircuitStatus& ros_msg);

}

#endif
