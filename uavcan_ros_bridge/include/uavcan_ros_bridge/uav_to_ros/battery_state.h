#ifndef UAV_TO_ROS_BATTERY_STATE_H
#define UAV_TO_ROS_BATTERY_STATE_H

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/BatteryState.h>
#include <uavcan/equipment/power/BatteryInfo.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::BatteryInfo& uav_msg, sensor_msgs::BatteryState& ros_msg);

}

#endif // UAV_TO_ROS_BATTERY_STATE_H
