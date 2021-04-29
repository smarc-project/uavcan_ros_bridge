#ifndef UAV_TO_ROS_GPS_FIX_H
#define UAV_TO_ROS_GPS_FIX_H

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/NavSatFix.h>
#include <uavcan/equipment/gnss/Fix.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::gnss::Fix& uav_msg, sensor_msgs::NavSatFix& ros_msg);

}

#endif // UAV_TO_ROS_GPS_FIX_H
