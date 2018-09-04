#ifndef UAV_TO_ROS_MAGNETIC_FIELD_H
#define UAV_TO_ROS_MAGNETIC_FIELD_H

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/MagneticField.h>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::ahrs::MagneticFieldStrength& uav_msg, sensor_msgs::MagneticField& ros_msg);

}

#endif // UAV_TO_ROS_MAGNETIC_FIELD_H
