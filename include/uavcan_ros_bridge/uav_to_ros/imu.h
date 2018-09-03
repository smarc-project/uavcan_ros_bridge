#ifndef UAV_TO_ROS_IMU_H
#define UAV_TO_ROS_IMU_H

#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/Imu.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::ahrs::RawIMU& uav_msg, sensor_msgs::Imu& ros_msg);

}

#endif
