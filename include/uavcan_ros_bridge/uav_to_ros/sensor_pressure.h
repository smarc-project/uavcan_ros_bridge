#ifndef UAV_TO_ROS_SENSOR_PRESSURE_H
#define UAV_TO_ROS_SENSOR_PRESSURE_H

//#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan_ros_bridge/SensorPressure.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/FluidPressure.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan_ros_bridge::SensorPressure>& uav_msg, sensor_msgs::FluidPressure& ros_msg, unsigned char uid);

}

#endif
