#ifndef UAV_TO_ROS_ACTUATOR_STATUS_H
#define UAV_TO_ROS_ACTUATOR_STATUS_H

#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/PercentStamped.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>& uav_msg, sam_msgs::PercentStamped& ros_msg, unsigned char uid);

}

#endif
