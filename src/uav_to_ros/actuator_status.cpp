#include <uavcan_ros_bridge/uav_to_ros/actuator_status.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>& uav_msg, sam_msgs::PercentStamped& ros_msg, unsigned char uid)
{
    if (uav_msg.actuator_id != uid) {
        return false;
    }

    ros_msg.header.stamp = ros::Time::now();
    ros_msg.value = uav_msg.position;

    return true;
}

}
