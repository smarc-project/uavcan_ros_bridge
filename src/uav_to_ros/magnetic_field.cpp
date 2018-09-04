#include <uavcan_ros_bridge/uav_to_ros/magnetic_field.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::ahrs::MagneticFieldStrength& uav_msg, sensor_msgs::MagneticField& ros_msg)
{
    ros_msg.magnetic_field.x = uav_msg.magnetic_field_ga[0];
    ros_msg.magnetic_field.y = uav_msg.magnetic_field_ga[1];
    ros_msg.magnetic_field.z = uav_msg.magnetic_field_ga[2];

    for (int i = 0; i < 9; ++i) {
        ros_msg.magnetic_field_covariance[i] = uav_msg.magnetic_field_covariance[i];
    }

    return true;
}

}
