#include <uavcan_ros_bridge/uav_to_ros/imu.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::ahrs::Solution& uav_msg, sensor_msgs::Imu& ros_msg)
{
    ros_msg.header.stamp = convert_timestamp(uav_msg.timestamp);
    ros_msg.orientation.x = uav_msg.orientation_xyzw[0];
    ros_msg.orientation.y = uav_msg.orientation_xyzw[1];
    ros_msg.orientation.z = uav_msg.orientation_xyzw[2];
    ros_msg.orientation.w = uav_msg.orientation_xyzw[3];
    ros_msg.linear_acceleration.x = uav_msg.linear_acceleration[0];
    ros_msg.linear_acceleration.y = uav_msg.linear_acceleration[1];
    ros_msg.linear_acceleration.z = uav_msg.linear_acceleration[2];
    ros_msg.angular_velocity.x = uav_msg.angular_velocity[0];
    ros_msg.angular_velocity.y = uav_msg.angular_velocity[1];
    ros_msg.angular_velocity.z = uav_msg.angular_velocity[2];
    return true;
}

}
