#include <uavcan_ros_bridge/uav_to_ros/imu.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::ahrs::RawIMU& uav_msg, sensor_msgs::Imu& ros_msg)
{
    ros_msg.linear_acceleration.x = uav_msg.accelerometer_latest[0];
    ros_msg.linear_acceleration.y = uav_msg.accelerometer_latest[1];
    ros_msg.linear_acceleration.z = uav_msg.accelerometer_latest[2];
    ros_msg.angular_velocity.x = uav_msg.rate_gyro_latest[0];
    ros_msg.angular_velocity.y = uav_msg.rate_gyro_latest[1];
    ros_msg.angular_velocity.z = uav_msg.rate_gyro_latest[2];
    return true;
}

}
