#include <uavcan_ros_bridge/uav_to_ros/pressure.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::air_data::StaticPressure& uav_msg, sensor_msgs::FluidPressure& ros_msg)
{
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.fluid_pressure = uav_msg.static_pressure;
    ros_msg.variance = uav_msg.static_pressure_variance;
    return true;
}

}
