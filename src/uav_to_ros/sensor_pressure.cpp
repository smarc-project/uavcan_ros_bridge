#include <uavcan_ros_bridge/uav_to_ros/sensor_pressure.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<smarc_uavcan_messages::SensorPressure>& uav_msg, sensor_msgs::FluidPressure& ros_msg, unsigned char uid)
{
    if (uid != 255 && uav_msg.device_id != uid) {
        return false;
    }

    ros_msg.header.stamp = ros::Time::now();
    ros_msg.fluid_pressure = uav_msg.pressure.static_pressure;
    ros_msg.variance = uav_msg.pressure.static_pressure_variance;
    return true;
}

}
