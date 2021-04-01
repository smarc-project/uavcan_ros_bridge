#include <uavcan_ros_bridge/uav_to_ros/circuit_status.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::CircuitStatus& uav_msg, uavcan_ros_msgs::CircuitStatus& ros_msg)
{
    ros_msg.error_flags = uav_msg.error_flags;
    ros_msg.circuit_id = uav_msg.circuit_id;
    ros_msg.voltage = uav_msg.voltage;
    ros_msg.current = uav_msg.current;

    return true;
}

}
