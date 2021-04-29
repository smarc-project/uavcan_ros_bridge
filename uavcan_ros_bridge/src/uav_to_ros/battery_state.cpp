#include <uavcan_ros_bridge/uav_to_ros/battery_state.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::BatteryInfo& uav_msg, sensor_msgs::BatteryState& ros_msg)
{
    ros_msg.voltage = uav_msg.voltage;
    ros_msg.current = uav_msg.current;
    // ros_msg.charge is in Ah
    ros_msg.charge = uav_msg.remaining_capacity_wh/uav_msg.voltage;
    // ros_msg.capacity is in Ah
    ros_msg.capacity = uav_msg.full_charge_capacity_wh/uav_msg.voltage;

    // TODO: add support for all the diagnostics flags

    return true;
}

}
