#include <uavcan_ros_bridge/uav_to_ros/battery_state.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::BatteryInfo& uav_msg, sensor_msgs::BatteryState& ros_msg)
{
    if (ros_msg.voltage != -1) {    // -1 indicates voltage is unknown
        ros_msg.charge = uav_msg.remaining_capacity_wh / uav_msg.voltage;
        ros_msg.capacity = uav_msg.full_charge_capacity_wh / uav_msg.voltage;
    } else {
        ros_msg.charge = uav_msg.remaining_capacity_wh;     // Sent as Ah if voltage unknown
        ros_msg.capacity = uav_msg.full_charge_capacity_wh; // Sent as Ah if voltage unknown
    }

    ros_msg.header.stamp = ros::Time::now();
    ros_msg.voltage = uav_msg.voltage;
    ros_msg.current = uav_msg.current;
    ros_msg.design_capacity = NaN;
    ros_msg.percentage = uav_msg.state_of_charge_pct;

    ros_msg.power_supply_status = 0;
    ros_msg.power_supply_health = 0;
    ros_msg.power_supply_technology = 0;

    ros_msg.location = uav_msg.battery_id;
    // ros_msg.location = uav_msg.model_name + uav_msg.model_instance_id;

    // TODO: add support for all the diagnostics flags

    return true;
}

}
