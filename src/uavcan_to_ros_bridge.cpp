#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/uav_to_ros/imu.h>
#include <uavcan_ros_bridge/uav_to_ros/gps_fix.h>
#include <uavcan_ros_bridge/uav_to_ros/battery_state.h>
#include <uavcan_ros_bridge/uav_to_ros/magnetic_field.h>

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uavcan_to_ros_bridge_node");
    ros::NodeHandle ros_node;

    int self_node_id;
    ros::param::param<int>("~uav_node_id", self_node_id, 114);

    auto& uav_node = getNode();
    uav_node.setNodeID(self_node_id);
    uav_node.setName("smarc.sam.uavcan_bridge.subscriber");

    /*
     * Dependent objects (e.g. publishers, subscribers, servers, callers, timers, ...) can be initialized only
     * if the node is running. Note that all dependent objects always keep a reference to the node object.
     */
    const int node_start_res = uav_node.start();
    if (node_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    ros::NodeHandle pn("~");
    uav_to_ros::ConversionServer<uavcan::equipment::ahrs::RawIMU, sensor_msgs::Imu> imu_server(uav_node, pn, "imu");
    uav_to_ros::ConversionServer<uavcan::equipment::gnss::Fix, sensor_msgs::NavSatFix> gps_server(uav_node, pn, "gps_fix");
    uav_to_ros::ConversionServer<uavcan::equipment::power::BatteryInfo, sensor_msgs::BatteryState> battery_server(uav_node, pn, "battery_state");
    uav_to_ros::ConversionServer<uavcan::equipment::ahrs::MagneticFieldStrength, sensor_msgs::MagneticField> magnetic_server(uav_node, pn, "magnetic_field");

    /*
     * Running the node.
     */
    uav_node.setModeOperational();

    while (ros::ok()) {
        /*
         * The method spin() may return earlier if an error occurs (e.g. driver failure).
         * All error codes are listed in the header uavcan/error.hpp.
         */
        const int res = uav_node.spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0) {
            ROS_ERROR("Transient failure or shutdown: %d", res);
        }
    }

    return 0;
}
