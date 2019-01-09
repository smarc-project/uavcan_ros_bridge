#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/uav_to_ros/imu.h>
#include <uavcan_ros_bridge/uav_to_ros/gps_fix.h>
#include <uavcan_ros_bridge/uav_to_ros/battery_state.h>
#include <uavcan_ros_bridge/uav_to_ros/magnetic_field.h>
#include <uavcan_ros_bridge/uav_to_ros/pressure.h>
#include <uavcan_ros_bridge/uav_to_ros/sensor_pressure.h>

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
    if (node_start_res < 0) {
        ROS_ERROR("Failed to start the node; error: %d", node_start_res);
        exit(0);
    }

    ros::NodeHandle pn("~");
    uav_to_ros::ConversionServer<uavcan::equipment::ahrs::Solution, sensor_msgs::Imu> imu_server(uav_node, pn, "imu");
    uav_to_ros::ConversionServer<uavcan::equipment::gnss::Fix, sensor_msgs::NavSatFix> gps_server(uav_node, pn, "gps_fix");
    uav_to_ros::ConversionServer<uavcan::equipment::power::BatteryInfo, sensor_msgs::BatteryState> battery_server(uav_node, pn, "battery_state");
    uav_to_ros::ConversionServer<uavcan::equipment::ahrs::MagneticFieldStrength, sensor_msgs::MagneticField> magnetic_server(uav_node, pn, "magnetic_field");
    // NOTE: the last argument is the source node id numbers, these are example values
    uav_to_ros::ConversionServer<uavcan::equipment::air_data::StaticPressure, sensor_msgs::FluidPressure> pressure_server1(uav_node, pn, "pressure1", 91);
    uav_to_ros::ConversionServer<uavcan::equipment::air_data::StaticPressure, sensor_msgs::FluidPressure> pressure_server2(uav_node, pn, "pressure2", 93);

    uav_to_ros::ConversionServer<uavcan_ros_bridge::SensorPressure, sensor_msgs::FluidPressure> sensor_pressure_server1(uav_node, pn, "sensor_pressure1", 1);
    uav_to_ros::ConversionServer<uavcan_ros_bridge::SensorPressure, sensor_msgs::FluidPressure> sensor_pressure_server2(uav_node, pn, "sensor_pressure2", 2);

    /*
     * Running the node.
     */
    uav_node.setModeOperational();
    signal(SIGINT, [] (int) { ros::shutdown(); });

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
