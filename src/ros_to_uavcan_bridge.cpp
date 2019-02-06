#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/ros_to_uav/command.h>
#include <uavcan_ros_bridge/ros_to_uav/rpm_command.h>
#include <uavcan_ros_bridge/ros_to_uav/array_command.h>
#include <uavcan_ros_bridge/ros_to_uav/percent_stamped.h>
#include <uavcan_ros_bridge/ros_to_uav/ballast_angles.h>

extern uavcan::ICanDriver& getCanDriver(const std::string&);
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node& getNode(const std::string& can_interface)
{
    static Node node(getCanDriver(can_interface), getSystemClock());
    return node;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_to_uavcan_bridge_node");
    ros::NodeHandle ros_node;

    int self_node_id;
    std::string can_interface;
    ros::param::param<int>("~uav_node_id", self_node_id, 113);
    ros::param::param<std::string>("~uav_can_interface", can_interface, "can0");
    
    auto& uav_node = getNode(can_interface);
    uav_node.setNodeID(self_node_id);
    uav_node.setName("smarc.sam.uavcan_bridge.publisher");

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
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, std_msgs::Float32> command_server(uav_node, pn, "command");
    ros_to_uav::ConversionServer<uavcan::equipment::esc::RPMCommand, std_msgs::Int32> rpm_server(uav_node, pn, "rpm_command");
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::ArrayCommand> array_server(uav_node, pn, "array_command");
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::PercentStamped> vbs_server(uav_node, pn, "vbs_command", 13);
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::PercentStamped> lcg_server(uav_node, pn, "lcg_command", 14);
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::BallastAngles> tcg_server1(uav_node, pn, "tcg_command1", 27);
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::BallastAngles> tcg_server2(uav_node, pn, "tcg_command2", 28);

    /*
     * Running the node.
     */
    uav_node.setModeOperational();

    std::function<void (const ros::TimerEvent&)> callback = [&] (const ros::TimerEvent&) {
        // Announce that the uav node is alive and well
        uav_node.spinOnce();
    };
    ros::Timer timer = ros_node.createTimer(ros::Duration(0.5), callback);

    ros::spin();

    return 0;
}
