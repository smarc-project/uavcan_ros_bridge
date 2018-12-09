#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/ros_to_uav/uavcan_node_info.h>

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
    ros::init(argc, argv, "ros_to_uavcan_services_node");
    ros::NodeHandle ros_node;

    int self_node_id;
    std::string can_interface;
    ros::param::param<int>("~uav_node_id", self_node_id, 115);
    ros::param::param<std::string>("~uav_can_interface", can_interface, "can0");
    
    auto& uav_node = getNode(can_interface);
    uav_node.setNodeID(self_node_id);
    uav_node.setName("smarc.sam.uavcan_bridge.service_caller");

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
    ros_to_uav::ServiceConversionServer<uavcan::protocol::GetNodeInfo, uavcan_ros_bridge::UavcanGetNodeInfo> node_info_server(uav_node, pn, "get_node_info");

    /*
     * Running the node.
     */
    uav_node.setModeOperational();

    std::function<void (const ros::TimerEvent&)> callback = [&] (const ros::TimerEvent&) {
        // Announce that the uav node is alive and well
        uav_node.spinOnce();
    };
    ros::Timer timer = ros_node.createTimer(ros::Duration(0.5), callback);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    //ros::spin();
    ros::waitForShutdown();

    return 0;
}
