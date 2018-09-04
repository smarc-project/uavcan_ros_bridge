#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/ros_to_uav/command.h>
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/power/CircuitStatus.hpp>

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

/*
static void registerTypes()
{
    uavcan::DefaultDataTypeRegistrator<uavcan::equipment::actuator::Command> _reg1;
}
*/

int main(int argc, char** argv)
{
    //registerTypes();
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <node-id>" << std::endl;
        return 1;
    }

    const int self_node_id = std::stoi(argv[1]);

    auto& node = getNode();
    node.setNodeID(self_node_id);
    node.setName("org.uavcan.tutorial.publisher");

    ros::init(argc, argv, "ros_to_uavcan_bridge_node");
    ros::NodeHandle ros_node;

    /*
     * Dependent objects (e.g. publishers, subscribers, servers, callers, timers, ...) can be initialized only
     * if the node is running. Note that all dependent objects always keep a reference to the node object.
     */
    const int node_start_res = node.start();
    if (node_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    //RosUavConversionServer<uavcan::equipment::ahrs::RawIMU, std_msgs::Float32> server(node, ros_node, "uavcan_command");
    RosUavConversionServer<uavcan::equipment::actuator::ArrayCommand, std_msgs::Float32> server(node, ros_node, "uavcan_command");
    //RosUavConversionServer<uavcan::equipment::power::CircuitStatus, std_msgs::Float32> server(node, ros_node, "uavcan_command");

    /*
     * Running the node.
     */
    node.setModeOperational();

/*
    const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
    if (spin_res < 0) {
        std::cerr << "Transient failure: " << spin_res << std::endl;
    }

    while (ros::ok()) {
        const int res = node.spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0) {
            ROS_ERROR("Transient failure or shutdown: %d", res);
        }
    }*/
    ros::spin();

    return 0;
}
