uavcan_ros_bridge
=================

## Dependencies & building

This package requires [libuavcan](https://github.com/UAVCAN/libuavcan) to be installed on the 
computer. Follow the instructions on their website to install the library.

Then, simply put this project into a catkin workspace and run `catkin_make` to build everything.

## Usage

Launch conversion in both directions (between uavcan and ros) by running the launch file:

```
roslaunch uavcan_ros_bridge bridge.launch
```
## Adding new conversions

You can add new headers for [ros to uavcan](https://gitr.sys.kth.se/smarc-project/sam_drivers/tree/master/uavcan_ros_bridge/include/uavcan_ros_bridge/ros_to_uav)
and [uavcan to ros](https://gitr.sys.kth.se/smarc-project/sam_drivers/tree/master/uavcan_ros_bridge/include/uavcan_ros_bridge/uav_to_ros)
with corresponding implementations in the [src folder](https://gitr.sys.kth.se/smarc-project/sam_drivers/tree/master/uavcan_ros_bridge/src).
You then need to add them to the build configuration in the cmake file [like this](https://gitr.sys.kth.se/smarc-project/sam_drivers/blob/master/uavcan_ros_bridge/CMakeLists.txt#L133)
and link them into one of the bridges (the publisher or the subscriber) [like this](https://gitr.sys.kth.se/smarc-project/sam_drivers/blob/master/uavcan_ros_bridge/CMakeLists.txt#L165).
Finally add them to either of the bridge nodes `uavcan_to_ros_bridge` or `ros_to_uavcan_bridge` similar to the
[already existing conversions](https://gitr.sys.kth.se/smarc-project/sam_drivers/blob/master/uavcan_ros_bridge/src/ros_to_uavcan_bridge.cpp#L44),
i.e. like:
```
ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, std_msgs::Float32> server(node, ros_node, "uavcan_command");
```
or
```
uav_to_ros::ConversionServer<uavcan::equipment::ahrs::RawIMU, sensor_msgs::Imu> server(node, ros_node, "uavcan_imu");
```
where `uavcan_command` and `uavcan_imu` are the ros topics being subscribed to and published to respectively.

## Existing conversions (*work in progress*)

### ROS to UAVCAN
* `std_msgs/Float32` to `uavcan.equipment.actuator.ArrayCommand` on ros topic `/uavcan_command`

### UAVCAN to ROS
* `uavcan.equipment.ahrs.RawIMU` to `sensor_msgs/Imu` on ros topic `/uavcan_imu`
