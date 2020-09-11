uavcan_ros_bridge
=================
![CI](https://github.com/smarc-project/uavcan_ros_bridge/workflows/CI/badge.svg?branch=noetic-devel) [![license](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

Bridge for communication between ROS and the uavcan CAN-bus protocol

## Dependencies & building

After cloning, execute the following command in the cloned repo:
```
git submodule update --init --recursive
```
Once done, go into the folder `libuavcan/libuavcan/dsdl_compiler/pyuavcan/` and execute:
```
sudo python setup.py install
```
Then, simply put this project into a catkin workspace and run `catkin_make` or `catkin build` to build everything.

## Usage

Launch conversion in both directions (between uavcan and ros) by running the launch file:
```
roslaunch uavcan_ros_bridge bridge.launch
```
This will launch two nodes: the `uavcan_to_ros_bridge_node` and the `ros_to_uavcan_bridge_node`
which handle the conversion in the respective directions. Any new type conversions should
be added to the node that handles the relevant direction.

## Adding new conversions

You can add new conversion headers for [ros to uavcan](https://gitr.sys.kth.se/smarc-project/sam_drivers/tree/master/uavcan_ros_bridge/include/uavcan_ros_bridge/ros_to_uav)
and [uavcan to ros](https://gitr.sys.kth.se/smarc-project/sam_drivers/tree/master/uavcan_ros_bridge/include/uavcan_ros_bridge/uav_to_ros)
with corresponding implementations in the [src folder](https://gitr.sys.kth.se/smarc-project/sam_drivers/tree/master/uavcan_ros_bridge/src). An example implementation for an IMU might look like this:
```cpp
template <>
bool convert(const uavcan::equipment::ahrs::RawIMU& uav_msg, sensor_msgs::Imu& ros_msg)
{
    ros_msg.header.stamp = convert_timestamp(uav_msg.timestamp);
    ros_msg.linear_acceleration.x = uav_msg.accelerometer_latest[0];
    ros_msg.linear_acceleration.y = uav_msg.accelerometer_latest[1];
    ros_msg.linear_acceleration.z = uav_msg.accelerometer_latest[2];
    ros_msg.angular_velocity.x = uav_msg.rate_gyro_latest[0];
    ros_msg.angular_velocity.y = uav_msg.rate_gyro_latest[1];
    ros_msg.angular_velocity.z = uav_msg.rate_gyro_latest[2];
    return true;
}
```
You then need to add them to the build configuration in the cmake file [like this](https://gitr.sys.kth.se/smarc-project/sam_drivers/blob/master/uavcan_ros_bridge/CMakeLists.txt#L133)
and link them into one of the bridges (the publisher or the subscriber) [like this](https://gitr.sys.kth.se/smarc-project/sam_drivers/blob/master/uavcan_ros_bridge/CMakeLists.txt#L181).
Finally add them to either of the bridge nodes `uavcan_to_ros_bridge` or `ros_to_uavcan_bridge` similar to the
[already existing conversions](https://gitr.sys.kth.se/smarc-project/sam_drivers/blob/master/uavcan_ros_bridge/src/ros_to_uavcan_bridge.cpp#L46),
i.e. like:
```
ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, std_msgs::Float32> server(uav_node, ros_node, "uavcan_command");
```
or
```
uav_to_ros::ConversionServer<uavcan::equipment::ahrs::RawIMU, sensor_msgs::Imu> server(uav_node, ros_node, "uavcan_imu");
```
where `uavcan_command` and `uavcan_imu` are the ros topics being subscribed to and published to respectively.

## Existing conversions (*work in progress*)

### ROS to UAVCAN
* `std_msgs/Float32` to `uavcan.equipment.actuator.ArrayCommand` on ros topic `/uavcan_command`
* `sam_msgs/ArrayCommand` to `uavcan.equipment.actuator.ArrayCommand` on ros topic `/uavcan_array_command`
* `std_msgs/Int32` to `uavcan.equipment.esc.RPMCommand` on ros topic `/ros_to_uavcan_bridge_node/rpm_command`


### UAVCAN to ROS
* `uavcan.equipment.ahrs.RawIMU` to `sensor_msgs/Imu` on ros topic `/uavcan_imu`
* `uavcan.equipment.gnss.Fix` to `sensor_msgs/NavSatFix` on ros topic `/uavcan_to_ros_bridge_node/gps_fix`
* `uavcan.equipment.power.BatteryInfo` to `sensor_msgs/BatteryState` on ros topic `/uavcan_to_ros_bridge_node/battery_state`
* `uavcan.equipment.ahrs.MagneticFieldStrength` to `sensor_msgs/MagneticField` on ros topic `/uavcan_to_ros_bridge_node/magnetic_field`
