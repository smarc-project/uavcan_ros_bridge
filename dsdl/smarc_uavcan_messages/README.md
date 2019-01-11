# smarc_uavcan_messages
Custom UAVCAN messages for the SMaRC project.

## How to create a new message type

Give your message type a new UNUSED ID in the range 20000-20999.
Decide on a descriptive name that starts with a letter [a-zA-Z]

Create a new file with `<ID>.<your_custom_name>.uavcan`
Example: `20100.SensorPressure.uavcan`

Populate your message type.
See files in this repo and [UAVCAN types](https://uavcan.org/Specification/7._List_of_standard_data_types/) for examples.