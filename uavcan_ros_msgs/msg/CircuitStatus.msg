#
# From uavcan CiruitStatus msg: Generic electrical circuit info.
#

uint8 ERROR_FLAG_OVERVOLTAGE  = 1
uint8 ERROR_FLAG_UNDERVOLTAGE = 2
uint8 ERROR_FLAG_OVERCURRENT  = 4
uint8 ERROR_FLAG_UNDERCURRENT = 8

uint8 error_flags

uint16 circuit_id

float32 voltage
float32 current
