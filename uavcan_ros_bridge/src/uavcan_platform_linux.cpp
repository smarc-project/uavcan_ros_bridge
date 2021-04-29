#include <uavcan_linux/uavcan_linux.hpp>

uavcan::ISystemClock& getSystemClock()
{
    static uavcan_linux::SystemClock clock;
    return clock;
}

uavcan::ICanDriver& getCanDriver(const std::string& can_interface="can0")
{
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock&>(getSystemClock()));
    if (driver.getNumIfaces() == 0)     // Will be executed once
    {
        if (driver.addIface(can_interface) < 0)
        {
            throw std::runtime_error("Failed to add iface");
        }
    }
    return driver;
}
