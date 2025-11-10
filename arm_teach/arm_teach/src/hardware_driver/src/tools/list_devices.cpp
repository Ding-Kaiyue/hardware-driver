#include "tools/usb_device_finder.h"
#include <iostream>

int main()
{
    std::cout << "========== DM-CANFD USB Device Finder ==========" << std::endl;
    std::cout << std::endl;

    // 列举所有DM-CANFD设备
    auto devices = UsbDeviceFinder::listDmCanfdDevices();

    if (devices.empty()) {
        std::cerr << "No DM-CANFD devices found!" << std::endl;
        return 1;
    }

    std::cout << "Found " << devices.size() << " DM-CANFD device(s):" << std::endl;
    std::cout << std::endl;

    for (size_t i = 0; i < devices.size(); i++) {
        const auto& device = devices[i];
        std::cout << "Device #" << i << ":" << std::endl;
        std::cout << "  VID:  0x" << std::hex << device.vid << std::endl;
        std::cout << "  PID:  0x" << std::hex << device.pid << std::endl;
        std::cout << "  SN:   " << std::dec << device.serial_number << std::endl;
        std::cout << std::endl;
    }

    std::cout << "You can use any of the above serial numbers to initialize Usb2CanfdManager" << std::endl;

    return 0;
}
