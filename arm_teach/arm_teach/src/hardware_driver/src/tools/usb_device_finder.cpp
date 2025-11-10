#include "tools/usb_device_finder.h"
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <sstream>

std::vector<UsbDeviceFinder::DeviceInfo> UsbDeviceFinder::getDevicesByVidPid(uint16_t vid, uint16_t pid)
{
    std::vector<DeviceInfo> devices;

    // 初始化 libusb
    libusb_context* context = nullptr;
    int result = libusb_init(&context);
    if (result < 0) {
        std::cerr << "[UsbDeviceFinder] Failed to initialize libusb: " << libusb_error_name(result) << std::endl;
        return devices;
    }

    // 获取设备列表
    libusb_device** usb_devices;
    ssize_t count = libusb_get_device_list(context, &usb_devices);
    if (count < 0) {
        std::cerr << "[UsbDeviceFinder] Failed to obtain device list: " << libusb_error_name(count) << std::endl;
        libusb_exit(context);
        return devices;
    }

    // 遍历所有设备
    for (int i = 0; usb_devices[i]; i++) {
        libusb_device* device = usb_devices[i];

        // 获取设备描述符
        libusb_device_descriptor desc;
        result = libusb_get_device_descriptor(device, &desc);
        if (result < 0) {
            continue;
        }

        // 检查VID/PID是否匹配
        if (desc.idVendor != vid || desc.idProduct != pid) {
            continue;
        }

        // 打开设备
        libusb_device_handle* handle = nullptr;
        result = libusb_open(device, &handle);
        if (result != LIBUSB_SUCCESS) {
            continue;
        }

        // 获取序列号
        char serial_number[256] = {0};
        if (desc.iSerialNumber > 0) {
            result = libusb_get_string_descriptor_ascii(
                handle,
                desc.iSerialNumber,
                reinterpret_cast<unsigned char*>(serial_number),
                sizeof(serial_number)
            );

            if (result < 0) {
                serial_number[0] = '\0';
            }
        }

        // 添加设备信息
        DeviceInfo info{};
        info.vid = desc.idVendor;
        info.pid = desc.idProduct;
        info.serial_number = serial_number;
        devices.push_back(info);

        // 关闭设备
        libusb_close(handle);
    }

    // 清理资源
    libusb_free_device_list(usb_devices, 1);
    libusb_exit(context);

    return devices;
}

std::string UsbDeviceFinder::getSerialNumberByPath(const std::string& device_path)
{
    // 注意：device_path 参数在这里主要用于调试/日志记录
    // 实际上我们需要列举所有DM-CANFD设备并返回第一个（或匹配的）

    // 获取所有DM-CANFD设备
    auto devices = listDmCanfdDevices();

    if (devices.empty()) {
        std::cerr << "[UsbDeviceFinder] No DM-CANFD devices found" << std::endl;
        return "";
    }

    // 返回第一个找到的设备序列号
    // 如果有多个设备，可以根据 device_path 进一步过滤（目前简化处理）
    return devices[0].serial_number;
}

std::string UsbDeviceFinder::getDmCanfdSerialNumber(const std::string& device_path)
{
    return getSerialNumberByPath(device_path);
}

std::vector<UsbDeviceFinder::DeviceInfo> UsbDeviceFinder::listDmCanfdDevices()
{
    return getDevicesByVidPid(DM_CANFD_VID, DM_CANFD_PID);
}
