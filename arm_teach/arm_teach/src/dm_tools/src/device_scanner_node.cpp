#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <filesystem>
#include <fstream>

class DeviceScannerNode : public rclcpp::Node
{
public:
    DeviceScannerNode() : Node("device_scanner")
    {
        RCLCPP_INFO(get_logger(), "Device Scanner Node initialized");
        
        // 声明参数
        declare_parameter<std::string>("device_path", "");
        declare_parameter<bool>("scan_usb", true);
        declare_parameter<bool>("scan_serial", true);
        declare_parameter<bool>("publish_info", false);
        
        // 创建发布器
        device_info_pub_ = create_publisher<std_msgs::msg::String>(
            "device_info", rclcpp::QoS(10));
        
        // 获取参数
        auto device_path = get_parameter("device_path").as_string();
        auto scan_usb = get_parameter("scan_usb").as_bool();
        auto scan_serial = get_parameter("scan_serial").as_bool();
        auto publish_info = get_parameter("publish_info").as_bool();
        
        // 执行扫描
        scan_devices(device_path, scan_usb, scan_serial, publish_info);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr device_info_pub_;
    
    struct DeviceInfo {
        std::string type;
        std::string path;
        std::string serial_number;
        uint16_t vid = 0;
        uint16_t pid = 0;
        int device_index = -1;
    };
    
    void scan_devices(const std::string& device_path, bool scan_usb, bool scan_serial, bool publish_info)
    {
        std::vector<DeviceInfo> found_devices;
        
        // 如果指定了设备路径，优先处理
        if (!device_path.empty()) {
            RCLCPP_INFO(get_logger(), "Scanning specific device: %s", device_path.c_str());
            
            if (device_path.find("/dev/ttyACM") != std::string::npos) {
                auto device = scan_serial_device(device_path);
                if (!device.serial_number.empty()) {
                    found_devices.push_back(device);
                }
            } else {
                RCLCPP_WARN(get_logger(), "Unsupported device path format: %s", device_path.c_str());
            }
        } else {
            // 扫描所有设备
            if (scan_usb) {
                RCLCPP_INFO(get_logger(), "Scanning USB devices...");
                auto usb_devices = scan_usb_devices();
                found_devices.insert(found_devices.end(), usb_devices.begin(), usb_devices.end());
            }
            
            if (scan_serial) {
                RCLCPP_INFO(get_logger(), "Scanning serial devices...");
                auto serial_devices = scan_serial_devices();
                found_devices.insert(found_devices.end(), serial_devices.begin(), serial_devices.end());
            }
        }
        
        // 显示结果
        display_results(found_devices, publish_info);
    }
    
    std::vector<DeviceInfo> scan_usb_devices()
    {
        std::vector<DeviceInfo> devices;
        
        // 初始化 libusb
        libusb_context* context = nullptr;
        int result = libusb_init(&context);
        if (result < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize libusb: %s", libusb_error_name(result));
            return devices;
        }
        
        // 获取设备列表
        libusb_device** device_list;
        ssize_t count = libusb_get_device_list(context, &device_list);
        if (count < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to obtain device list: %s", libusb_error_name(count));
            libusb_exit(context);
            return devices;
        }
        
        int device_index = 0;
        for (int i = 0; device_list[i]; i++) {
            libusb_device* device = device_list[i];
            
            // 获取设备描述符
            libusb_device_descriptor desc;
            result = libusb_get_device_descriptor(device, &desc);
            if (result < 0) {
                continue;
            }
            
            // 只处理DM-CANFD的VID和PID的设备
            if (desc.idVendor != 0x34B7 || desc.idProduct != 0x6877) {
                continue;
            }
            
            // 打开设备获取序列号
            libusb_device_handle* handle = nullptr;
            result = libusb_open(device, &handle);
            if (result != LIBUSB_SUCCESS) {
                RCLCPP_WARN(get_logger(), "Failed to open USB device %d: %s", i, libusb_error_name(result));
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
                    RCLCPP_WARN(get_logger(), "Failed to obtain serial number for device %d: %s", 
                               i, libusb_error_name(result));
                    serial_number[0] = '\0';
                }
            }
            
            // 创建设备信息
            DeviceInfo info;
            info.type = "USB";
            info.path = "USB Device " + std::to_string(device_index);
            info.serial_number = serial_number[0] ? serial_number : "[No serial number]";
            info.vid = desc.idVendor;
            info.pid = desc.idProduct;
            info.device_index = device_index;
            
            devices.push_back(info);
            device_index++;
            
            libusb_close(handle);
        }
        
        libusb_free_device_list(device_list, 1);
        libusb_exit(context);
        
        return devices;
    }
    
    std::vector<DeviceInfo> scan_serial_devices()
    {
        std::vector<DeviceInfo> devices;
        
        // 扫描 /dev/ttyACM* 设备
        for (int i = 0; i < 10; ++i) {
            std::string device_path = "/dev/ttyACM" + std::to_string(i);
            
            if (std::filesystem::exists(device_path)) {
                auto device = scan_serial_device(device_path);
                if (!device.serial_number.empty()) {
                    devices.push_back(device);
                }
            }
        }
        
        return devices;
    }
    
    DeviceInfo scan_serial_device(const std::string& device_path)
    {
        DeviceInfo info;
        info.type = "Serial";
        info.path = device_path;
        
        try {
            // 通过sysfs获取设备信息
            std::string dev_name = std::filesystem::path(device_path).filename();
            std::string sys_path = "/sys/class/tty/" + dev_name + "/device";
            
            if (std::filesystem::exists(sys_path)) {
                // 尝试读取序列号
                std::string serial_path = sys_path + "/serial";
                if (std::filesystem::exists(serial_path)) {
                    std::ifstream serial_file(serial_path);
                    if (serial_file.is_open()) {
                        std::getline(serial_file, info.serial_number);
                        serial_file.close();
                        
                        // 去除换行符
                        info.serial_number.erase(info.serial_number.find_last_not_of(" \n\r\t") + 1);
                    }
                }
                
                // 尝试读取VID/PID
                std::string idVendor_path = sys_path + "/../idVendor";
                std::string idProduct_path = sys_path + "/../idProduct";
                
                if (std::filesystem::exists(idVendor_path) && std::filesystem::exists(idProduct_path)) {
                    std::ifstream vid_file(idVendor_path);
                    std::ifstream pid_file(idProduct_path);
                    
                    if (vid_file.is_open() && pid_file.is_open()) {
                        std::string vid_str, pid_str;
                        std::getline(vid_file, vid_str);
                        std::getline(pid_file, pid_str);
                        
                        try {
                            info.vid = std::stoul(vid_str, nullptr, 16);
                            info.pid = std::stoul(pid_str, nullptr, 16);
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(get_logger(), "Failed to parse VID/PID for %s: %s", 
                                       device_path.c_str(), e.what());
                        }
                        
                        vid_file.close();
                        pid_file.close();
                    }
                }
                
                // 检查是否是达妙设备
                if (info.vid != 0x34B7 || info.pid != 0x6877) {
                    // 不是达妙设备，清空信息
                    info.serial_number.clear();
                }
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Error scanning serial device %s: %s", 
                       device_path.c_str(), e.what());
        }
        
        return info;
    }
    
    void display_results(const std::vector<DeviceInfo>& devices, bool publish_info)
    {
        if (devices.empty()) {
            RCLCPP_WARN(get_logger(), "No DamiaoTech USB2CANFD devices found!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Found %zu DamiaoTech device(s):", devices.size());
        
        for (size_t i = 0; i < devices.size(); ++i) {
            const auto& device = devices[i];
            
            std::string info_msg;
            if (device.type == "USB") {
                info_msg = "U2CANFD_DEV " + std::to_string(device.device_index) + ":\n";
            } else {
                info_msg = "U2CANFD_SERIAL " + device.path + ":\n";
            }
            
            info_msg += "  Type: " + device.type + "\n";
            info_msg += "  Path: " + device.path + "\n";
            if (device.vid != 0 && device.pid != 0) {
                info_msg += "  VID: 0x" + to_hex_string(device.vid) + "\n";
                info_msg += "  PID: 0x" + to_hex_string(device.pid) + "\n";
            }
            info_msg += "  SN: " + device.serial_number;
            
            RCLCPP_INFO(get_logger(), "\n%s", info_msg.c_str());
            
            // 发布信息到ROS2话题
            if (publish_info) {
                auto msg = std_msgs::msg::String();
                msg.data = info_msg;
                device_info_pub_->publish(msg);
            }
        }
        
        // 输出使用建议
        if (!devices.empty()) {
            RCLCPP_INFO(get_logger(), "\n=== Usage ===");
            RCLCPP_INFO(get_logger(), "To use with motor_driver, copy the SN and run:");
            RCLCPP_INFO(get_logger(), "ros2 launch motor_driver arm_controller.launch.py device_serial_number:=%s", 
                       devices[0].serial_number.c_str());
        }
    }
    
    std::string to_hex_string(uint16_t value)
    {
        std::stringstream ss;
        ss << std::hex << value;
        return ss.str();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DeviceScannerNode>();
    
    // 运行一次后退出
    rclcpp::spin_some(node);
    
    rclcpp::shutdown();
    return 0;
}