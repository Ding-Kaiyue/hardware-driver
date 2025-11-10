#ifndef USB_DEVICE_FINDER_H
#define USB_DEVICE_FINDER_H

#include <string>
#include <vector>

/**
 * @brief USB设备查找工具
 *
 * 用于通过VID/PID或设备路径查找USB设备的序列号
 */
class UsbDeviceFinder
{
public:
    /**
     * @struct DeviceInfo
     * @brief USB设备信息
     */
    struct DeviceInfo
    {
        uint16_t vid;                    // 厂商ID
        uint16_t pid;                    // 产品ID
        std::string serial_number;       // 序列号
        std::string device_path;         // 设备路径（如果可用）
    };

    /**
     * @brief 获取所有匹配VID/PID的USB设备
     * @param vid 厂商ID
     * @param pid 产品ID
     * @return 设备信息列表
     */
    static std::vector<DeviceInfo> getDevicesByVidPid(uint16_t vid, uint16_t pid);

    /**
     * @brief 通过设备路径获取序列号
     * @param device_path 设备路径（如 /dev/ttyACM0）
     * @return 序列号，如果失败返回空字符串
     *
     * @note 该方法通过udev或sysfs来获取设备序列号
     */
    static std::string getSerialNumberByPath(const std::string& device_path);

    /**
     * @brief 获取DM-CANFD设备的序列号
     * @param device_path 设备路径（如 /dev/ttyACM0）
     * @return 序列号
     *
     * @note DM-CANFD的VID=0x34B7, PID=0x6877
     */
    static std::string getDmCanfdSerialNumber(const std::string& device_path);

    /**
     * @brief 列出所有DM-CANFD设备
     * @return DM-CANFD设备信息列表
     */
    static std::vector<DeviceInfo> listDmCanfdDevices();

private:
    static constexpr uint16_t DM_CANFD_VID = 0x34B7;   // DM-CANFD厂商ID
    static constexpr uint16_t DM_CANFD_PID = 0x6877;   // DM-CANFD产品ID
};

#endif // USB_DEVICE_FINDER_H
