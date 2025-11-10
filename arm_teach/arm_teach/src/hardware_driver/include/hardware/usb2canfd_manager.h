#ifndef USB2CANFD_MANAGER_H
#define USB2CANFD_MANAGER_H

#include <memory>
#include <string>
#include <functional>
#include <linux/can.h>
#include "protocol/usb_class.h"
#include <pthread.h>

/**
 * @file usb2canfd_manager.h
 * @brief USB2CANFD硬件通信管理器
 *
 * 实现ICommunicationManager接口，封装USB2CANFD硬件的具体操作。
 * 此类完全不知道上层使用的电机协议，只负责CAN帧的收发。
 *
 * 特性：
 * - 完全解耦的硬件层
 * - 标准化的接口
 * - 易于替换为其他CAN硬件（如SocketCAN）
 * - 向后兼容现有的usb_class
 * - 支持通过设备路径自动获取SN
 */
using CanFrameCallback = std::function<void(const canfd_frame& frame)>;

class Usb2CanfdManager
{
public:
    /**
     * @brief 构造函数 (通过设备序列号)
     * @param device_sn USB设备序列号
     * @param nominal_bitrate 标称波特率（默认1Mbps）
     * @param data_bitrate 数据波特率（默认5Mbps，CANFD）
     */
    explicit Usb2CanfdManager(
        const std::string& device_sn,
        uint32_t nominal_bitrate = 1000000,
        uint32_t data_bitrate = 5000000
    );

    /**
     * @brief 构造函数 (通过设备路径自动获取SN)
     * @param device_path 设备路径 (如 /dev/ttyACM0)
     * @param nominal_bitrate 标称波特率（默认1Mbps）
     * @param data_bitrate 数据波特率（默认5Mbps，CANFD）
     *
     * @note 此构造函数会自动通过设备路径查询序列号
     */
    static std::shared_ptr<Usb2CanfdManager> createFromDevicePath(
        const std::string& device_path,
        uint32_t nominal_bitrate = 1000000,
        uint32_t data_bitrate = 5000000
    );
    
    /**
     * @brief 析构函数
     */
    ~Usb2CanfdManager();

    static void init_mutex()
    {
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        pthread_mutex_init(&send_mutex, &attr);
        pthread_mutexattr_destroy(&attr);

        pthread_cond_init(&send_cond, nullptr);
    }

    // ========== ICommunicationManager接口实现 ==========

    /**
     * @brief 发送CAN帧
     * @param frame 要发送的CAN帧
     * @return 发送是否成功
     */
    bool sendFrame(const canfd_frame& frame, const bool is_high_priority = true);

    /**
     * @brief 设置CAN帧接收回调
     * @param callback 接收回调函数
     */
    void setReceiveCallback(CanFrameCallback callback);

    /**
     * @brief 检查硬件是否就绪
     * @return 硬件是否可用
     */
    bool isReady() const;

    /**
     * @brief 获取硬件标识信息
     * @return USB设备序列号
     */
    std::string getHardwareIdentifier() const;

    /**
     * @brief 配置CAN总线参数
     * @param nominal_bitrate 标称波特率
     * @param data_bitrate 数据波特率（CANFD）
     * @return 配置是否成功
     */
    bool configureBitrate(uint32_t nominal_bitrate, uint32_t data_bitrate = 0);

    // ========== USB2CANFD特有方法 ==========

    /**
     * @brief 获取底层USB硬件接口（兼容旧代码）
     * @return USB硬件共享指针
     */
    std::shared_ptr<usb_class> getUsbHardware() const;

private:
    /**
     * @brief 内部回调适配器
     * @param value 原始can_value_type数据
     *
     * 将usb_class的can_value_type转换为标准canfd_frame格式
     */
    void internalFrameCallback(can_value_type& value);

    /**
     * @brief 将canfd_frame转换为can_value_type
     * @param frame 标准CAN帧
     * @return can_value_type数据
     */
    can_value_type frameToCanValueType(const canfd_frame& frame) const;

    /**
     * @brief 将can_value_type转换为canfd_frame
     * @param value can_value_type数据
     * @return 标准CAN帧
     */
    canfd_frame canValueTypeToFrame(const can_value_type& value) const;

    uint8_t Len2DLC(const uint8_t length) const;

    uint8_t DLC2Len(uint8_t dlc) const;

    uint8_t RoundUpLen(uint8_t length);

private:
    std::shared_ptr<usb_class> usb_hw_;         // 底层USB硬件
    std::string device_sn_;                      // 设备序列号
    uint32_t nominal_bitrate_;                   // 标称波特率
    uint32_t data_bitrate_;                      // 数据波特率
    CanFrameCallback receive_callback_;          // 用户注册的接收回调

    static pthread_mutex_t send_mutex;
    static pthread_cond_t send_cond;
    static pthread_once_t init_once;
    static std::atomic_bool high_waiting;
};

#endif // USB2CANFD_MANAGER_H
