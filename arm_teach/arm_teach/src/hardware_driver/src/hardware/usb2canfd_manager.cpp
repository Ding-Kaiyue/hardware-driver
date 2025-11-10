#include "hardware/usb2canfd_manager.h"
#include "tools/usb_device_finder.h"
#include <iostream>



pthread_once_t Usb2CanfdManager::init_once = PTHREAD_ONCE_INIT;
std::atomic_bool Usb2CanfdManager::high_waiting{false};
pthread_mutex_t Usb2CanfdManager::send_mutex;
pthread_cond_t Usb2CanfdManager::send_cond;

std::shared_ptr<Usb2CanfdManager> Usb2CanfdManager::createFromDevicePath(
    const std::string& device_path,
    uint32_t nominal_bitrate,
    uint32_t data_bitrate)
{
    init_once = PTHREAD_ONCE_INIT;
    high_waiting = false;

    // 通过设备路径获取序列号
    std::string device_sn = UsbDeviceFinder::getDmCanfdSerialNumber(device_path);

    if (device_sn.empty()) {
        std::cerr << "[Usb2CanfdManager] Failed to get serial number from device path: " << device_path << std::endl;
        return nullptr;
    }

    std::cout << "[Usb2CanfdManager] Found device SN from path " << device_path << ": " << device_sn << std::endl;

    // 创建并返回管理器实例
    return std::make_shared<Usb2CanfdManager>(device_sn, nominal_bitrate, data_bitrate);
}

Usb2CanfdManager::Usb2CanfdManager(
    const std::string& device_sn,
    uint32_t nominal_bitrate,
    uint32_t data_bitrate)
    : device_sn_(device_sn)
    , nominal_bitrate_(nominal_bitrate)
    , data_bitrate_(data_bitrate)
{
    // 初始化USB硬件
    usb_hw_ = std::make_shared<usb_class>(nominal_bitrate, data_bitrate, device_sn);

    if (!usb_hw_) 
    {
        throw std::runtime_error("Failed to create USB hardware interface");
    }

    std::cout << "[Usb2CanfdManager] Initialized with device: " << device_sn << std::endl;
    std::cout << "[Usb2CanfdManager] Nominal bitrate: " << nominal_bitrate << " bps" << std::endl;
    std::cout << "[Usb2CanfdManager] Data bitrate: " << data_bitrate << " bps" << std::endl;
}

Usb2CanfdManager::~Usb2CanfdManager()
{
    std::cout << "[Usb2CanfdManager] Shutting down..." << std::endl;
    usb_hw_.reset();
}

bool Usb2CanfdManager::sendFrame(const canfd_frame& frame, const bool is_high_priority)
{
    pthread_once(&init_once, init_mutex);

    // 如果是高优先级任务，立即设置标志（在获取锁之前）
    if (is_high_priority)
    {
        high_waiting = true;
    }

    pthread_mutex_lock(&send_mutex);

    // 如果是低优先级任务，当有高优先级任务等待时，进入条件变量等待
    if (!is_high_priority)
    {
        while (high_waiting.load(std::memory_order_relaxed))
        {
            pthread_cond_wait(&send_cond, &send_mutex);
        }
    }

    if (!usb_hw_) {
        pthread_mutex_unlock(&send_mutex);
        std::cerr << "[Usb2CanfdManager] Error: Hardware not initialized" << std::endl;

        return false;
    }

    // 创建CAN TX帧结构体并设置扩展帧标志
    can_tx_type frame_tx{};
    frame_tx.can_id = frame.can_id;      // CAN ID
    frame_tx.can_type = 1;   // CAN FD
    frame_tx.fd_acc = 1;
    frame_tx.fram_type = 1;
    frame_tx.id_type = 1;    // 扩展帧
    frame_tx.id_increase = 0;
    frame_tx.cmd = 0;
    frame_tx.send_time = 1;
    frame_tx.interval = 0;

    // 将实际数据长度向上取整到CANFD支持的长度
    frame_tx.dlc = RoundUpLen(frame.len);

    // 复制数据到发送缓冲区
    std::copy(frame.data, frame.data + frame.len, frame_tx.data);

    usb_hw_->set_tx_frame(&frame_tx);

    usb_hw_->send_data();

    // 发送完成后，如果是高优先级任务，清除标志并唤醒等待的低优先级线程
    if (is_high_priority)
    {
        high_waiting = false;
        pthread_cond_broadcast(&send_cond);
    }

    pthread_mutex_unlock(&send_mutex);

    return true;
}

void Usb2CanfdManager::setReceiveCallback(CanFrameCallback callback)
{
    receive_callback_ = callback;

    if (usb_hw_) 
    {
        // 使用lambda适配器将usb_class的回调格式转换为标准格式
        auto adapter = [this](can_value_type& value) 
        {
            this->internalFrameCallback(value);
        };

        usb_hw_->setFrameCallback(adapter);
    }
}

bool Usb2CanfdManager::isReady() const
{
    return usb_hw_ != nullptr;
}

std::string Usb2CanfdManager::getHardwareIdentifier() const
{
    return device_sn_;
}

bool Usb2CanfdManager::configureBitrate(uint32_t nominal_bitrate, uint32_t data_bitrate)
{
    nominal_bitrate_ = nominal_bitrate;
    data_bitrate_ = data_bitrate;

    std::cout << "[Usb2CanfdManager] Bitrate reconfiguration requested" << std::endl;
    std::cout << "[Usb2CanfdManager] Note: Runtime reconfiguration not supported" << std::endl;

    return false;
}

std::shared_ptr<usb_class> Usb2CanfdManager::getUsbHardware() const
{
    return usb_hw_;
}

void Usb2CanfdManager::internalFrameCallback(can_value_type& value)
{
    if (!receive_callback_) 
    {
        return;
    }

    // 将can_value_type转换为标准canfd_frame
    canfd_frame frame = canValueTypeToFrame(value);

    // 调用用户注册的回调
    receive_callback_(frame);
}

uint8_t Usb2CanfdManager::Len2DLC(const uint8_t length) const
{
    uint8_t dlc = 0;
    if (length <= 8) 
    {
        dlc = length;
    } 
    else if (length <= 12) 
    {
        dlc = 9;
    } 
    else if (length <= 16) 
    {
        dlc = 10;
    } 
    else if (length <= 20) 
    {
        dlc = 11;
    } 
    else if (length <= 24) 
    {
        dlc = 12;
    } 
    else if (length <= 32) 
    {
        dlc = 13;
    } 
    else if (length <= 48) 
    {
        dlc = 14;
    } 
    else if (length <= 64) 
    {
        dlc = 15;
    } 
    else 
    {
        dlc = 0; // Invalid length
    }
    return dlc;
}

uint8_t Usb2CanfdManager::DLC2Len(uint8_t dlc) const
{
    uint8_t length = 0;
    if (dlc <= 8) 
    {
        length = dlc;
    } 
    else if (dlc == 9) 
    {
        length = 12;
    } 
    else if (dlc == 10) 
    {
        length = 16;
    } 
    else if (dlc == 11) 
    {
        length = 20;
    } 
    else if (dlc == 12) 
    {
        length = 24;
    } 
    else if (dlc == 13) 
    {
        length = 32;
    } 
    else if (dlc == 14) 
    {
        length = 48;
    } 
    else if (dlc == 15) 
    {
        length = 64;
    } 
    else 
    {
        length = 0; // Invalid DLC
    }
    return length;
}

// 将长度向上取整到CANFD支持的长度: 0-8, 12, 16, 20, 24, 32, 48, 64
uint8_t Usb2CanfdManager::RoundUpLen(uint8_t length)
{
    if (length <= 8) {
        return length;
    } else if (length <= 12) {
        return 12;
    } else if (length <= 16) {
        return 16;
    } else if (length <= 20) {
        return 20;
    } else if (length <= 24) {
        return 24;
    } else if (length <= 32) {
        return 32;
    } else if (length <= 48) {
        return 48;
    } else {
        return 64;  // length > 48, 返回最大值64
    }
}

canfd_frame Usb2CanfdManager::canValueTypeToFrame(const can_value_type& value) const
{
    canfd_frame frame{};
    frame.can_id = value.head.id;
    frame.len = DLC2Len(value.head.dlc);

    // 复制数据
    uint8_t len = frame.len;
    std::copy(value.data, value.data + len, frame.data);

    return frame;
}
