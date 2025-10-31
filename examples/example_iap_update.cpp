/*******************************************************************************
 * @file        example_iap_update.cpp
 * @brief       IAP固件更新示例程序
 *
 * 这个示例演示如何使用IAP管理器进行单电机固件更新
 *
 * @author      Hardware Driver Team
 * @version     0.0.1.0
 * @date        2025-10-30
 *******************************************************************************/

#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;

class IAPObserver : public hardware_driver::motor_driver::IAPStatusObserver {
public:
    void on_iap_status_feedback(const std::string& interface,
                                uint32_t motor_id,
                                const hardware_driver::iap_protocol::IAPStatusMessage& msg) override {
        using namespace hardware_driver::iap_protocol;
        std::cout << "[IAP反馈] " << interface << ":" << motor_id << " -> "
                  << iap_status_to_string(msg) << std::endl;
        switch (msg) {
            case IAPStatusMessage::BS00: std::cout << "  Bootloader 启动\n"; break;
            case IAPStatusMessage::BK01: std::cout << "  收到 Key，进入 IAP 模式\n"; break;
            case IAPStatusMessage::BK03: std::cout << "  开始接收固件数据\n"; break;
            case IAPStatusMessage::BD05: std::cout << "  数据接收完成，校验中\n"; break;
            case IAPStatusMessage::AS00: std::cout << "  APP 启动成功 ✅\n"; break;
            default: break;
        }
    }
};

// ========== 主程序入口 ==========
int main() {
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1}}    // can0 接口上的电机 1
    };

    auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
    auto iap_observer = std::make_shared<IAPObserver>();
    auto robot = std::make_shared<RobotHardware>(motor_driver, motor_config, iap_observer);

    std::cout << "====== IAP Firmware Update Test ======" << std::endl;
    robot->start_update("can0", 1, "../../update_files/9NM.bin");
    std::cout << "====== IAP Firmware Update Test End ======" << std::endl;

    return 0;
}
