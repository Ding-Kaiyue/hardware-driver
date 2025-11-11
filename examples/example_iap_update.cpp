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
                                const IAPStatus& msg) override {
        std::cout << "[IAP反馈] " << interface << ":" << motor_id << " -> ";

        // 根据状态值打印对应的状态消息
        switch (msg) {
            case IAPStatus::BS00: std::cout << "BS00 (Bootloader启动)\n"; break;
            case IAPStatus::BK01: std::cout << "BK01 (收到Key，进入IAP模式)\n"; break;
            case IAPStatus::BK02: std::cout << "BK02 (擦除APP程序)\n"; break;
            case IAPStatus::BK03: std::cout << "BK03 (准备接收固件数据)\n"; break;
            case IAPStatus::BD04: std::cout << "BD04 (开始接收数据)\n"; break;
            case IAPStatus::BD05: std::cout << "BD05 (数据接收完成)\n"; break;
            case IAPStatus::BJ06: std::cout << "BJ06 (APP地址正确，准备跳转)\n"; break;
            case IAPStatus::BJ07: std::cout << "BJ07 (跳转错误)\n"; break;
            case IAPStatus::AS00: std::cout << "AS00 (APP启动成功 ✅)\n"; break;
            case IAPStatus::AJ01: std::cout << "AJ01 (APP收到IAP更新指令)\n"; break;
            default: std::cout << "Unknown status: " << static_cast<uint32_t>(msg) << "\n"; break;
        }
    }
};

// ========== 主程序入口 ==========
int main() {
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {6}}    // can0 接口上的电机 2
    };

    auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
    auto iap_observer = std::make_shared<IAPObserver>();
    auto robot = std::make_shared<RobotHardware>(motor_driver, motor_config, iap_observer);

    std::cout << "====== IAP Firmware Update Test ======" << std::endl;
    robot->start_update("can0", motor_config["can0"][0], "../../update_files/51NM.bin");
    std::cout << "====== IAP Firmware Update Test End ======" << std::endl;

    return 0;
}
