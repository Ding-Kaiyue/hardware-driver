#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <array>

/**
 * @file example_batch_control.cpp
 * @brief 电机批量控制命令示例程序
 *
 * 本示例展示如何使用 RobotHardware 提供的批量控制接口：
 * - send_realtime_velocity_command()  : 批量速度控制
 * - send_realtime_position_command()  : 批量位置控制
 * - send_realtime_effort_command()    : 批量力矩控制
 * - send_realtime_mit_command()       : 批量 MIT 模式控制
 *
 * 这些接口通过一次CAN总线通信控制多个电机，提高控制效率。
 */

// 简单的电机状态观察者
class MotorStatusPrinter : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    void on_motor_status_update(const std::string& interface, uint32_t motor_id,
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::cout << "[" << interface << ":" << motor_id << "] "
                  << "位置:" << status.position
                  << " | 速度:" << status.velocity
                  << " | 力矩:" << status.effort
                  << std::endl;
    }
};

int main() {
    std::cout << "=== 电机批量控制示例程序 ===" << std::endl;

    // 配置电机接口和ID映射
    // 假设 can0 接口上有 6 个电机（ID: 1, 2, 3, 4, 5, 6）
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3, 4, 5, 6}}
    };

    // 使用工厂函数创建CANFD电机驱动
    auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});

    try {
        // 创建观察者和机器人硬件接口
        auto status_printer = std::make_shared<MotorStatusPrinter>();
        auto robot = std::make_unique<RobotHardware>(motor_driver, motor_config, status_printer);

        std::cout << "硬件初始化完成，开始批量控制测试...\n" << std::endl;

        // ============================================================
        // 示例 1: 批量速度控制 - 所有电机以相同速度运动
        // ============================================================
        std::cout << "\n=== 示例 1: 批量速度控制 ===" << std::endl;
        std::cout << "所有电机以 10 度/秒 的速度运动..." << std::endl;

        uint8_t motor_mode = 4;
        // 批量电机使能
        robot->enable_motors("can0", motor_config["can0"], motor_mode);
        std::cout << "✅ 批量电机使能到速度模式成功" << std::endl;

        // 为 6 个电机设置速度（使用 std::array 避免动态分配）
        std::array<double, 6> velocities = {-10.0, -10.0, -10.0, -10.0, -10.0, -10.0};

        // 调用批量速度控制接口（不指定 kps 和 kds，使用默认值 0.0）
        if (robot->send_realtime_velocity_command("can0", velocities)) {
            std::cout << "✅ 批量速度命令发送成功" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        robot->disable_motors("can0", motor_config["can0"], motor_mode);
        std::cout << "✅ 批量电机失能成功" << std::endl;

        // ============================================================
        // 示例 2: 批量位置控制 - 所有电机运动到零位
        // ============================================================
        std::cout << "\n=== 示例 2: 批量位置控制 ===" << std::endl;
        std::cout << "所有电机运动到零位..." << std::endl;

        motor_mode = 5;
        // 批量电机使能
        robot->enable_motors("can0", motor_config["can0"], motor_mode);
        std::cout << "✅ 批量电机使能到位置模式成功" << std::endl;

        // 为 6 个电机设置目标位置（使用 std::array 避免动态分配）
        std::array<double, 6> positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // 调用批量位置控制接口（ kps = 0.05, kds = 0.01）
        std::array<double, 6> kps = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
        std::array<double, 6> kds = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
        if (robot->send_realtime_position_command("can0", positions, kps, kds)) {
            std::cout << "✅ 批量位置命令发送成功" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // 批量电机失能
        robot->disable_motors("can0", motor_config["can0"], motor_mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (const std::exception& e) {
        std::cerr << "❌ 错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
