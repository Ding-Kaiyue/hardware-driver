#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"

#define CAN0_ENABLE
#define CAN1_ENABLE

// 简单的Mock MotorDriverInterface用于测试
class MockMotorDriver : public hardware_driver::motor_driver::MotorDriverInterface {
public:
    MOCK_METHOD(void, enable_motor, (const std::string interface, const uint32_t motor_id, uint8_t mode), (override));
    MOCK_METHOD(void, send_position_cmd, (const std::string interface, const uint32_t motor_id, float position), (override));
    MOCK_METHOD(void, send_velocity_cmd, (const std::string interface, const uint32_t motor_id, float velocity), (override));
    MOCK_METHOD(void, send_effort_cmd, (const std::string interface, const uint32_t motor_id, float effort), (override));
    MOCK_METHOD(void, send_mit_cmd, (const std::string interface, const uint32_t motor_id, float position, float velocity, float effort), (override));
    MOCK_METHOD(void, disable_motor, (const std::string interface, const uint32_t motor_id), (override));
    MOCK_METHOD(void, motor_parameter_read, (const std::string interface, const uint32_t motor_id, uint16_t address), (override));
    MOCK_METHOD(void, motor_parameter_write, (const std::string interface, const uint32_t motor_id, uint16_t address, int32_t value), (override));
    MOCK_METHOD(void, motor_parameter_write, (const std::string interface, const uint32_t motor_id, uint16_t address, float value), (override));
    MOCK_METHOD(void, motor_function_operation, (const std::string interface, const uint32_t motor_id, uint8_t operation), (override));
    MOCK_METHOD(void, motor_feedback_request, (const std::string interface, const uint32_t motor_id), (override));
    MOCK_METHOD(void, motor_feedback_request_all, (const std::string interface), (override));
    MOCK_METHOD(hardware_driver::motor_driver::Motor_Status, get_motor_status, (const std::string& interface, uint32_t motor_id), (override));
};

using ::testing::_;

TEST(RobotHardwareTest, VelocityAndDisable) {
    // 创建CAN总线接口 - 使用默认波特率
    std::vector<std::string> interfaces = {"can1"};
    auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);

    // 使用真实的电机驱动
    auto real_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    std::map<std::string, std::vector<uint32_t>> config = {
        // {"can0", {1, 2, 3, 4, 5, 6, 7, 8}},
        {"can1", {1, 2, 3, 4, 7, 9, 10, 12}}
    };
    RobotHardware hw(real_driver, config);

    std::cout << "=== 速度指令和失能测试 ===" << std::endl;
    
    // 等待系统稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 发送速度命令让电机转动，代码内部已处理时序控制
    std::cout << "发送can0 速度命令 (50.0 degrees/s)" << std::endl;
    hw.control_motor_in_velocity_mode("can0", 1, 50.0f);
    hw.control_motor_in_velocity_mode("can0", 2, 50.0f);
    hw.control_motor_in_velocity_mode("can0", 3, 50.0f);
    hw.control_motor_in_velocity_mode("can0", 4, 50.0f);
    hw.control_motor_in_velocity_mode("can0", 5, 50.0f);
    hw.control_motor_in_velocity_mode("can0", 6, 50.0f);
    hw.control_motor_in_velocity_mode("can0", 7, 50.0f);
    hw.control_motor_in_velocity_mode("can0", 8, 50.0f);
    std::cout << "发送can1 速度命令 (50.0 degrees/s)" << std::endl;
    hw.control_motor_in_velocity_mode("can1", 1, 50.0f);
    hw.control_motor_in_velocity_mode("can1", 2, 50.0f);
    hw.control_motor_in_velocity_mode("can1", 3, 50.0f);
    hw.control_motor_in_velocity_mode("can1", 4, 50.0f);
    hw.control_motor_in_velocity_mode("can1", 7, 50.0f);
    hw.control_motor_in_velocity_mode("can1", 9, 50.0f);
    hw.control_motor_in_velocity_mode("can1", 10, 50.0f);
    hw.control_motor_in_velocity_mode("can1", 12, 50.0f);
    
    // 等待电机转动
    std::cout << "等待电机转动..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 检查电机状态，看看哪些电机在转动
    std::cout << "检查电机转动状态:" << std::endl;
    for (int i = 1; i <= 8; ++i) {
        auto status = hw.get_motor_status("can0", i);
        std::cout << "  电机" << i << " 速度: " << status.velocity 
                  << " 使能标志: " << (int)status.enable_flag 
                  << " 电机模式: " << (int)status.motor_mode << std::endl;
    }
    for (int i = 1; i <= 4; ++i) {
        auto status = hw.get_motor_status("can1", i);
        std::cout << "  电机" << i << " 速度: " << status.velocity 
                  << " 使能标志: " << (int)status.enable_flag 
                  << " 电机模式: " << (int)status.motor_mode << std::endl;
    }
    auto status7 = hw.get_motor_status("can1", 7);
    std::cout << "  电机" << 7 << " 速度: " << status7.velocity 
              << " 使能标志: " << (int)status7.enable_flag 
              << " 电机模式: " << (int)status7.motor_mode << std::endl;
    auto status9 = hw.get_motor_status("can1", 9);
    std::cout << "  电机" << 9 << " 速度: " << status9.velocity 
              << " 使能标志: " << (int)status9.enable_flag 
              << " 电机模式: " << (int)status9.motor_mode << std::endl;
    auto status10 = hw.get_motor_status("can1", 10);
    std::cout << "  电机" << 10 << " 速度: " << status10.velocity 
              << " 使能标志: " << (int)status10.enable_flag 
              << " 电机模式: " << (int)status10.motor_mode << std::endl;
    auto status12 = hw.get_motor_status("can1", 12);
    std::cout << "  电机" << 12 << " 速度: " << status12.velocity 
              << " 使能标志: " << (int)status12.enable_flag 
              << " 电机模式: " << (int)status12.motor_mode << std::endl;

    // 发送速度0命令停止电机，分批发送
    std::cout << "发送can0 速度0命令停止电机" << std::endl;
    hw.control_motor_in_velocity_mode("can0", 1, 0.0f);
    hw.control_motor_in_velocity_mode("can0", 2, 0.0f);
    hw.control_motor_in_velocity_mode("can0", 3, 0.0f);
    hw.control_motor_in_velocity_mode("can0", 4, 0.0f);
    hw.control_motor_in_velocity_mode("can0", 5, 0.0f);
    hw.control_motor_in_velocity_mode("can0", 6, 0.0f);
    hw.control_motor_in_velocity_mode("can0", 7, 0.0f);
    hw.control_motor_in_velocity_mode("can0", 8, 0.0f);
    std::cout << "发送can1 速度0命令停止电机" << std::endl;
    hw.control_motor_in_velocity_mode("can1", 1, 0.0f);
    hw.control_motor_in_velocity_mode("can1", 2, 0.0f);
    hw.control_motor_in_velocity_mode("can1", 3, 0.0f);
    hw.control_motor_in_velocity_mode("can1", 4, 0.0f);
    hw.control_motor_in_velocity_mode("can1", 7, 0.0f);
    hw.control_motor_in_velocity_mode("can1", 9, 0.0f);
    hw.control_motor_in_velocity_mode("can1", 10, 0.0f);
    hw.control_motor_in_velocity_mode("can1", 12, 0.0f);
    // 等待电机完全停止
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 失能电机
    std::cout << "失能电机can0 motor" << std::endl;
    hw.disable_motor("can0", 1);
    hw.disable_motor("can0", 2);
    hw.disable_motor("can0", 3);
    hw.disable_motor("can0", 4);
    hw.disable_motor("can0", 5);
    hw.disable_motor("can0", 6);
    hw.disable_motor("can0", 7);
    hw.disable_motor("can0", 8);
    std::cout << "失能电机can1 motor" << std::endl;
    hw.disable_motor("can1", 1);
    hw.disable_motor("can1", 2);
    hw.disable_motor("can1", 3);
    hw.disable_motor("can1", 4);
    hw.disable_motor("can1", 7);
    hw.disable_motor("can1", 9);
    hw.disable_motor("can1", 10);
    hw.disable_motor("can1", 12);
    // 等待失能命令生效
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 获取电机状态
    auto can0_status = hw.get_all_motor_status("can0");
    std::cout << "can0电机状态:" << std::endl;
    for (const auto& [key, status] : can0_status) {
        std::cout << "can0 motor" << key.second << "  使能标志: " << (int)status.enable_flag << std::endl;
        std::cout << "can0 motor" << key.second << "  速度: " << status.velocity << std::endl;
    }    
    auto can1_status = hw.get_all_motor_status("can1");
    std::cout << "can1电机状态:" << std::endl;
    for (const auto& [key, status] : can1_status) {
        std::cout << "can1 motor" << key.second << "  使能标志: " << (int)status.enable_flag << std::endl;
        std::cout << "can1 motor" << key.second << "  速度: " << status.velocity << std::endl;
    }
}
