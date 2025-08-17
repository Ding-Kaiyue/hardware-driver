#include "hardware_driver/hardware_driver.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "=== 硬件驱动简单示例 ===" << std::endl;
    
    // 配置CAN接口和电机
    std::vector<std::string> interfaces = {"can0", "can1"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3, 4}},
        {"can1", {1, 2, 3, 4, 5, 6, 7, 8}}
    };
    
    try {
        // 创建硬件驱动实例
        hardware_driver::HardwareDriver driver(interfaces, motor_config);
        
        std::cout << "硬件驱动初始化成功！" << std::endl;
        
        // 使能电机
        std::cout << "使能电机..." << std::endl;
        driver.enable_motor("can0", 1, 4);  // 使能can0上的电机1，模式4
        driver.enable_motor("can0", 2, 4);  // 使能can0上的电机2，模式4
        
        // 控制电机
        std::cout << "控制电机..." << std::endl;
        driver.control_motor_in_velocity_mode("can0", 1, 10.0);  // 速度模式，10 degrees/s
        driver.control_motor_in_position_mode("can0", 2, 90.0);  // 位置模式，90 degrees
        
        // 获取电机状态
        std::cout << "获取电机状态..." << std::endl;
        auto status = driver.get_motor_status("can0", 1);
        std::cout << "电机1状态 - 位置: " << status.position 
                  << ", 速度: " << status.velocity 
                  << ", 力矩: " << status.effort << std::endl;
        
        // 运行一段时间
        std::cout << "运行5秒..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // 失能电机
        std::cout << "失能电机..." << std::endl;
        driver.disable_motor("can0", 1);
        driver.disable_motor("can0", 2);
        
        std::cout << "示例完成！" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 