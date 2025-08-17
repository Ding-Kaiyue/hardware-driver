#include "hardware_driver.hpp"
#include <iostream>

int main() {
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3, 4}}
    };
    std::map<std::string, std::string> label_to_interface_map = {
        {"arm_left", "can0"}
    };
    
    try {
        // 创建驱动
        hardware_driver::HardwareDriver driver(interfaces, motor_config, label_to_interface_map);
        
        // 使能电机
        driver.enable_motor("can0", 1, 4);
        
        // 控制电机
        driver.control_motor_in_velocity_mode("can0", 1, 5.0);
        
        // 获取状态
        auto status = driver.get_motor_status("can0", 1);
        std::cout << "位置: " << status.position << std::endl;
        
        // 失能电机
        driver.disable_motor("can0", 1);
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}