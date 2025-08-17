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
        std::cout << "成功创建 HardwareDriver!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
}