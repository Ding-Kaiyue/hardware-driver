#include <iostream>
#include "hardware_driver.hpp"

int main() {
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {{"can0", {1, 2, 3, 4}}};
    
    try {
        hardware_driver::HardwareDriver driver(interfaces, motor_config);
        std::cout << "HardwareDriver created successfully!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
}