#include "hardware_driver/hardware_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

// 全局计数器，统计回调次数
std::atomic<int> callback_count{0};
std::atomic<bool> running{true};

// 电机状态回调函数
void motor_status_callback(const std::string& interface, uint32_t motor_id, const hardware_driver::motor_driver::Motor_Status& status) {
    callback_count++;
    
    // 每100次回调打印一次状态信息
    if (callback_count % 100 == 0) {
        std::cout << "[Callback " << callback_count << "] Interface: " << interface 
                  << ", Motor: " << motor_id 
                  << ", Position: " << status.position 
                  << ", Velocity: " << status.velocity << std::endl;
    }
}

int main(){
    std::vector<std::string> interface = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {{"can0", {9}}};
    
    try{
        // 使用带回调的构造函数
        hardware_driver::HardwareDriver driver(interface, motor_config, motor_status_callback);
        
        std::cout << "开始测试回调功能..." << std::endl;
        std::cout << "系统将自动推送电机状态数据到回调函数" << std::endl;
        std::cout << "频率：高频模式2.5kHz，低频模式20Hz" << std::endl;
        
        // 使能电机
        driver.enable_motor("can0", 9, 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 运行10秒，观察回调频率
        auto start_time = std::chrono::steady_clock::now();
        auto last_count = 0;
        
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            auto current_count = callback_count.load();
            auto frequency = current_count - last_count;
            
            std::cout << "1秒内收到回调: " << frequency << " 次，累计: " << current_count << " 次" << std::endl;
            last_count = current_count;
        }
        
        std::cout << "测试完成！" << std::endl;
        std::cout << "总回调次数: " << callback_count.load() << std::endl;
        std::cout << "平均频率: " << callback_count.load() / 10.0 << " Hz" << std::endl;
        
        // 失能电机
        driver.disable_motor("can0", 9);
        
    } catch(const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}