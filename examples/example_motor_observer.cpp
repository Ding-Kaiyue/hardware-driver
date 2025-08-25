#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

// 电机状态观察者 - 打印接收到的数据
class MotorStatusPrinter : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::cout << "[状态] 电机 " << interface << ":" << motor_id 
                  << " | 位置:" << status.position 
                  << " | 速度:" << status.velocity
                  << " | 力矩:" << status.effort 
                  << " | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                  << " | 使能:" << static_cast<int>(status.enable_flag)
                  << " | 模式:" << static_cast<int>(status.motor_mode)
                  << std::endl;
    }
};

int main() {
    std::cout << "=== 电机测试程序 ===" << std::endl;
    
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}}
    };
    
    try {
        // 创建CAN总线
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        
        // 创建电机驱动
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
        
        // 添加状态观察者
        auto status_printer = std::make_shared<MotorStatusPrinter>();
        motor_driver->add_observer(status_printer);
        
        // 创建硬件接口
        RobotHardware robot(motor_driver, motor_config);
        
        std::cout << "硬件初始化完成" << std::endl;
        
        // 测试电机1 - 从配置中获取电机ID
        uint32_t test_motor1 = motor_config["can0"][0];
        uint32_t test_motor2 = motor_config["can0"][1];
        
        std::cout << "\n步骤1: 使能电机 " << test_motor1 << test_motor2 << std::endl;
        // robot.enable_motor("can0", test_motor, 4);  // 模式4
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n步骤2: 控制电机运动" << std::endl;
        
        // 正向旋转
        std::cout << "正向旋转 5 度/秒..." << std::endl;
        for (int i = 0; i < 200000; i++) {
            robot.control_motor_in_velocity_mode("can0", test_motor1, 5.0);
            robot.control_motor_in_velocity_mode("can0", test_motor2, 5.0);
        }
        // robot.control_motor_in_velocity_mode("can0", test_motor1, 5.0);
        // robot.control_motor_in_velocity_mode("can0", test_motor2, 5.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 停止
        std::cout << "停止..." << std::endl;
        robot.control_motor_in_velocity_mode("can0", test_motor1, 0.0);
        robot.control_motor_in_velocity_mode("can0", test_motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 反向旋转
        std::cout << "反向旋转 -3 度/秒..." << std::endl;
        robot.control_motor_in_velocity_mode("can0", test_motor1, -3.0);
        robot.control_motor_in_velocity_mode("can0", test_motor2, -3.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 停止
        std::cout << "停止..." << std::endl;
        robot.control_motor_in_velocity_mode("can0", test_motor1, 0.0);
        robot.control_motor_in_velocity_mode("can0", test_motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n步骤3: 失能电机" << std::endl;
        robot.disable_motor("can0", test_motor1);
        robot.disable_motor("can0", test_motor2);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n测试完成！" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "请检查：" << std::endl;
        std::cerr << "1. 电机是否连接到can0总线" << std::endl;
        std::cerr << "2. 电机ID是否为1（可修改motor_config中的ID）" << std::endl;
        std::cerr << "3. 电机是否已上电" << std::endl;
        return 1;
    }
    
    return 0;
}
