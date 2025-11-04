#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

// 简单的电机状态观察者示例
class MotorStatusPrinter : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    // 必须实现的纯虚函数
    void on_motor_status_update(const std::string& interface, uint32_t motor_id,
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::cout << "[观察者] 电机 " << interface << ":" << motor_id 
                  << " | 位置:" << status.position 
                  << " | 速度:" << status.velocity
                  << " | 力矩:" << status.effort 
                  << " | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                  << " | 使能:" << static_cast<int>(status.enable_flag)
                  << " | 模式:" << static_cast<int>(status.motor_mode)
                  << std::endl;
    }
    
    // 重写函数操作结果事件处理
    void on_motor_function_result(const std::string& interface,
                                 uint32_t motor_id,
                                 uint8_t op_code,
                                 bool success) override {
        std::cout << "🔧 电机操作结果: " << interface << ":" << motor_id
                  << " | 操作码:" << static_cast<int>(op_code)
                  << " | 结果:" << (success ? "✅ 成功" : "❌ 失败") << std::endl;
    }
    
    // 重写参数读写结果事件处理
    void on_motor_parameter_result(const std::string& interface,
                                  uint32_t motor_id,
                                  uint16_t address,
                                  uint8_t data_type,
                                  const std::any& /*data*/) override {
        std::cout << "📊 参数操作结果: " << interface << ":" << motor_id
                  << " | 地址:0x" << std::hex << address << std::dec
                  << " | 类型:" << static_cast<int>(data_type) 
                  << std::endl;
    }
};

int main() {
    std::cout << "=== 电机观察者示例程序 ===" << std::endl;
    
    // 配置电机接口和ID映射
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1}}    // can0 接口上的电机 1 和 9
    };
    
    // 使用工厂函数创建CANFD电机驱动
    auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
    
    try {
        // 创建观察者对象
        auto status_printer = std::make_shared<MotorStatusPrinter>();
        
        // 使用观察者构造函数创建机器人硬件接口
        auto robot = std::make_unique<RobotHardware>(motor_driver, motor_config, status_printer);
        
        std::cout << "硬件初始化完成，开始监控电机状态..." << std::endl;
        
        // 测试电机
        uint32_t test_motor1 = motor_config["can0"][0];  // 电机1
        // uint32_t test_motor2 = motor_config["can0"][1];  // 电机9
        // uint32_t test_motor3 = motor_config["can0"][2];
        // uint32_t test_motor4 = motor_config["can0"][3];
        // uint32_t test_motor5 = motor_config["can0"][4];
        // uint32_t test_motor6 = motor_config["can0"][5];
        
        std::cout << "\n=== 步骤1: 速度控制测试 ===" << std::endl;
        std::cout << "按Enter开始速度控制测试...";
        // robot->pause_status_monitoring();  // 停止状态输出
        // std::cin.get();
        // robot->resume_status_monitoring(); // 恢复状态输出
        
        std::cout << "速度控制：电机" << test_motor1 << " -> 5 度/秒..." << std::endl;
        robot->control_motor_in_velocity_mode("can0", test_motor1, 5.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor2, 5.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor3, 5.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor4, 5.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor5, 5.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor6, 5.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n=== 步骤2: 停止并失能电机 ===" << std::endl;
        std::cout << "按Enter停止并失能电机...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "停止并失能电机..." << std::endl;
        // robot->control_motor_in_velocity_mode("can0", test_motor1, 0.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor2, 0.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor3, 0.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor4, 0.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor5, 0.0);
        // robot->control_motor_in_velocity_mode("can0", test_motor6, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        robot->disable_motor("can0", test_motor1, 4);
        // robot->disable_motor("can0", test_motor2);
        // robot->disable_motor("can0", test_motor3);
        // robot->disable_motor("can0", test_motor4);
        // robot->disable_motor("can0", test_motor5);
        // robot->disable_motor("can0", test_motor6);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n=== 步骤3: 位置控制测试 ===" << std::endl;
        std::cout << "按Enter开始位置控制测试...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "位置控制：转到 90 度..." << std::endl;
        robot->control_motor_in_position_mode("can0", test_motor1, 90.0);
        // robot->control_motor_in_position_mode("can0", test_motor2, 90.0);
        // robot->control_motor_in_position_mode("can0", test_motor3, 90.0);
        // robot->control_motor_in_position_mode("can0", test_motor4, 90.0);
        // robot->control_motor_in_position_mode("can0", test_motor5, 90.0);
        // robot->control_motor_in_position_mode("can0", test_motor6, 90.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        std::cout << "\n=== 步骤4: 回零位置 ===" << std::endl;
        std::cout << "按Enter开始回零...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        robot->control_motor_in_position_mode("can0", test_motor1, 0.0);
        // robot->control_motor_in_position_mode("can0", test_motor2, 0.0);
        // robot->control_motor_in_position_mode("can0", test_motor3, 0.0);
        // robot->control_motor_in_position_mode("can0", test_motor4, 0.0);
        // robot->control_motor_in_position_mode("can0", test_motor5, 0.0);
        // robot->control_motor_in_position_mode("can0", test_motor6, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        std::cout << "\n=== 步骤5: 失能电机 ===" << std::endl;
        std::cout << "按Enter失能电机...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "失能电机..." << std::endl;
        robot->disable_motor("can0", test_motor1, 4);
        // robot->disable_motor("can0", test_motor2);
        // robot->disable_motor("can0", test_motor3);
        // robot->disable_motor("can0", test_motor4);
        // robot->disable_motor("can0", test_motor5);
        // robot->disable_motor("can0", test_motor6);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n观察者测试完成！" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
