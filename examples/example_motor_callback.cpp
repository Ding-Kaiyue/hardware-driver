#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <chrono>
#include <thread>

// 电机状态回调处理器
class MotorStatusHandler {
public:
    // 单个电机状态回调处理器
    static void handle_single_motor_status(const std::string& interface, uint32_t motor_id, 
                                          const hardware_driver::motor_driver::Motor_Status& status) {
        std::cout << "[实时状态] " << interface << ":" << motor_id
                  << " | 位置:" << status.position 
                  << "° | 速度:" << status.velocity
                  << "°/s | 力矩:" << status.effort 
                  << "Nm | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                  << " | 使能:" << static_cast<int>(status.enable_flag)
                  << " | 模式:" << static_cast<int>(status.motor_mode)
                  << std::endl;
    }
    
    // 批量电机状态回调处理器
    static void handle_batch_motor_status(const std::string& interface, 
                                         const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) {
        std::cout << "[批量状态] " << interface << " - " << status_all.size() << "个电机" << std::endl;
        for (const auto& [motor_id, status] : status_all) {
            std::cout << "  电机" << motor_id << ": " << status.position << "° | " << status.velocity << "°/s" << std::endl;
        }
    }
};

int main() {
    std::cout << "=== 电机回调模式示例程序 ===" << std::endl;
    
    // 配置电机接口和ID映射
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}}    // can0 接口上的电机 1 和 9
    };
    
    try {
        // 使用工厂函数创建CANFD电机驱动
        auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
        
        // 选择回调模式
        std::cout << "选择回调模式:" << std::endl;
        std::cout << "1. 单个状态回调" << std::endl;
        std::cout << "2. 批量状态回调" << std::endl;
        std::cout << "请输入 (1 or 2): ";
        
        int choice;
        std::cin >> choice;
        
        std::unique_ptr<RobotHardware> robot;
        
        if (choice == 1) {
            std::cout << "使用单个状态回调模式..." << std::endl;
            // 使用单个状态回调构造函数
            robot = std::make_unique<RobotHardware>(motor_driver, motor_config, 
                                                   MotorStatusHandler::handle_single_motor_status);
        } else if (choice == 2) {
            std::cout << "使用批量状态回调模式..." << std::endl;
            // 使用批量状态回调构造函数
            robot = std::make_unique<RobotHardware>(motor_driver, motor_config, 
                                                   MotorStatusHandler::handle_batch_motor_status);
        } else {
            std::cout << "无效选择" << std::endl;
            return 1;
        }
        
        std::cout << "硬件初始化完成，开始电机测试..." << std::endl;
        
        // 实际的电机控制流程
        std::cout << "\n步骤1: 使能电机 1 和 9" << std::endl;
        robot->enable_motor("can0", 1, 4);  // 模式4
        robot->enable_motor("can0", 9, 4);  // 模式4
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n步骤2: 速度控制测试" << std::endl;
        std::cout << "按Enter开始速度控制测试, 电机1: 3°/s, 电机9: -3°/s";
        robot->pause_status_monitoring();  // 停止状态输出
        std::cin.get();
        robot->resume_status_monitoring(); // 恢复状态输出

        robot->control_motor_in_velocity_mode("can0", motor_config["can0"][0], 3.0);
        robot->control_motor_in_velocity_mode("can0", motor_config["can0"][1], -3.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "\n步骤3: 停止并失能电机" << std::endl;
        std::cout << "按Enter停止并失能电机...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();

        robot->control_motor_in_velocity_mode("can0", motor_config["can0"][0], 0.0);
        robot->control_motor_in_velocity_mode("can0", motor_config["can0"][1], 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        robot->disable_motor("can0", motor_config["can0"][0], 4);
        robot->disable_motor("can0", motor_config["can0"][1], 4);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\n步骤4: 位置控制测试，电机1: 90°, 电机9: -45°" << std::endl;
        std::cout << "按Enter开始位置控制测试...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_position_mode("can0", motor_config["can0"][0], 90.0);
        robot->control_motor_in_position_mode("can0", motor_config["can0"][1], -45.0);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        std::cout << "\n步骤5: 回零" << std::endl;
        std::cout << "按Enter开始回零...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_position_mode("can0", motor_config["can0"][0], 0.0);
        robot->control_motor_in_position_mode("can0", motor_config["can0"][1], 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n步骤6: 失能电机" << std::endl;
        std::cout << "按Enter失能电机...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->disable_motor("can0", motor_config["can0"][0], 4);
        robot->disable_motor("can0", motor_config["can0"][1], 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n✅ 电机控制回调模式测试完成!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "请检查:" << std::endl;
        std::cerr << "1. 电机1和9是否正确连接到can0" << std::endl;
        std::cerr << "2. 电机是否已上电" << std::endl;
        std::cerr << "3. CAN总线是否正常工作" << std::endl;
        return 1;
    }
    
    return 0;
}