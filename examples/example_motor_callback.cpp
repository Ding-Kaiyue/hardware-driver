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
                  << " | 错误:" << static_cast<u_int32_t>(status.error_code)
                  << std::endl;
    }
    
    // 批量电机状态回调处理器
    static void handle_batch_motor_status(const std::string& interface, 
                                         const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) {
        std::cout << "[批量状态] " << interface << " - " << status_all.size() << "个电机" << std::endl;
        for (const auto& [motor_id, status] : status_all) {
            std::cout << "  电机" << motor_id << ": " << status.position << "° | " << status.velocity << "°/s" << " | Err " << status.error_code <<std::endl;
        }
    }
};

int main() {
    std::cout << "=== 电机回调模式示例程序 ===" << std::endl;

    // 配置电机接口和ID映射
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 2, 3, 4, 5, 6}}    // can0 接口上的电机 1-6
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

        // 获取电机ID
        uint32_t motor1 = motor_config["can0"][0];   // 电机1 - 速度模式
        uint32_t motor2 = motor_config["can0"][1];   // 电机2 - 速度模式
        uint32_t motor3 = motor_config["can0"][2];   // 电机3 - 位置模式
        uint32_t motor4 = motor_config["can0"][3];   // 电机4 - 位置模式
        uint32_t motor5 = motor_config["can0"][4];   // 电机5 - MIT模式
        uint32_t motor6 = motor_config["can0"][5];   // 电机6 - MIT模式

        std::cout << "测试电机: " << motor1 << " " << motor2 << " " << motor3 << " "
                  << motor4 << " " << motor5 << " " << motor6 << std::endl;

        // ========== 先让所有电机都失能 ==========
        robot->disable_motors("can0", motor_config["can0"], 4);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // ========== 电机1和2：速度模式控制 ==========
        std::cout << "\n=== 电机1和2：速度模式控制 ===" << std::endl;

        uint8_t velocity_mode = 4; // 速度模式
        std::cout << "按Enter使能电机1和2的速度模式...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->enable_motors("can0", {motor1, motor2}, velocity_mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "按Enter执行电机1和2正转6度/秒...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_velocity_mode("can0", motor1, 6.0);
        robot->control_motor_in_velocity_mode("can0", motor2, 6.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "按Enter停止电机1和2...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_velocity_mode("can0", motor1, 0.0);
        robot->control_motor_in_velocity_mode("can0", motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "按Enter执行电机1和2反转6度/秒...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_velocity_mode("can0", motor1, -6.0);
        robot->control_motor_in_velocity_mode("can0", motor2, -6.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "按Enter停止并失能电机1和2...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_velocity_mode("can0", motor1, 0.0);
        robot->control_motor_in_velocity_mode("can0", motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        robot->disable_motor("can0", motor1, velocity_mode);
        robot->disable_motor("can0", motor2, velocity_mode);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // ========== 电机3和4：位置模式控制 ==========
        std::cout << "\n=== 电机3和4：位置模式控制 ===" << std::endl;

        uint8_t position_mode = 5; // 位置模式
        std::cout << "按Enter使能电机3和4的位置模式...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->enable_motors("can0", {motor3, motor4}, position_mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "按Enter执行电机3和4运动到30和-30度...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_position_mode("can0", motor3, 30.0);
        robot->control_motor_in_position_mode("can0", motor4, -30.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::cout << "按Enter执行电机3和4运动到-30和30度...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_position_mode("can0", motor3, -30.0);
        robot->control_motor_in_position_mode("can0", motor4, 30.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::cout << "按Enter执行电机3和4回到零位置...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->control_motor_in_position_mode("can0", motor3, 0.0);
        robot->control_motor_in_position_mode("can0", motor4, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::cout << "按Enter失能电机3和4...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        robot->disable_motor("can0", motor3, position_mode);
        robot->disable_motor("can0", motor4, position_mode);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // ========== 电机5和6：MIT模式控制 ==========
        // std::cout << "\n=== 电机5和6：MIT模式控制 ===" << std::endl;

        // uint8_t mit_mode = 3; // MIT模式
        // std::cout << "按Enter使能电机5和6的MIT模式...";
        // robot->pause_status_monitoring();
        // std::cin.get();
        // robot->resume_status_monitoring();
        // robot->enable_motors("can0", {motor5, motor6}, mit_mode);
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // std::cout << "按Enter执行电机5和6运动到90和-90度，速度10和-10度/秒...";
        // robot->pause_status_monitoring();
        // std::cin.get();
        // robot->resume_status_monitoring();
        // robot->control_motor_in_mit_mode("can0", motor5, 90.0, 10.0, 0.0);
        // robot->control_motor_in_mit_mode("can0", motor6, -90.0, -10.0, 0.0);
        // std::this_thread::sleep_for(std::chrono::seconds(3));

        // std::cout << "按Enter停止电机5和6...";
        // robot->pause_status_monitoring();
        // std::cin.get();
        // robot->resume_status_monitoring();
        // robot->control_motor_in_mit_mode("can0", motor5, 90.0, 0.0, 0.0);
        // robot->control_motor_in_mit_mode("can0", motor6, -90.0, 0.0, 0.0);
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // std::cout << "按Enter失能电机5和6...";
        // robot->pause_status_monitoring();
        // std::cin.get();
        // robot->resume_status_monitoring();
        // robot->disable_motor("can0", motor5, mit_mode);
        // robot->disable_motor("can0", motor6, mit_mode);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // std::cout << "\n✅ 回调模式电机控制测试完成!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "请检查：" << std::endl;
        std::cerr << "1. 电机1-6是否连接到can0总线" << std::endl;
        std::cerr << "2. 电机ID是否为1到6（可修改motor_config中的ID）" << std::endl;
        std::cerr << "3. 电机是否已上电" << std::endl;
        std::cerr << "4. CAN总线是否正常工作" << std::endl;
        return 1;
    }
    
    return 0;
}