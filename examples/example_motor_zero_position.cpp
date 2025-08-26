#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <chrono>
#include <thread>

// 零位设置状态观察者
class ZeroPositionObserver : public hardware_driver::motor_driver::MotorStatusObserver{
public:
    // 单个电机零位设置状态回调处理器
    void on_motor_status_update(const std::string& interface, uint32_t motor_id, 
                                          const hardware_driver::motor_driver::Motor_Status& status) {
        std::cout << "[零位监控] " << interface << ":" << motor_id
                  << " | 位置:" << status.position 
                  << "° | 速度:" << status.velocity
                  << "°/s | 力矩:" << status.effort 
                  << "Nm | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                  << " | 使能:" << static_cast<int>(status.enable_flag)
                  << " | 模式:" << static_cast<int>(status.motor_mode)
                  << std::endl;
    }
    
    // 批量电机零位设置状态回调处理器
    void on_motor_status_update(const std::string& interface, 
                                         const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) {
        std::cout << "[批量零位监控] " << interface << " - " << status_all.size() << "个电机状态" << std::endl;
        
        size_t near_zero_count = 0;
        for (const auto& [motor_id, status] : status_all) {
            std::cout << "  电机" << motor_id << ": " << status.position << "° | " 
                      << status.velocity << "°/s | 力矩:" << status.effort << "Nm";
            
            if (std::abs(status.position) < 0.5 && std::abs(status.velocity) < 1.0) {
                std::cout << " [接近零位]";
                near_zero_count++;
            }
            std::cout << std::endl;
        }
        
        if (near_zero_count == status_all.size()) {
            std::cout << "  ✅ 所有电机都已接近零位!" << std::endl;
        }
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
};

int main() {
    std::cout << "=== 电机零位设置示例程序 ===" << std::endl;
    
    // 配置电机接口和ID映射
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}}    // can0 接口上的电机
    };
    
    // 使用工厂函数创建CANFD电机驱动
    auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
    
    try {
        // 创建零位设置观察者对象
        auto zero_observer = std::make_shared<ZeroPositionObserver>();
        
        // 使用观察者构造函数创建机器人硬件接口
        auto robot = std::make_unique<RobotHardware>(motor_driver, motor_config, zero_observer);

        std::cout << "硬件初始化完成，准备设置零位..." << std::endl;
        
        std::vector<uint32_t> arm_motors = motor_config["can0"];  // 配置的电机
        
        std::cout << "\n步骤1: 失能所有电机" << std::endl;
        std::cout << "按Enter开始失能全部电机...";
        robot->pause_status_monitoring();  // 停止状态输出
        std::cin.get();
        robot->resume_status_monitoring(); // 恢复状态输出
        for (uint32_t motor_id : arm_motors) {
            robot->disable_motor("can0", motor_id);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\n步骤2: 设置零位" << std::endl;
        std::cout << "警告：请确保机器人在安全位置！" << std::endl;
        std::cout << "按Enter开始设置全部电机零位...";
        robot->pause_status_monitoring();  // 停止状态输出
        std::cin.get();
        robot->resume_status_monitoring(); // 恢复状态输出
        robot->arm_zero_position_set("can0", arm_motors);
        std::cout << "零位设置命令已发送，请等待完成..." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n步骤3: 验证零位设置" << std::endl;
        std::cout << "按Enter开始小幅位置控制测试...";
        robot->pause_status_monitoring();  // 停止状态输出
        std::cin.get();
        robot->resume_status_monitoring(); // 恢复状态输出
        
        // 小幅运动测试
        for (uint32_t motor_id : arm_motors) {
            robot->control_motor_in_position_mode("can0", motor_id, 10.0);  // 移动10度
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 回零测试
        std::cout << "\n步骤4: 回零测试" << std::endl;
        std::cout << "按Enter开始回零...";
        robot->pause_status_monitoring();  // 停止状态输出
        std::cin.get();
        robot->resume_status_monitoring(); // 恢复状态输出

        for (uint32_t motor_id : arm_motors) {
            robot->control_motor_in_position_mode("can0", motor_id, 0.0);   // 回到零位
        }
        std::cout << "观察电机是否成功回到零位..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n步骤5: 失能电机" << std::endl;
        for (uint32_t motor_id : arm_motors) {
            robot->disable_motor("can0", motor_id);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "\n零位设置示例完成！" << std::endl;
        std::cout << "提示：检查所有电机是否正确回到零位，验证设置是否成功。" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "请检查：" << std::endl;
        std::cerr << "1. 所有电机是否连接并上电" << std::endl;
        std::cerr << "2. 机器人是否在安全位置" << std::endl;
        std::cerr << "3. 电机ID配置是否正确" << std::endl;
        return 1;
    }
    
    return 0;
}