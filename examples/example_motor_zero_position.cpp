#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <limits>

// 电机状态观察者 - 监控零位设置过程
class ZeroPositionObserver : public hardware_driver::motor_driver::MotorStatusObserver {
private:
    bool zero_position_set_ = false;
    
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
    
    // 函数操作结果监控
    void on_motor_function_result(const std::string& interface,
                                 uint32_t motor_id,
                                 uint8_t op_code,
                                 bool success) override {
        std::cout << "[函数操作] 电机 " << interface << ":" << motor_id 
                  << " | 操作码:" << static_cast<int>(op_code)
                  << " | 结果:" << (success ? "成功" : "失败") << std::endl;
                  
        // 操作码4是零位设置，操作码2是保存设置
        if ((op_code == 4 || op_code == 2) && success) {
            zero_position_set_ = true;
            std::cout << "✅ 零位设置成功！操作码:" << static_cast<int>(op_code) << std::endl;
        }
    }
    
    bool is_zero_position_set() const { return zero_position_set_; }
    void reset_flag() { zero_position_set_ = false; }
};

int main() {
    std::cout << "=== 电机零位设置示例 ===" << std::endl;
    
    // 配置硬件
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}}  // 可以设置多个电机: {1, 2, 3}
    };
    
    try {
        // 创建CAN总线
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        
        // 创建电机驱动
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
        
        // 添加状态观察者
        auto observer = std::make_shared<ZeroPositionObserver>();
        motor_driver->add_observer(observer);
        
        // 创建硬件接口 - 和观察者示例保持一致
        RobotHardware robot(motor_driver, motor_config);
        
        std::cout << "硬件初始化完成" << std::endl;
        
        // 从配置中获取电机ID列表
        const auto& motor_ids = motor_config["can0"];
        uint32_t test_motor1 = motor_ids[0];
        uint32_t test_motor2 = motor_ids[1];
        
        std::cout << "\n=== 零位设置流程 ===" << std::endl;
        
        // 步骤1: 失能电机
        std::cout << "\n步骤1: 失能电机" << std::endl;
        robot.disable_motor("can0", test_motor1);
        robot.disable_motor("can0", test_motor2);
        std::cout << "等待电机失能..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "请确认电机已失能(使能标志应为0)，然后按回车继续..." << std::endl;
        
        // 临时移除观察者以避免输入干扰
        motor_driver->remove_observer(observer);
        std::cin.get();
        // 恢复观察者
        motor_driver->add_observer(observer);

        // 步骤2: 查看当前位置
        std::cout << "\n步骤2: 查看电机当前状态 (2秒)" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 步骤3: 设置零位
        std::cout << "\n步骤3: 设置电机零位" << std::endl;
        std::cout << "等待电机完全静止（速度接近0）..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));  // 等待更长时间确保静止
        
        observer->reset_flag();
        
        // 使用arm_zero_position_set设置零位
        robot.arm_zero_position_set("can0", motor_ids);
        
        // 等待零位设置完成
        std::cout << "等待零位设置完成..." << std::endl;
        int timeout_count = 0;
        while (!observer->is_zero_position_set() && timeout_count < 10) {  // 1秒超时
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timeout_count++;
        }
        
        if (observer->is_zero_position_set()) {
            std::cout << "✅ 零位设置操作已完成！" << std::endl;
        } else {
            std::cout << "⚠️  零位设置超时，请检查电机连接" << std::endl;
        }
        
        // 步骤4: 测试零位后的运动
        std::cout << "\n步骤4: 测试零位后的位置控制" << std::endl;
        std::cout << "是否进行位置控制测试? (y/n): " << std::flush;
        
        // 临时移除观察者以避免输入干扰
        motor_driver->remove_observer(observer);
        char choice;
        std::cin >> choice;
        // 恢复观察者
        motor_driver->add_observer(observer);
        
        if (choice == 'y' || choice == 'Y') {
            // 使能电机
            std::cout << "使能电机..." << std::endl;
            // robot.enable_motor("can0", test_motor, 4);  // 位置模式
            // std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 移动到不同位置测试
            std::cout << "移动到位置 30 度..." << std::endl;
            robot.control_motor_in_position_mode("can0", test_motor1, 30.0);
            robot.control_motor_in_position_mode("can0", test_motor2, 30.0);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            std::cout << "移动到位置 -30 度..." << std::endl;
            robot.control_motor_in_position_mode("can0", test_motor1, -30.0);
            robot.control_motor_in_position_mode("can0", test_motor2, -30.0);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            std::cout << "回到零位 (0 度)..." << std::endl;
            robot.control_motor_in_position_mode("can0", test_motor1, 0.0);
            robot.control_motor_in_position_mode("can0", test_motor2, 0.0);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 失能电机
            std::cout << "\n失能电机" << std::endl;
            robot.disable_motor("can0", test_motor1);
            robot.disable_motor("can0", test_motor2);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            std::cout << "跳过位置控制测试" << std::endl;
        }
        
        std::cout << "\n✅ 零位设置示例完成！" << std::endl;
        std::cout << "\n说明：" << std::endl;
        std::cout << "- 零位设置后，位置读数应该以设置时的位置为0度基准" << std::endl;
        std::cout << "- 如需重新设置零位，可以再次运行此程序" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "请检查：" << std::endl;
        std::cerr << "1. 电机是否连接到can0总线" << std::endl;
        std::cerr << "2. 电机ID是否正确（可修改motor_config中的ID）" << std::endl;
        std::cerr << "3. 电机是否已上电" << std::endl;
        std::cerr << "4. 电机是否支持零位设置功能" << std::endl;
        return 1;
    }
    
    return 0;
}