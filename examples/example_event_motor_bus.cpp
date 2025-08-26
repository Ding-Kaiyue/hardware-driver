#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <map>

// 事件处理器类 - 展示如何订阅和处理事件
class MotorEventHandler : public hardware_driver::motor_driver::MotorEventHandler {
public:
    MotorEventHandler() {
        std::cout << "MotorEventHandler created" << std::endl;
    }
    
    ~MotorEventHandler() {
        std::cout << "MotorEventHandler destroyed" << std::endl;
    }
    
    // 处理电机状态事件 - 通过事件总线接收
    void on_motor_status_update(const std::string& interface, uint32_t motor_id, 
                            const hardware_driver::motor_driver::Motor_Status& status) override {
        std::cout << "[状态事件] 电机 " << interface << ":" << motor_id
                  << " | 位置:" << status.position 
                  << " | 速度:" << status.velocity
                  << " | 力矩:" << status.effort 
                  << " | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                  << " | 使能:" << static_cast<int>(status.enable_flag)
                  << " | 模式:" << static_cast<int>(status.motor_mode)
                  << std::endl;
        
        // 事件总线特有的状态分析
        if (status.enable_flag) {
            if (std::abs(status.velocity) > 50.0) {
                std::cout << "  ⚠️  高速运动警告: 电机" << motor_id << " 速度 " << status.velocity << "°/s" << std::endl;
            }
            if (status.temperature > 600) { // 60°C
                std::cout << "  🔥 温度警告: 电机" << motor_id << " 温度 " << status.temperature/10.0 << "°C" << std::endl;
            }
        }
    }
    
    // 处理批量电机状态事件
    void on_motor_status_update(const std::string& interface, 
                            const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) override {
        std::cout << "批量状态事件: " << interface 
                  << " | 电机数量:" << status_all.size() << std::endl;
        
        size_t enabled_count = 0;
        double max_temp = 0.0;
        double max_velocity = 0.0;
        
        for (const auto& [motor_id, status] : status_all) {
            std::cout << "  └─ 电机" << motor_id << ": 位置" << status.position << "° | 速度" << status.velocity << "°/s";
            
            if (status.enable_flag) {
                enabled_count++;
                std::cout << " [已使能]";
            }
            std::cout << std::endl;
            
            // 统计信息
            double temp = static_cast<double>(status.temperature) / 10.0;
            double vel = std::abs(static_cast<double>(status.velocity));
            if (temp > max_temp) max_temp = temp;
            if (vel > max_velocity) max_velocity = vel;
        }
        
        // 事件总线统计分析
        std::cout << "  📊 系统状态: " << enabled_count << "/" << status_all.size() 
                  << "个电机使能 | 最高温度:" << max_temp << "°C | 最高速度:" << max_velocity << "°/s" << std::endl;
        
        if (enabled_count == status_all.size() && max_velocity < 1.0) {
            std::cout << "  ✅ 系统稳定: 所有电机已使能且静止" << std::endl;
        }
    }
    
    // 处理函数操作结果事件
    void on_motor_function_result(const std::string& interface, uint32_t motor_id, 
                               uint8_t op_code, bool success) override {
        std::cout << "⚡ 函数操作结果: " << interface << ":" << motor_id
                  << " | 操作码:" << static_cast<int>(op_code)
                  << " | 结果:" << (success ? "✅ 成功" : "❌ 失败") << std::endl;
    }
    
    // 处理参数操作结果事件
    void on_motor_parameter_result(const std::string& interface, uint32_t motor_id,
                                uint16_t address, uint8_t data_type, const std::any& data) override {
        std::cout << "🔧 参数操作结果: " << interface << ":" << motor_id
                  << " | 地址:0x" << std::hex << address << std::dec
                  << " | 数据类型:" << static_cast<int>(data_type);
        
        // 根据数据类型显示数值
        if (data_type == 0x01) {  // int
            try {
                std::cout << " | 数值:" << std::any_cast<int32_t>(data);
            } catch (const std::bad_any_cast&) {
                std::cout << " | 数值:类型转换失败";
            }
        } else if (data_type == 0x02) {  // float
            try {
                std::cout << " | 数值:" << std::any_cast<float>(data);
            } catch (const std::bad_any_cast&) {
                std::cout << " | 数值:类型转换失败";
            }
        } else {
            std::cout << " | 数值:未知类型";
        }
        std::cout << std::endl;
    }
};

int main() {
    std::cout << "=== 电机测试程序（事件总线版本） ===" << std::endl;
    
    try {
        // 1. 配置硬件
        std::vector<std::string> interfaces = {"can0"};
        std::map<std::string, std::vector<uint32_t>> motor_config = {
            {"can0", {1, 9}}
        };
        
        std::cout << "事件总线模式特点：" << std::endl;
        std::cout << "- 支持多种事件类型订阅和处理" << std::endl;
        std::cout << "- 解耦事件生产者和消费者" << std::endl;
        std::cout << "- 支持异步事件处理和状态监控" << std::endl;
        std::cout << "- 提供实时状态分析和警告机制" << std::endl;
        
        // 2. 创建事件总线
        auto event_bus = std::make_shared<hardware_driver::event::EventBus>();
        std::cout << "✅ 事件总线已创建" << std::endl;
        
        // 3. 创建事件处理器
        auto event_handler = std::make_shared<MotorEventHandler>();
        
        // 4. 创建硬件组件
        // 使用工厂函数创建CANFD电机驱动
        auto motor_driver = hardware_driver::createCanFdMotorDriver({"can0"});
        
        // 使用事件总线构造函数创建机器人硬件接口（与观察者模式保持相同结构）
        auto robot = std::make_unique<RobotHardware>(motor_driver, motor_config, event_bus, event_handler);
        
        std::cout << "✅ 事件总线电机系统初始化完成" << std::endl;
        
        // 4. 执行和观察者示例相同的测试流程，观察事件总线的工作
        // 测试电机1和9 - 从配置中获取电机ID
        uint32_t test_motor1 = motor_config["can0"][0];
        uint32_t test_motor2 = motor_config["can0"][1];
        
        std::cout << "测试电机: " << test_motor1 << " 和 " << test_motor2 << std::endl;
        
        std::cout << "\n=== 步骤1: 高速运动测试 ===" << std::endl;
        std::cout << "按Enter开始高速运动测试 (将触发警告事件)...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "正向旋转 6 度/秒 (将触发高速警告事件)..." << std::endl;
        robot->control_motor_in_velocity_mode("can0", test_motor1, 6.0);
        robot->control_motor_in_velocity_mode("can0", test_motor2, 6.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "\n=== 步骤2: 停止电机 ===" << std::endl;
        std::cout << "按Enter停止电机...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "停止电机..." << std::endl;
        robot->control_motor_in_velocity_mode("can0", test_motor1, 0.0);
        robot->control_motor_in_velocity_mode("can0", test_motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n=== 步骤3: 正常速度运动 ===" << std::endl;
        std::cout << "按Enter开始正常速度运动...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "反向旋转 -30 度/秒..." << std::endl;
        robot->control_motor_in_velocity_mode("can0", test_motor1, -3.0);
        robot->control_motor_in_velocity_mode("can0", test_motor2, -3.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n=== 步骤4: 停止并失能电机 ===" << std::endl;
        std::cout << "按Enter停止并失能电机...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();

        robot->control_motor_in_velocity_mode("can0", test_motor1, 0.0);
        robot->control_motor_in_velocity_mode("can0", test_motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        robot->disable_motor("can0", test_motor1);
        robot->disable_motor("can0", test_motor2);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "\n=== 步骤5: 位置控制测试 ===" << std::endl;
        std::cout << "按Enter开始位置控制...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "位置控制到10度..." << std::endl;
        robot->control_motor_in_position_mode("can0", test_motor1, 10.0);
        robot->control_motor_in_position_mode("can0", test_motor2, -10.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "\n=== 步骤6: 回零位置 ===" << std::endl;
        std::cout << "按Enter开始回零...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "回零位置..." << std::endl;
        robot->control_motor_in_position_mode("can0", test_motor1, 0.0);
        robot->control_motor_in_position_mode("can0", test_motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        std::cout << "\n=== 步骤6: 失能电机 ===" << std::endl;
        std::cout << "按Enter失能电机...";
        robot->pause_status_monitoring();
        std::cin.get();
        robot->resume_status_monitoring();
        
        std::cout << "失能电机..." << std::endl;
        robot->disable_motor("can0", test_motor1);
        robot->disable_motor("can0", test_motor2);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n✅ 事件总线电机控制测试完成!" << std::endl;
        std::cout << "事件总线成功提供了:" << std::endl;
        std::cout << "- 实时状态事件监控和处理" << std::endl;
        std::cout << "- 解耦的事件生产者和消费者架构" << std::endl;
        std::cout << "- 智能警告和异常检测事件" << std::endl;
        std::cout << "- 系统级状态统计分析" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "请检查：" << std::endl;
        std::cerr << "1. 电机是否连接到can0总线" << std::endl;
        std::cerr << "2. 电机ID是否为1和9（可修改motor_config中的ID）" << std::endl;
        std::cerr << "3. 电机是否已上电" << std::endl;
        std::cerr << "4. 事件总线系统是否正常工作" << std::endl;
        return 1;
    }
    
    return 0;
}