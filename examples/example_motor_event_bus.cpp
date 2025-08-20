#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

using namespace hardware_driver::event;

// 事件总线集成的电机状态监控器
class EventBasedMotorMonitor {
public:
    std::atomic<int> motor_status_count{0};
    std::atomic<int> batch_status_count{0};
    
    EventBasedMotorMonitor(std::shared_ptr<EventBus> bus) : bus_(bus) {
        // 使用简化的订阅API监控电机状态
        motor_handler_ = bus_->subscribe<MotorStatusEvent>(
            [this](const auto& event) { 
                motor_status_count++;
                auto& status = event->get_status();
                std::cout << "[状态] 电机 " << event->get_interface() << ":" << event->get_motor_id() 
                  << " | 位置:" << status.position 
                  << " | 速度:" << status.velocity
                  << " | 力矩:" << status.effort 
                  << " | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                  << " | 使能:" << static_cast<int>(status.enable_flag)
                  << " | 模式:" << static_cast<int>(status.motor_mode)
                  << std::endl;
            });
            
        batch_handler_ = bus_->subscribe<MotorBatchStatusEvent>(
            [this](const auto& event) { 
                batch_status_count++;
                std::cout << "[Batch] 接口 " << event->get_interface()
                         << " 收到 " << event->get_status_all().size() << " 个电机状态:" << std::endl;
                
                // 遍历所有电机状态并打印详细信息
                for (const auto& [motor_id, status] : event->get_status_all()) {
                    std::cout << "  [电机" << motor_id << "] "
                              << "位置:" << status.position 
                              << " | 速度:" << status.velocity
                              << " | 力矩:" << status.effort 
                              << " | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                              << " | 使能:" << static_cast<int>(status.enable_flag)
                              << " | 模式:" << static_cast<int>(status.motor_mode)
                              << std::endl;
                }
            });
    }
    
    ~EventBasedMotorMonitor() {
        if (motor_handler_) {
            bus_->unsubscribe<MotorStatusEvent>(motor_handler_);
        }
        if (batch_handler_) {
            bus_->unsubscribe<MotorBatchStatusEvent>(batch_handler_);
        }
    }
    
private:
    std::shared_ptr<EventBus> bus_;
    std::shared_ptr<EventHandler> motor_handler_;
    std::shared_ptr<EventHandler> batch_handler_;
};

int main() {
    std::cout << "=== 事件总线集成硬件驱动示例 ===" << std::endl;
    
    // 创建事件总线
    auto bus = std::make_shared<EventBus>();
    
    // 创建事件监控器
    auto monitor = std::make_unique<EventBasedMotorMonitor>(bus);
    
    // 配置CAN接口和电机
    std::vector<std::string> interfaces = {"can0"};
    std::map<std::string, std::vector<uint32_t>> motor_config = {
        {"can0", {1, 9}},
    };
    
    try {
        std::cout << "=== 电机测试程序 ===" << std::endl;
        // 创建CAN总线
        auto can_bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        
        // 创建电机驱动
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(can_bus);

        // 定义单个电机状态回调函数 (use this)
        // auto single_callback = [bus](const std::string& interface, uint32_t motor_id, 
        //                             const hardware_driver::motor_driver::Motor_Status& status) {
        //     bus->emit<MotorStatusEvent>(interface, motor_id, status);
        // };
        // RobotHardware robot_hardware(motor_driver, motor_config, single_callback);

        // 定义批量电机状态回调函数 (or this)
        auto batch_status_callback = [bus](const std::string& interface, 
                                          const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) {
            // 发布批量电机状态事件
            bus->emit<MotorBatchStatusEvent>(interface, status_all);
        };
        
        // 创建硬件接口 - 使用批量状态回调
        RobotHardware robot_hardware(motor_driver, motor_config, batch_status_callback);
        
        std::cout << "硬件驱动和事件总线初始化成功！" << std::endl;
        
        // 测试电机1 - 从配置中获取电机ID
        uint32_t test_motor1 = motor_config["can0"][0];
        uint32_t test_motor2 = motor_config["can0"][1];
        
        std::cout << "\n步骤1: 使能电机 " << test_motor1 << test_motor2 << std::endl;
        // robot_hardware.enable_motor("can0", test_motor, 4);  // 模式4
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n步骤2: 控制电机运动" << std::endl;
        
        // 正向旋转
        std::cout << "正向旋转 5 度/秒..." << std::endl;
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor1, 5.0);
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor2, 5.0);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 停止
        std::cout << "停止..." << std::endl;
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor1, 0.0);
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 反向旋转
        std::cout << "反向旋转 -3 度/秒..." << std::endl;
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor1, -3.0);
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor2, -3.0);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 停止
        std::cout << "停止..." << std::endl;
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor1, 0.0);
        robot_hardware.control_motor_in_velocity_mode("can0", test_motor2, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::cout << "\n步骤3: 失能电机" << std::endl;
        robot_hardware.disable_motor("can0", test_motor1);
        robot_hardware.disable_motor("can0", test_motor2);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 输出事件统计信息
        auto stats = bus->get_statistics();
        std::cout << "\n=== 事件总线统计信息 ===" << std::endl;
        std::cout << "总处理器数量: " << stats.total_handlers << std::endl;
        std::cout << "已发布事件数: " << stats.events_published << std::endl;
        std::cout << "电机状态事件: " << monitor->motor_status_count.load() << std::endl;
        std::cout << "批量状态事件: " << monitor->batch_status_count.load() << std::endl;
        
        std::cout << "\n示例完成！事件总线成功集成硬件驱动。" << std::endl;
        
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