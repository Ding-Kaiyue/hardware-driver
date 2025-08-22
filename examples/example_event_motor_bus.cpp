#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/bus/canfd_bus_impl.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

// 事件处理器类 - 展示如何订阅和处理事件
class MotorEventHandler {
public:
    MotorEventHandler() {
        std::cout << "MotorEventHandler created" << std::endl;
    }
    
    ~MotorEventHandler() {
        std::cout << "MotorEventHandler destroyed" << std::endl;
    }
    
    // 处理电机状态事件 - 和观察者模式保持一致的格式
    void handle_motor_status(const std::shared_ptr<hardware_driver::event::MotorStatusEvent>& event) {
        const auto& status = event->get_status();
        std::cout << "[状态] 电机 " << event->get_interface() << ":" << event->get_motor_id()
                  << " | 位置:" << status.position 
                  << " | 速度:" << status.velocity
                  << " | 力矩:" << status.effort 
                  << " | 温度:" << static_cast<int>(status.temperature) / 10.0 << "°C"
                  << " | 使能:" << static_cast<int>(status.enable_flag)
                  << " | 模式:" << static_cast<int>(status.motor_mode)
                  << std::endl;
    }
    
    // 处理批量电机状态事件
    void handle_batch_status(const std::shared_ptr<hardware_driver::event::MotorBatchStatusEvent>& event) {
        std::cout << "批量状态事件: " << event->get_interface() 
                  << " | 电机数量:" << event->get_status_all().size() << std::endl;
        
        for (const auto& [motor_id, status] : event->get_status_all()) {
            std::cout << "  └─ 电机" << motor_id << ": 位置" << status.position << "° | 速度" << status.velocity << "°/s" << std::endl;
        }
    }
    
    // 处理函数操作结果事件
    void handle_function_result(const std::shared_ptr<hardware_driver::event::MotorFunctionResultEvent>& event) {
        std::cout << "⚡ 函数操作结果: " << event->get_interface() << ":" << event->get_motor_id()
                  << " | 操作码:" << static_cast<int>(event->get_operation_code())
                  << " | 结果:" << (event->is_success() ? "✅ 成功" : "❌ 失败") << std::endl;
    }
    
    // 处理参数操作结果事件
    void handle_parameter_result(const std::shared_ptr<hardware_driver::event::MotorParameterResultEvent>& event) {
        std::cout << "🔧 参数操作结果: " << event->get_interface() << ":" << event->get_motor_id()
                  << " | 地址:0x" << std::hex << event->get_address() << std::dec
                  << " | 数据类型:" << static_cast<int>(event->get_data_type());
        
        // 根据数据类型显示数值
        const auto& data = event->get_data();
        if (event->get_data_type() == 0x01) {  // int
            try {
                std::cout << " | 数值:" << std::any_cast<int32_t>(data);
            } catch (const std::bad_any_cast&) {
                std::cout << " | 数值:类型转换失败";
            }
        } else if (event->get_data_type() == 0x02) {  // float
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
        
        // 2. 创建事件总线
        auto event_bus = std::make_shared<hardware_driver::event::EventBus>();
        std::cout << "✅ 事件总线已创建" << std::endl;
        
        // 3. 创建事件处理器
        auto event_handler = std::make_shared<MotorEventHandler>();
        
        // 4. 订阅电机状态事件
        auto status_subscription = event_bus->subscribe<hardware_driver::event::MotorStatusEvent>(
            [event_handler](const auto& event) {
                event_handler->handle_motor_status(event);
            }
        );
        
        // 5. 创建硬件组件
        // 创建CAN总线
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        
        // 创建电机驱动
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
        
        // 创建机器人硬件接口（集成事件总线）
        RobotHardware robot(motor_driver, motor_config, event_bus);
        
        std::cout << "硬件初始化完成" << std::endl;
        
        // 6. 执行和观察者示例相同的测试流程
        // 测试电机1和9 - 从配置中获取电机ID
        uint32_t test_motor1 = motor_config["can0"][0];
        uint32_t test_motor2 = motor_config["can0"][1];
        
        std::cout << "\n步骤1: 使能电机 " << test_motor1 << " " << test_motor2 << std::endl;
        // robot.enable_motor("can0", test_motor1, 4);  // 模式4
        // robot.enable_motor("can0", test_motor2, 4);  // 模式4
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n按回车开始电机运动测试..." << std::endl;
        // 临时取消订阅以避免输出干扰
        status_subscription.reset();
        std::cin.get();
        // 恢复订阅
        status_subscription = event_bus->subscribe<hardware_driver::event::MotorStatusEvent>(
            [event_handler](const auto& event) {
                event_handler->handle_motor_status(event);
            }
        );

        std::cout << "\n步骤2: 控制电机运动" << std::endl;
        
        // 正向旋转
        std::cout << "正向旋转 5 度/秒..." << std::endl;
        robot.control_motor_in_velocity_mode("can0", test_motor1, 5.0);
        robot.control_motor_in_velocity_mode("can0", test_motor2, 5.0);
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
        
        std::cout << "\n按回车退出程序..." << std::endl;
        // 临时取消订阅以避免输出干扰
        status_subscription.reset();
        std::cin.get();
        std::cout << "程序结束。" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        std::cerr << "请检查：" << std::endl;
        std::cerr << "1. 电机是否连接到can0总线" << std::endl;
        std::cerr << "2. 电机ID是否为1和9（可修改motor_config中的ID）" << std::endl;
        std::cerr << "3. 电机是否已上电" << std::endl;
        return 1;
    }
    
    return 0;
} 