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
    
    // 处理电机状态事件
    void handle_motor_status(const std::shared_ptr<hardware_driver::event::MotorStatusEvent>& event) {
        const auto& status = event->get_status();
        std::cout << "🔄 电机状态事件: " << event->get_interface() << ":" << event->get_motor_id()
                  << " | 位置:" << status.position << "° | 速度:" << status.velocity << "°/s"
                  << " | 力矩:" << status.effort << "N⋅m | 温度:" << static_cast<int>(status.temperature) << "°C"
                  << " | 时间:" << std::chrono::duration_cast<std::chrono::milliseconds>(
                      event->get_timestamp().time_since_epoch()).count() << "ms" << std::endl;
    }
    
    // 处理批量电机状态事件
    void handle_batch_status(const std::shared_ptr<hardware_driver::event::MotorBatchStatusEvent>& event) {
        std::cout << "📊 批量状态事件: " << event->get_interface() 
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
    std::cout << "🚀 事件总线集成示例启动" << std::endl;
    std::cout << "========================================" << std::endl;
    
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
        
        // 4. 订阅事件 - 展示不同类型的事件订阅
        std::cout << "📡 开始订阅事件..." << std::endl;
        
        // 订阅电机状态事件
        auto status_subscription = event_bus->subscribe<hardware_driver::event::MotorStatusEvent>(
            [event_handler](const auto& event) {
                event_handler->handle_motor_status(event);
            }
        );
        std::cout << "  ✅ 订阅电机状态事件" << std::endl;
        
        // 订阅批量状态事件
        auto batch_subscription = event_bus->subscribe<hardware_driver::event::MotorBatchStatusEvent>(
            [event_handler](const auto& event) {
                event_handler->handle_batch_status(event);
            }
        );
        std::cout << "  ✅ 订阅批量状态事件" << std::endl;
        
        // 订阅函数操作结果事件
        auto function_subscription = event_bus->subscribe<hardware_driver::event::MotorFunctionResultEvent>(
            [event_handler](const auto& event) {
                event_handler->handle_function_result(event);
            }
        );
        std::cout << "  ✅ 订阅函数操作结果事件" << std::endl;
        
        // 订阅参数操作结果事件
        auto parameter_subscription = event_bus->subscribe<hardware_driver::event::MotorParameterResultEvent>(
            [event_handler](const auto& event) {
                event_handler->handle_parameter_result(event);
            }
        );
        std::cout << "  ✅ 订阅参数操作结果事件" << std::endl;
        
        // 5. 创建硬件组件
        std::cout << "🔧 创建硬件组件..." << std::endl;
        
        // 创建CAN总线
        auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
        std::cout << "  ✅ CAN-FD总线已创建" << std::endl;
        
        // 创建电机驱动
        auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
        std::cout << "  ✅ 电机驱动已创建" << std::endl;
        
        // 创建机器人硬件接口（集成事件总线）
        RobotHardware robot(motor_driver, motor_config, event_bus);
        std::cout << "  ✅ 机器人硬件接口已创建（事件总线已集成）" << std::endl;
        
        // 6. 演示事件自动发布
        std::cout << "\n🎯 开始演示事件自动发布..." << std::endl;
        std::cout << "注意：以下事件将自动发布到事件总线，无需手动调用" << std::endl;
        
        // 等待一下让系统初始化
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 演示电机控制（会自动触发状态事件）
        std::cout << "\n📤 发送电机控制命令..." << std::endl;
        robot.enable_motor("can0", 1, 4);
        std::cout << "  ✅ 电机1使能命令已发送" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        robot.control_motor_in_velocity_mode("can0", 1, 5.0);
        std::cout << "  ✅ 电机1速度控制命令已发送 (5°/s)" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // 演示参数操作（会自动触发参数结果事件）
        std::cout << "\n📤 发送参数操作命令..." << std::endl;
        robot.motor_parameter_read("can0", 1, 0x1001);
        std::cout << "  ✅ 电机1参数读取命令已发送 (地址:0x1001)" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        robot.motor_parameter_write("can0", 1, 0x1002, 1000);
        std::cout << "  ✅ 电机1参数写入命令已发送 (地址:0x1002, 值:1000)" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // 演示函数操作（会自动触发函数结果事件）
        std::cout << "\n📤 发送函数操作命令..." << std::endl;
        robot.motor_function_operation("can0", 1, 4);
        std::cout << "  ✅ 电机1函数操作命令已发送 (操作码:4)" << std::endl;
        
        // 7. 等待事件处理
        std::cout << "\n⏳ 等待事件处理..." << std::endl;
        std::cout << "事件将自动发布到事件总线，观察者将收到通知" << std::endl;
        
        // 等待一段时间让事件被处理
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 8. 演示主题订阅
        std::cout << "\n🎭 演示主题订阅..." << std::endl;
        
        // 订阅特定主题的事件
        auto topic_subscription = event_bus->subscribe_topic<hardware_driver::event::MotorStatusEvent>(
            "motor.can0.1.status",
            [](const auto& event) {
                std::cout << "🎯 主题订阅收到: 电机" << event->get_interface() 
                         << ":" << event->get_motor_id() << " 状态更新" << std::endl;
                std::cout << "   位置: " << event->get_status().position 
                         << ", 模式: " << static_cast<int>(event->get_status().motor_mode) << std::endl;
            }
        );
        std::cout << "  ✅ 订阅主题: motor.can0.1.status" << std::endl;
        
        // 再次发送命令触发主题事件
        robot.control_motor_in_position_mode("can0", 1, 90.0);
        std::cout << "  ✅ 电机1位置控制命令已发送 (90°)" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 9. 清理和总结
        std::cout << "\n🧹 清理资源..." << std::endl;
        
        // 停止电机
        robot.disable_motor("can0", 1);
        std::cout << "  ✅ 电机1已停止" << std::endl;
        
        // 取消订阅（可选，智能指针会自动管理）
        std::cout << "  ✅ 事件订阅将自动清理" << std::endl;
        
        std::cout << "\n🎉 事件总线集成示例完成！" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "✨ 主要特性演示:" << std::endl;
        std::cout << "  • 自动事件发布：电机状态、参数操作、函数操作" << std::endl;
        std::cout << "  • 类型安全订阅：支持所有事件类型" << std::endl;
        std::cout << "  • 主题过滤：支持按主题订阅特定事件" << std::endl;
        std::cout << "  • 完全解耦：电机驱动无需知道谁在监听" << std::endl;
        std::cout << "  • 实时性能：事件发布不影响控制线程" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ 错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 