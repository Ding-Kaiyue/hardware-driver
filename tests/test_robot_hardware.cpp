#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"

#define CAN0_ENABLE

// 简单的Mock MotorDriverInterface用于测试
class MockMotorDriver : public hardware_driver::motor_driver::MotorDriverInterface {
public:
    MOCK_METHOD(void, enable_motor, (const std::string interface, const uint32_t motor_id, uint8_t mode), (override));
    MOCK_METHOD(void, send_position_cmd, (const std::string interface, const uint32_t motor_id, float position), (override));
    MOCK_METHOD(void, send_velocity_cmd, (const std::string interface, const uint32_t motor_id, float velocity), (override));
    MOCK_METHOD(void, send_effort_cmd, (const std::string interface, const uint32_t motor_id, float effort), (override));
    MOCK_METHOD(void, send_mit_cmd, (const std::string interface, const uint32_t motor_id, float position, float velocity, float effort), (override));
    MOCK_METHOD(void, disable_motor, (const std::string interface, const uint32_t motor_id), (override));
    MOCK_METHOD(void, motor_parameter_read, (const std::string interface, const uint32_t motor_id, uint16_t address), (override));
    MOCK_METHOD(void, motor_parameter_write, (const std::string interface, const uint32_t motor_id, uint16_t address, int32_t value), (override));
    MOCK_METHOD(void, motor_parameter_write, (const std::string interface, const uint32_t motor_id, uint16_t address, float value), (override));
    MOCK_METHOD(void, motor_function_operation, (const std::string interface, const uint32_t motor_id, uint8_t operation), (override));
};

using ::testing::_;
using namespace hardware_driver::event;

// 测试用的电机状态观察者
class TestMotorStatusCollector : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    std::map<std::string, std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>> latest_status;
    std::mutex status_mutex;
    std::atomic<int> status_received_count{0};
    std::atomic<int> batch_received_count{0};
    
    TestMotorStatusCollector(std::shared_ptr<EventBus> event_bus) : event_bus_(event_bus) {
        // 只有在event_bus不为空时才订阅事件
        if (event_bus_) {
            // 订阅单个电机状态事件
            motor_status_handler_ = event_bus_->subscribe<MotorStatusEvent>(
                [this](const std::shared_ptr<MotorStatusEvent>& event) {
                    std::lock_guard<std::mutex> lock(status_mutex);
                    latest_status[event->get_interface()][event->get_motor_id()] = event->get_status();
                    status_received_count++;
                });
                
            // 订阅批量电机状态事件
            batch_status_handler_ = event_bus_->subscribe<MotorBatchStatusEvent>(
                [this](const std::shared_ptr<MotorBatchStatusEvent>& event) {
                    std::lock_guard<std::mutex> lock(status_mutex);
                    for (const auto& [motor_id, status] : event->get_status_all()) {
                        latest_status[event->get_interface()][motor_id] = status;
                    }
                    batch_received_count++;
                });
        }
    }
    
    ~TestMotorStatusCollector() {
        if (event_bus_ && motor_status_handler_) {
            event_bus_->unsubscribe<MotorStatusEvent>(motor_status_handler_);
        }
        if (event_bus_ && batch_status_handler_) {
            event_bus_->unsubscribe<MotorBatchStatusEvent>(batch_status_handler_);
        }
    }
    
    // 实现 MotorStatusObserver 接口
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        latest_status[interface][motor_id] = status;
        status_received_count++;
        
        // 只有在event_bus不为空时才发布到事件总线
        if (event_bus_) {
            event_bus_->emit<MotorStatusEvent>(interface, motor_id, status);
        }
    }
    
    void on_motor_status_update(const std::string& interface,
                               const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        for (const auto& [motor_id, status] : status_all) {
            latest_status[interface][motor_id] = status;
        }
        batch_received_count++;
        
        // 只有在event_bus不为空时才发布到事件总线
        if (event_bus_) {
            event_bus_->emit<MotorBatchStatusEvent>(interface, status_all);
        }
    }
    
    hardware_driver::motor_driver::Motor_Status get_motor_status(const std::string& interface, uint32_t motor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        if (latest_status.find(interface) != latest_status.end() &&
            latest_status[interface].find(motor_id) != latest_status[interface].end()) {
            return latest_status[interface][motor_id];
        }
        // 返回默认状态如果没有找到
        return hardware_driver::motor_driver::Motor_Status{};
    }
    
    std::map<uint32_t, hardware_driver::motor_driver::Motor_Status> get_all_motor_status(const std::string& interface) {
        std::lock_guard<std::mutex> lock(status_mutex);
        if (latest_status.find(interface) != latest_status.end()) {
            return latest_status[interface];
        }
        return {};
    }
    
private:
    std::shared_ptr<EventBus> event_bus_;
    std::shared_ptr<EventHandler> motor_status_handler_;
    std::shared_ptr<EventHandler> batch_status_handler_;
};

TEST(RobotHardwareTest, VelocityAndDisable) {
    // 检查是否有可用的CAN接口 - 只使用can0（实际有硬件连接的接口）
    std::vector<std::string> available_interfaces;
    std::vector<std::string> test_interfaces = {"can0"};  // 只测试can0
    
    for (const auto& interface : test_interfaces) {
        try {
            std::vector<std::string> single_interface = {interface};
            auto test_bus = std::make_shared<hardware_driver::bus::CanFdBus>(single_interface);
            available_interfaces.push_back(interface);
        } catch (const std::exception& e) {
            // 接口不可用，跳过
        }
    }
    
    if (available_interfaces.empty()) {
        GTEST_SKIP() << "No CAN interfaces available for testing";
    }
    
    // 创建CAN总线接口 - 使用可用的接口
    auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(available_interfaces);

    // 使用真实的电机驱动
    auto real_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
    std::map<std::string, std::vector<uint32_t>> config;
    
    // 根据可用接口配置电机 - 使用实际连接的硬件配置
    for (const auto& interface : available_interfaces) {
        if (interface == "can0" || interface == "vcan0") {
            config["can0"] = {1, 9};  // 实际连接的电机ID
        }
        // 移除can1配置，因为没有硬件连接
    }
    
    // 使用简单的观察者模式，类似示例代码
    auto status_collector = std::make_shared<TestMotorStatusCollector>(nullptr);  // 不使用事件总线
    real_driver->add_observer(status_collector);
    
    // 创建RobotHardware实例，使用观察者模式构造函数
    RobotHardware hw(real_driver, config, status_collector);

    std::cout << "=== 速度指令和失能测试 ===" << std::endl;
    std::cout << "可用接口: ";
    for (const auto& interface : available_interfaces) {
        std::cout << interface << " ";
    }
    std::cout << std::endl;
    
    // 等待系统稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 根据可用接口发送速度命令 - 只对can0发送
    for (const auto& interface : available_interfaces) {
        std::string config_interface = (interface == "vcan0") ? "can0" : interface;
        
        if (config.find(config_interface) != config.end() && !config[config_interface].empty()) {
            std::cout << "发送" << config_interface << " 速度命令 (50.0 degrees/s)" << std::endl;
            for (auto motor_id : config[config_interface]) {
                hw.control_motor_in_velocity_mode(config_interface, motor_id, 50.0f);
            }
        }
    }
    
    // 等待电机转动
    std::cout << "等待电机转动..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 等待一段时间让状态反馈到达
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 检查电机状态
    std::cout << "检查电机转动状态:" << std::endl;
    std::cout << "状态事件接收计数: " << status_collector->status_received_count.load() << std::endl;
    
    for (const auto& interface : available_interfaces) {
        std::string config_interface = (interface == "vcan0") ? "can0" : interface;
        
        if (config.find(config_interface) != config.end() && !config[config_interface].empty()) {
            for (auto motor_id : config[config_interface]) {
                auto status = status_collector->get_motor_status(config_interface, motor_id);
                std::cout << "  " << config_interface << " 电机" << motor_id 
                          << " 速度: " << status.velocity 
                          << " 使能标志: " << (int)status.enable_flag 
                          << " 电机模式: " << (int)status.motor_mode << std::endl;
            }
        }
    }

    // 发送速度0命令停止电机
    for (const auto& interface : available_interfaces) {
        std::string config_interface = (interface == "vcan0") ? "can0" : interface;
        
        if (config.find(config_interface) != config.end() && !config[config_interface].empty()) {
            std::cout << "发送" << config_interface << " 速度0命令停止电机" << std::endl;
            for (auto motor_id : config[config_interface]) {
                hw.control_motor_in_velocity_mode(config_interface, motor_id, 0.0f);
            }
        }
    }
    
    // 等待电机完全停止
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 失能电机 - 只对实际连接的电机进行操作
    for (const auto& interface : available_interfaces) {
        std::string config_interface = (interface == "vcan0") ? "can0" : interface;
        
        if (config.find(config_interface) != config.end() && !config[config_interface].empty()) {
            std::cout << "失能" << config_interface << " 电机" << std::endl;
            for (auto motor_id : config[config_interface]) {
                hw.disable_motor(config_interface, motor_id);
            }
        }
    }
    // 等待失能命令生效和状态反馈
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 获取电机状态
    std::cout << "最终状态事件接收计数: " << status_collector->status_received_count.load() << std::endl;
    
    auto can0_status = status_collector->get_all_motor_status("can0");
    std::cout << "can0电机状态:" << std::endl;
    for (const auto& [motor_id, status] : can0_status) {
        std::cout << "can0 motor" << motor_id << "  使能标志: " << (int)status.enable_flag << std::endl;
        std::cout << "can0 motor" << motor_id << "  速度: " << status.velocity << std::endl;
    }
    
    // 验证观察者模式正常工作
    std::cout << "状态观察者统计: 接收状态更新=" << status_collector->status_received_count.load() << std::endl;
    
    // 明确清理资源，避免析构时的竞态条件
    std::cout << "清理测试资源..." << std::endl;
    real_driver->remove_observer(status_collector);
    real_driver.reset();
    bus.reset();
    
    // 等待所有资源完全释放
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "测试完成" << std::endl;
}
