#ifndef __MOTOR_DRIVER_IMPL_HPP__
#define __MOTOR_DRIVER_IMPL_HPP__

#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "protocol/motor_protocol.hpp"
#include "hardware_driver/bus/bus_interface.hpp"
#include <iostream>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <functional>
#include <atomic>
#include <vector>
#include <thread>

// MotorKey 结构体和哈希
struct Motor_Key {
    std::string interface;
    uint32_t motor_id;
    bool operator==(const Motor_Key& other) const {
        return interface == other.interface && motor_id == other.motor_id;
    }
};

// 哈希特化
namespace std {
template<>
struct hash<Motor_Key> {
    std::size_t operator()(const Motor_Key& k) const {
        return std::hash<std::string>()(k.interface) ^ (std::hash<uint32_t>()(k.motor_id) << 1);
    }
};
}

namespace hardware_driver {
namespace motor_driver {

class MotorDriverImpl : public MotorDriverInterface {
public:
    explicit MotorDriverImpl(std::shared_ptr<bus::BusInterface> bus);
    ~MotorDriverImpl() override;

    void disable_motor(const std::string interface, const uint32_t motor_id) override;
    void enable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) override;
    void send_mit_cmd(const std::string interface, const uint32_t motor_id, float position, float velocity, float effort) override;
    void send_position_cmd(const std::string interface, const uint32_t motor_id, float position) override;
    void send_velocity_cmd(const std::string interface, const uint32_t motor_id, float velocity) override;
    void send_effort_cmd(const std::string interface, const uint32_t motor_id, float effort) override;
    
    void motor_parameter_read(const std::string interface, const uint32_t motor_id, uint16_t address) override;
    void motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, int32_t value) override;
    void motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, float value) override;
    void motor_function_operation(const std::string interface, const uint32_t motor_id, uint8_t operation) override;

    void motor_feedback_request(const std::string interface, const uint32_t motor_id) override;
    void motor_feedback_request_all(const std::string interface) override;

    Motor_Status get_motor_status(const std::string& interface, uint32_t motor_id) override;
    
private:
    std::shared_ptr<bus::BusInterface> bus_;
    std::unordered_map<Motor_Key, Motor_Status> motor_status_map_;
    std::mutex send_mutex_;      // 保护发送操作
    std::mutex receive_mutex_;   // 保护接收操作

    // 命令优先级队列
    enum class CommandPriority : int {
        // 高优先级 (8-10): 紧急、安全、控制命令
        EMERGENCY_STOP = 10,      // 紧急停止 (最高优先级)
        SAFETY_COMMAND = 9,       // 安全相关命令 (失能、使能) - 高优先级
        CONTROL_COMMAND = 8,      // 控制命令 (位置、速度、力矩) - 高优先级
        
        // 中优先级 (4-7): 参数操作、函数操作
        PARAM_WRITE = 7,          // 参数写入
        PARAM_READ = 6,           // 参数读取
        FUNCTION_OPERATION = 5,   // 函数操作
        STATUS_QUERY = 4,         // 状态查询
        
        // 低优先级 (1-3): 反馈请求、调试命令
        FEEDBACK_REQUEST = 3,     // 反馈请求 (低优先级，可被抢占)
        DEBUG_COMMAND = 2,        // 调试命令
        IDLE_COMMAND = 1          // 空闲命令 (最低优先级)
    };
    
    struct Command {
        enum class Type { CONTROL, FEEDBACK_REQUEST, PARAM_OP, FUNCTION_OP, SAFETY_OP } type;
        bus::GenericBusPacket packet;
        CommandPriority priority;
        std::chrono::steady_clock::time_point timestamp;
        
        bool operator<(const Command& other) const {
            return static_cast<int>(priority) < static_cast<int>(other.priority);  // 优先级高的先执行
        }
    };
    
    std::priority_queue<Command> command_queue_;
    std::mutex queue_mutex_;
    
    // 队列处理线程
    std::thread queue_processing_thread_;
    std::atomic<bool> running_{true};

    void send_packet(const bus::GenericBusPacket& packet);
    void send_feedback_packet(const bus::GenericBusPacket& packet);  // 专门用于反馈请求
    void handle_bus_packet(const bus::GenericBusPacket& packet);
    void send_with_priority(const bus::GenericBusPacket& packet, Command::Type type);
    void queue_processing_worker();  // 队列处理工作线程
};

}    // namespace motor_driver
}    // namespace hardware_driver

#endif    // __MOTOR_DRIVER_IMPL_HPP__
