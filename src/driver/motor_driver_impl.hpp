#ifndef __MOTOR_DRIVER_IMPL_HPP__
#define __MOTOR_DRIVER_IMPL_HPP__

#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "protocol/motor_protocol.hpp"
#include "hardware_driver/bus/bus_interface.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include <iostream>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <functional>
#include <atomic>
#include <vector>
#include <thread>
#include <condition_variable>
#include <shared_mutex>
#include <pthread.h>
#include <any>
#include <memory>
#include <map>
#include <array>

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

// 命令优先级枚举
enum class CommandPriority : uint8_t {
    LOW = 0,        // 低优先级：普通控制命令（速度、位置等）
    NORMAL = 1,     // 正常优先级：参数读写
    HIGH = 2,       // 高优先级：使能/失能命令
    EMERGENCY = 3   // 紧急优先级：紧急停止、故障清除
};

// 带优先级的控制命令包装
struct PriorityCommand {
    bus::GenericBusPacket packet;
    CommandPriority priority;
    std::chrono::steady_clock::time_point timestamp;
    
    PriorityCommand(const bus::GenericBusPacket& pkt, CommandPriority prio = CommandPriority::NORMAL) 
        : packet(pkt), priority(prio), timestamp(std::chrono::steady_clock::now()) {}
};

// 优先级比较器：优先级高的先执行，同优先级按时间排序
struct PriorityComparator {
    bool operator()(const PriorityCommand& a, const PriorityCommand& b) const {
        if (a.priority != b.priority) {
            return static_cast<uint8_t>(a.priority) < static_cast<uint8_t>(b.priority);  // 优先级高的在前
        }
        return a.timestamp > b.timestamp;  // 同优先级按时间排序，早的在前
    }
};

// 电机状态观察者接口
class MotorStatusObserver {
public:
    virtual ~MotorStatusObserver() = default;
    
    // 单个电机状态更新事件
    virtual void on_motor_status_update(const std::string& interface, 
                                       uint32_t motor_id, 
                                       const Motor_Status& status) = 0;
    
    // 批量电机状态更新事件 - 一个接口的所有电机状态
    virtual void on_motor_status_update(const std::string& /*interface*/,
                                       const std::map<uint32_t, Motor_Status>& /*status_all*/) {}
    
    // 函数操作结果事件
    virtual void on_motor_function_result(const std::string& /*interface*/,
                                         uint32_t /*motor_id*/,
                                         uint8_t /*op_code*/,
                                         bool /*success*/) {}
    
    // 参数读写结果事件
    virtual void on_motor_parameter_result(const std::string& /*interface*/,
                                          uint32_t /*motor_id*/,
                                          uint16_t /*address*/,
                                          uint8_t /*data_type*/,
                                          const std::any& /*data*/) {}
};

// 可配置的时序参数结构体
struct TimingConfig {
    std::chrono::microseconds control_interval{200};     // 控制命令间隔
    std::chrono::microseconds high_freq_feedback{400};   // 高频反馈间隔 (2.5kHz)
    std::chrono::milliseconds low_freq_feedback{50};     // 低频反馈间隔 (20Hz)
    std::chrono::milliseconds mode_timeout{100};         // 高频模式超时
    int control_cpu_core{4};                           // 控制线程CPU绑定 (-1表示不绑定)(2~5 P核心可以获得最佳实时性能)
};

class MotorDriverImpl : public MotorDriverInterface {
public:
    // 队列大小限制
    static constexpr size_t MAX_QUEUE_SIZE = 128;
    
    // 回调函数类型定义
    using FeedbackCallback = std::function<void(const std::string& interface, 
                                               uint32_t motor_id, 
                                               const Motor_Status& status)>;

    explicit MotorDriverImpl(std::shared_ptr<bus::BusInterface> bus);
    
    // 事件总线集成
    void set_event_bus(std::shared_ptr<event::EventBus> event_bus);
    std::shared_ptr<event::EventBus> get_event_bus() const;
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

    // 新增公共接口
    void register_feedback_callback(FeedbackCallback callback);
    void send_control_command(const bus::GenericBusPacket& packet);
    void send_control_command(const bus::GenericBusPacket& packet, CommandPriority priority);
    bool send_control_command_timeout(const bus::GenericBusPacket& packet, std::chrono::milliseconds timeout = std::chrono::milliseconds(10));
    
    // 便捷的紧急停止接口
    void send_emergency_stop(const std::string& interface, uint32_t motor_id);

    // 设置要监控的电机配置（用于反馈请求）
    void set_motor_config(const std::map<std::string, std::vector<uint32_t>>& config);

    // 观察者模式接口
    void add_observer(std::shared_ptr<MotorStatusObserver> observer);
    void remove_observer(std::shared_ptr<MotorStatusObserver> observer);
    
private:
    std::shared_ptr<bus::BusInterface> bus_;
    // 简化的状态存储 - 使用线程安全哈希表，只保存最新状态
    std::unordered_map<Motor_Key, Motor_Status> status_map_;
    mutable std::shared_mutex status_map_mutex_;  // 读写锁，支持多读者单写者
    FeedbackCallback feedback_callback_;
    
    // 观察者模式支持
    std::vector<std::weak_ptr<MotorStatusObserver>> observers_;
    std::mutex observers_mutex_;  // 保护观察者列表

    // 控制命令优先级队列和同步
    std::priority_queue<PriorityCommand, std::vector<PriorityCommand>, PriorityComparator> control_priority_queue_;
    std::mutex control_mutex_;
    std::condition_variable control_cv_;
    
    // 重复命令过滤机制
    std::unordered_map<Motor_Key, bus::GenericBusPacket> last_commands_;  // 存储每个电机的最后一条命令
    mutable std::mutex last_commands_mutex_;  // 保护最后命令映射
    
    // 接收数据队列和同步
    std::queue<bus::GenericBusPacket> receive_queue_;
    std::mutex receive_mutex_;
    std::condition_variable receive_cv_;

    // 频率控制
    std::atomic<bool> high_freq_mode_{false};
    std::chrono::steady_clock::time_point last_control_time_;
    std::map<std::string, std::vector<uint32_t>> interface_motor_config_;
    
    // 时序参数
    TimingConfig timing_config_;

    // 三线程架构
    std::thread feedback_request_thread_;   // 反馈线程：发送请求
    std::thread data_processing_thread_;   // 数据处理线程：处理接收队列
    std::thread control_thread_;    // 控制线程：专门发送控制命令
    std::atomic<bool> running_{true};

    // 三线程工作函数
    void feedback_request_worker();        // 反馈请求线程：定时发送反馈请求
    void data_processing_worker();         // 数据处理线程：阻塞处理接收队列  
    void control_worker();                 // 控制线程：发送控制命令
    
    // 数据处理函数  
    void handle_bus_packet(const bus::GenericBusPacket& packet);  // 处理单个数据包
    // 请求反馈数据
    bus::GenericBusPacket create_feedback_request_all(const std::string& interface);
    
    // CPU亲和性设置
    void set_thread_cpu_affinity(int cpu_core);
    
    // 系统负载检测和自适应CPU绑定
    double get_system_load_average() const;
    void cleanup_cpu_binding();
    
    // 通知观察者的私有方法
    void notify_motor_status_observers(const std::string& interface, uint32_t motor_id, const Motor_Status& status);
    void notify_function_result_observers(const std::string& interface, uint32_t motor_id, uint8_t op_code, bool success);
    void notify_parameter_result_observers(const std::string& interface, uint32_t motor_id, uint16_t address, uint8_t data_type, const std::any& data);
    
    // 重复命令检测
    bool is_duplicate_command(const bus::GenericBusPacket& packet);
    
    // 事件总线支持
    std::shared_ptr<hardware_driver::event::EventBus> event_bus_;
    mutable std::mutex event_bus_mutex_;
    
    // 事件发布方法
    void emit_motor_status_event(const std::string& interface, uint32_t motor_id, const Motor_Status& status);
    void emit_motor_batch_status_event(const std::string& interface, const std::map<uint32_t, Motor_Status>& status_all);
    void emit_motor_function_result_event(const std::string& interface, uint32_t motor_id, uint8_t op_code, bool success);
    void emit_motor_parameter_result_event(const std::string& interface, uint32_t motor_id, uint16_t address, uint8_t data_type, const std::any& data);
};

}    // namespace motor_driver
}    // namespace hardware_driver

#endif    // __MOTOR_DRIVER_IMPL_HPP__
