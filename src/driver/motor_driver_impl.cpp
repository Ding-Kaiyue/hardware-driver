#include "motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"

namespace hardware_driver {
namespace motor_driver {

// #define PRINT_DEBUG 

MotorDriverImpl::MotorDriverImpl(std::shared_ptr<bus::BusInterface> bus)
    : bus_(std::move(bus))
{
    // 设置CAN FD和扩展帧
    auto canfd_bus = std::dynamic_pointer_cast<bus::CanFdBus>(bus_);
    if (canfd_bus) {
        bool use_canfd_ = true;
        bool use_extended_ = true;
        for (const auto& interface : canfd_bus->get_interface_names()) {
            canfd_bus->set_extended_frame(interface, use_extended_);
            canfd_bus->set_fd_mode(interface, use_canfd_);
        }
    }
    
    // 注册异步接收回调
    bus_->async_receive([this](const bus::GenericBusPacket& packet) {
        this->handle_bus_packet(packet);
    });
    
    // 启动队列处理线程
    queue_processing_thread_ = std::thread(&MotorDriverImpl::queue_processing_worker, this);
}

MotorDriverImpl::~MotorDriverImpl() {
    // 停止队列处理线程
    running_ = false;
    if (queue_processing_thread_.joinable()) {
        queue_processing_thread_.join();
    }
}

void MotorDriverImpl::disable_motor(const std::string interface, const uint32_t motor_id) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;  // 与其他控制命令保持一致
    
    if (motor_protocol::pack_disable_command(packet.data, packet.len)) {
        send_with_priority(packet, Command::Type::SAFETY_OP);
    }
}

void MotorDriverImpl::enable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_enable_command(packet.data, packet.len, mode)) {
        send_with_priority(packet, Command::Type::SAFETY_OP);
    }
}

void MotorDriverImpl::send_position_cmd(const std::string interface, const uint32_t motor_id, float position) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_position_command(packet.data, packet.len, position)) {
        send_with_priority(packet, Command::Type::CONTROL);
    }
}

void MotorDriverImpl::send_velocity_cmd(const std::string interface, const uint32_t motor_id, float velocity) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_velocity_command(packet.data, packet.len, velocity)) {
        // std::cout << "准备发送速度命令: ID=" << packet.id << ", 速度=" << velocity << " rad/s" << std::endl;
        send_with_priority(packet, Command::Type::CONTROL);
    } 
    // else {
    //     // std::cout << "速度命令打包失败!" << std::endl;
    // }
}

void MotorDriverImpl::send_effort_cmd(const std::string interface, const uint32_t motor_id, float effort) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_effort_command(packet.data, packet.len, effort)) {
        send_with_priority(packet, Command::Type::CONTROL);
    }
}

void MotorDriverImpl::send_mit_cmd(const std::string interface, const uint32_t motor_id, float position, float velocity, float effort) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id;
    
    if (motor_protocol::pack_mit_command(packet.data, packet.len, position, velocity, effort)) {
        send_with_priority(packet, Command::Type::CONTROL);
    }
}

void MotorDriverImpl::motor_parameter_read(const std::string interface, const uint32_t motor_id, uint16_t address) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x600;
    
    if (motor_protocol::pack_param_read(packet.data, packet.len, address)) {
        send_with_priority(packet, Command::Type::PARAM_OP);
    }
}

void MotorDriverImpl::motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, int32_t value) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x600;
    
    if (motor_protocol::pack_param_write(packet.data, packet.len, address, value)) {
        send_with_priority(packet, Command::Type::PARAM_OP);
    }
}

void MotorDriverImpl::motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, float value) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x600;
    
    if (motor_protocol::pack_param_write(packet.data, packet.len, address, value)) {
        send_with_priority(packet, Command::Type::PARAM_OP);
    }
}

void MotorDriverImpl::motor_function_operation(const std::string interface, const uint32_t motor_id, uint8_t operation) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x400;
    
    if (motor_protocol::pack_function_operation(packet.data, packet.len, operation)) {
        send_with_priority(packet, Command::Type::FUNCTION_OP);
    }
}

void MotorDriverImpl::motor_feedback_request(const std::string interface, const uint32_t motor_id) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = motor_id + 0x200;
    
    if (motor_protocol::pack_motor_feedback_request(packet.data, packet.len)) {
        send_with_priority(packet, Command::Type::FEEDBACK_REQUEST);
    }
}

void MotorDriverImpl::motor_feedback_request_all(const std::string interface) {
    bus::GenericBusPacket packet;
    packet.interface = interface;
    packet.id = 0; // 广播ID
    
    if (motor_protocol::pack_motor_feedback_request_all(packet.data, packet.len)) {
        send_with_priority(packet, Command::Type::FEEDBACK_REQUEST);
    }
}

Motor_Status MotorDriverImpl::get_motor_status(const std::string& interface, uint32_t motor_id) {
    Motor_Key key{interface, motor_id};
    auto it = motor_status_map_.find(key);
    if (it != motor_status_map_.end()) {
        return it->second;
    }
    return Motor_Status{};
}

void MotorDriverImpl::send_packet(const bus::GenericBusPacket& packet) {
    // 直接使用锁，确保命令能够发送
    std::lock_guard<std::mutex> lock(send_mutex_);
    bus_->send(packet);
}

void MotorDriverImpl::send_with_priority(const bus::GenericBusPacket& packet, Command::Type type) {
    Command cmd;
    cmd.type = type;
    cmd.packet = packet;
    
    // 根据命令类型分配优先级
    switch (type) {
        case Command::Type::CONTROL:
            cmd.priority = CommandPriority::CONTROL_COMMAND;
            break;
        case Command::Type::FEEDBACK_REQUEST:
            cmd.priority = CommandPriority::FEEDBACK_REQUEST;
            break;
        case Command::Type::PARAM_OP:
            // 根据数据包内容判断是读取还是写入
            if (packet.data[1] == 0x02) {  // 参数写入命令
                cmd.priority = CommandPriority::PARAM_WRITE;
            } else {  // 参数读取命令
                cmd.priority = CommandPriority::PARAM_READ;
            }
            break;
        case Command::Type::FUNCTION_OP:
            cmd.priority = CommandPriority::FUNCTION_OPERATION;
            break;
        case Command::Type::SAFETY_OP:
            cmd.priority = CommandPriority::SAFETY_COMMAND;
            break;
    }
    
    cmd.timestamp = std::chrono::steady_clock::now();
    
    // 高优先级命令直接发送，使用try_lock避免阻塞
    if (static_cast<int>(cmd.priority) >= static_cast<int>(CommandPriority::CONTROL_COMMAND)) {
        // 使用try_lock，如果无法获取锁则等待很短时间
        std::unique_lock<std::mutex> lock(send_mutex_, std::try_to_lock);
        if (lock.owns_lock()) {
            bus_->send(packet);
        } else {
            // 等待很短时间再试
            std::this_thread::sleep_for(std::chrono::microseconds(5)); // 减少到5us
            lock.lock();
            bus_->send(packet);
        }
        return;
    }
    
    // 低优先级命令放入队列，不立即处理
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push(cmd);
    }
}

void MotorDriverImpl::queue_processing_worker() {
    while (running_) {
        // 处理队列中的低优先级命令
        std::unique_lock<std::mutex> lock(send_mutex_, std::try_to_lock);
        if (lock.owns_lock()) {
            // 处理队列中的命令，限制每次处理的命令数量
            const int max_commands_per_batch = 5; // 增加批处理数量
            int processed_count = 0;
            
            while (processed_count < max_commands_per_batch) {
                Command cmd;
                bool has_command = false;
                
                // 从队列中取出一个命令
                {
                    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
                    if (!command_queue_.empty()) {
                        cmd = command_queue_.top();
                        command_queue_.pop();
                        has_command = true;
                    }
                }
                
                if (!has_command) {
                    break;  // 队列为空，退出
                }
                
                // 发送命令
                bus_->send(cmd.packet);
                processed_count++;
            }
        }
        
        // 休眠一段时间，避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::microseconds(25)); // 进一步减少到25us
    }
}


void MotorDriverImpl::handle_bus_packet(const bus::GenericBusPacket& packet) {
    std::lock_guard<std::mutex> lock(receive_mutex_);
    // 使用协议库解析反馈
    auto feedback_opt = motor_protocol::parse_feedback(packet);
    
    if (feedback_opt) {
        // 使用std::visit处理不同类型的反馈
        std::visit([this](auto&& feedback) {
            using T = std::decay_t<decltype(feedback)>;
            if constexpr (std::is_same_v<T, motor_protocol::MotorStatusFeedback>) {
                // 处理电机状态反馈
                Motor_Key key{feedback.interface, feedback.motor_id};
                motor_status_map_[key] = feedback.status;
#ifdef PRINT_DEBUG
                std::cout << "Interface: " << feedback.interface << " Motor ID: " 
                << feedback.motor_id << " enable_flag: " << feedback.status.enable_flag 
                << " motor_mode: " << feedback.status.motor_mode << " position: " << feedback.status.position
                << " limit_flag: " << feedback.status.limit_flag << " temperature: " << feedback.status.temperature
                << " velocity: " << feedback.status.velocity << " voltage: " << feedback.status.voltage
                << " effort: " << feedback.status.effort << " error_code: " << feedback.status.error_code
                << std::endl;
#endif
            } else if constexpr (std::is_same_v<T, motor_protocol::FuncResultFeedback>) {
                // 处理函数操作反馈
                Motor_Key key{feedback.interface, feedback.motor_id};
            } else if constexpr (std::is_same_v<T, motor_protocol::ParamResultFeedback>) {
                // 处理参数读写反馈
                Motor_Key key{feedback.interface, feedback.motor_id};
            }
        }, *feedback_opt);
    }
}

}   // namespace motor_driver
}   // namespace hardware_driver


