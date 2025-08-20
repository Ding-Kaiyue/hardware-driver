#ifndef __EVENT_BRIDGE_HPP__
#define __EVENT_BRIDGE_HPP__

#include "driver/motor_driver_impl.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include <memory>

namespace hardware_driver {
namespace event {

// 事件桥接器：将观察者模式转换为事件总线
class EventBridge : public motor_driver::MotorStatusObserver {
public:
    explicit EventBridge(std::shared_ptr<EventBus> event_bus)
        : event_bus_(event_bus) {}
    
    // 实现观察者接口 - 单个电机
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const motor_driver::Motor_Status& status) override {
        if (event_bus_) {
            event_bus_->emit<MotorStatusEvent>(interface, motor_id, status);
        }
    }
    
    // 实现观察者接口 - 批量电机
    void on_motor_status_update(const std::string& interface,
                               const std::map<uint32_t, motor_driver::Motor_Status>& status_all) override {
        if (event_bus_) {
            event_bus_->emit<MotorBatchStatusEvent>(interface, status_all);
        }
    }
    
    void on_motor_function_result(const std::string& interface,
                                 uint32_t motor_id,
                                 uint8_t op_code,
                                 bool success) override {
        if (event_bus_) {
            event_bus_->emit<MotorFunctionResultEvent>(interface, motor_id, op_code, success);
        }
    }
    
    void on_motor_parameter_result(const std::string& interface,
                                  uint32_t motor_id,
                                  uint16_t address,
                                  uint8_t data_type,
                                  const std::any& data) override {
        if (event_bus_) {
            event_bus_->emit<MotorParameterResultEvent>(interface, motor_id, address, data_type, data);
        }
    }
    
    // 获取事件总线引用
    std::shared_ptr<EventBus> get_event_bus() const {
        return event_bus_;
    }
    
private:
    std::shared_ptr<EventBus> event_bus_;
};

}  // namespace event
}  // namespace hardware_driver

#endif  // __EVENT_BRIDGE_HPP__
