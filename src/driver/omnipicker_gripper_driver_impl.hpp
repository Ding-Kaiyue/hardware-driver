/*********************************************************************
 * @file        omnipicker_gripper_driver_impl.hpp
 * @brief       OmniPicker 夹爪驱动实现（专用）
 *********************************************************************/

#ifndef __OMNIPICKER_GRIPPER_DRIVER_IMPL_HPP__
#define __OMNIPICKER_GRIPPER_DRIVER_IMPL_HPP__

#include "hardware_driver/driver/gripper_driver_interface.hpp"
#include "hardware_driver/bus/bus_interface.hpp"
#include "../protocol/gripper_omnipicker_protocol.hpp"

#include <memory>
#include <vector>
#include <mutex>
#include <map>

namespace hardware_driver {
namespace gripper_driver {

class OmniPickerGripperDriverImpl : public GripperDriverInterface {
public:
    explicit OmniPickerGripperDriverImpl(std::shared_ptr<bus::BusInterface> bus);
    ~OmniPickerGripperDriverImpl() override;

    void control_gripper(const std::string& interface,
                        GripperType gripper_type,
                        uint8_t position,
                        uint8_t velocity,
                        uint8_t effort) override;

    void send_raw_data(const std::string& interface,
                      const uint8_t* raw_data,
                      size_t raw_data_len) override;

    void open_gripper(const std::string& interface,
                     GripperType gripper_type,
                     uint8_t velocity = 50,
                     uint8_t effort = 50) override;

    void close_gripper(const std::string& interface,
                      GripperType gripper_type,
                      uint8_t velocity = 50,
                      uint8_t effort = 50) override;

    void add_observer(std::shared_ptr<GripperStatusObserver> observer) override;
    void remove_observer(std::shared_ptr<GripperStatusObserver> observer) override;

private:
    void receive_callback(const bus::GenericBusPacket& packet);
    void notify_observers(const std::string& interface,
                         uint32_t gripper_id,
                         const GripperStatus& status);
    bool send_packet(const bus::GenericBusPacket& packet);

private:
    std::shared_ptr<bus::BusInterface> bus_;
    std::vector<std::shared_ptr<GripperStatusObserver>> observers_;
    std::mutex observers_mutex_;

    std::map<std::string, std::map<uint32_t, GripperStatus>> status_cache_;
    std::mutex cache_mutex_;
};

}  // namespace gripper_driver
}  // namespace hardware_driver

#endif  // __OMNIPICKER_GRIPPER_DRIVER_IMPL_HPP__
