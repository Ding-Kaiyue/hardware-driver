/*********************************************************************
 * 本文件实现了GripperDriverInterface接口，负责：
 * - 将高层夹爪控制命令转换为底层总线协议
 * - 处理夹爪反馈数据
 * - 管理观察者模式的状态回调
 *********************************************************************/

#ifndef __GRIPPER_DRIVER_IMPL_HPP__
#define __GRIPPER_DRIVER_IMPL_HPP__

#include "hardware_driver/driver/gripper_driver_interface.hpp"
#include "hardware_driver/bus/bus_interface.hpp"
#include "../protocol/gripper_omnipicker_protocol.hpp"
#include <memory>
#include <vector>
#include <mutex>
#include <map>

namespace hardware_driver {
namespace gripper_driver {

//夹爪驱动实现类  使用观察者模式实现状态回调，支持多个总线接口
class GripperDriverImpl : public GripperDriverInterface {
public:
    //构造函数    bus 总线接口对象
    explicit GripperDriverImpl(std::shared_ptr<bus::BusInterface> bus);

    // 析构函数
    ~GripperDriverImpl() override;

    // 实现 GripperDriverInterface 接口 

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

    // 启动异步接收线程
    void start_receive();

    //停止异步接收线程
    void stop_receive();

private:
    //接收回调函数，处理总线接收到的数据包
    void receive_callback(const bus::GenericBusPacket& packet);

    //通知所有观察者
    void notify_observers(const std::string& interface,
                         uint32_t gripper_id,
                         const GripperStatus& status);

    //发送数据包到总线
    bool send_packet(const bus::GenericBusPacket& packet);

    std::shared_ptr<bus::BusInterface> bus_;                        ///< 总线接口
    std::vector<std::shared_ptr<GripperStatusObserver>> observers_; ///< 观察者列表
    std::mutex observers_mutex_;                                    ///< 观察者列表互斥锁
    std::mutex send_mutex_;                                         ///< 发送互斥锁

    // 状态缓存 (interface -> gripper_id -> status)
    std::map<std::string, std::map<uint32_t, GripperStatus>> status_cache_;
    std::mutex cache_mutex_;
};

}  // namespace gripper_driver
}  // namespace hardware_driver

#endif  // __GRIPPER_DRIVER_IMPL_HPP__
