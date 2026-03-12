/*********************************************************************
 * @file        pgc_gripper_driver_impl.hpp
 * @brief       PGC夹爪驱动实现（针对PGC夹爪的优化实现）
 *
 * 本文件实现了针对PGC夹爪的特定驱动，基于用户定义的CAN-FD到RS485
 * 转换协议，支持位置、速度、力量的精确控制。
 *********************************************************************/

#ifndef __PGC_GRIPPER_DRIVER_IMPL_HPP__
#define __PGC_GRIPPER_DRIVER_IMPL_HPP__

#include "hardware_driver/driver/gripper_driver_interface.hpp"
#include "hardware_driver/bus/bus_interface.hpp"
#include <memory>
#include <vector>
#include <mutex>
#include <map>
#include <atomic>

namespace hardware_driver {
namespace gripper_driver {

/**
 * @brief PGC夹爪驱动实现类
 *
 * 针对PGC夹爪的专用实现，继承GripperDriverInterface接口
 * 支持通过CAN-FD总线进行精确控制和反馈处理
 */
class PGCGripperDriverImpl : public GripperDriverInterface {
public:
    /**
     * @brief 构造函数
     * @param bus 总线接口对象
     */
    explicit PGCGripperDriverImpl(std::shared_ptr<bus::BusInterface> bus);

    /**
     * @brief 析构函数
     */
    ~PGCGripperDriverImpl() override;

    // ========== GripperDriverInterface 接口实现 ==========

    /**
     * @brief 控制夹爪
     * @param interface 总线接口名称 (如"can0")
     * @param gripper_type 夹爪类型
     * @param position 位置 (0-100%)
     * @param velocity 速度 (1-100%)
     * @param effort 力 (20-100%)
     */
    void control_gripper(const std::string& interface,
                        GripperType gripper_type,
                        uint8_t position,
                        uint8_t velocity,
                        uint8_t effort) override;

    /**
     * @brief 发送原始数据（Raw_Frame模式）
     * @param interface 总线接口名称
     * @param raw_data 原始数据指针
     * @param raw_data_len 原始数据长度
     */
    void send_raw_data(const std::string& interface,
                      const uint8_t* raw_data,
                      size_t raw_data_len) override;

    /**
     * @brief 打开夹爪（快捷方法）
     * @param interface 总线接口名称
     * @param gripper_type 夹爪类型
     * @param velocity 速度 (1-100%)
     * @param effort 力 (20-100%)
     */
    void open_gripper(const std::string& interface,
                     GripperType gripper_type,
                     uint8_t velocity = 50,
                     uint8_t effort = 50) override;

    /**
     * @brief 关闭夹爪（快捷方法）
     * @param interface 总线接口名称
     * @param gripper_type 夹爪类型
     * @param velocity 速度 (1-100%)
     * @param effort 力 (20-100%)
     */
    void close_gripper(const std::string& interface,
                      GripperType gripper_type,
                      uint8_t velocity = 50,
                      uint8_t effort = 50) override;

    /**
     * @brief 添加状态观察者
     * @param observer 观察者对象
     */
    void add_observer(std::shared_ptr<GripperStatusObserver> observer) override;

    /**
     * @brief 移除状态观察者
     * @param observer 观察者对象
     */
    void remove_observer(std::shared_ptr<GripperStatusObserver> observer) override;

    // ========== PGC特定接口 ==========

    /**
     * @brief 启动异步接收
     */
    void start_receive();

    /**
     * @brief 停止异步接收
     */
    void stop_receive();

    /**
     * @brief 查询夹爪状态
     * @param interface 总线接口名称
     */
    void query_gripper_status(const std::string& interface);

private:
    // ========== 私有方法 ==========

    /**
     * @brief 接收回调函数，处理总线接收到的数据包
     * @param packet 接收到的数据包
     */
    void receive_callback(const bus::GenericBusPacket& packet);

    /**
     * @brief 通知所有观察者
     * @param interface 总线接口名称
     * @param gripper_id 夹爪ID
     * @param status 夹爪状态
     */
    void notify_observers(const std::string& interface,
                         uint32_t gripper_id,
                         const GripperStatus& status);

    /**
     * @brief 发送数据包到总线
     * @param packet 数据包
     * @return 发送是否成功
     */
    bool send_packet(const bus::GenericBusPacket& packet);

    /**
     * @brief 构建PGC控制帧
     * @param velocity 速度 (1-100%)
     * @param effort 力 (20-100%)
     * @param position 位置 (0-100%)
     * @return 完整的CAN-FD帧数据
     */
    std::vector<uint8_t> build_pgc_control_frame(uint8_t velocity,
                                                  uint8_t effort,
                                                  uint8_t position);

    /**
     * @brief 构建PGC状态查询帧
     * @return 完整的CAN-FD帧数据
     */
    std::vector<uint8_t> build_pgc_status_query_frame();

    /**
     * @brief 使用CAN-FD格式封装数据
     * @param data 原始PGC数据
     * @return 完整的CAN-FD帧
     */
    std::vector<uint8_t> wrap_with_canfd_frame(const std::vector<uint8_t>& data);

    /**
     * @brief 解析PGC状态反馈
     * @param data 接收到的原始数据
     * @param data_len 数据长度
     * @return 解析后的GripperStatus结构体
     */
    GripperStatus parse_pgc_status(const uint8_t* data, size_t data_len);

    // ========== 成员变量 ==========

    std::shared_ptr<bus::BusInterface> bus_;                        ///< 总线接口
    std::vector<std::shared_ptr<GripperStatusObserver>> observers_; ///< 观察者列表
    std::mutex observers_mutex_;                                    ///< 观察者列表互斥锁

    // 状态缓存 (interface -> gripper_id -> status)
    std::map<std::string, std::map<uint32_t, GripperStatus>> status_cache_;
    std::mutex cache_mutex_;

    // CAN-FD帧格式常量
    static constexpr uint8_t CANFD_HEADER_0 = 0x55;
    static constexpr uint8_t CANFD_HEADER_1 = 0xaa;
    static constexpr uint8_t CANFD_TAIL_0 = 0xeb;
    static constexpr uint8_t CANFD_TAIL_1 = 0xaa;

    // PGC协议常量
    static constexpr uint8_t PGC_CONTROL_CMD = 0x02;
    static constexpr uint8_t PGC_STATUS_QUERY = 0x02;

    std::atomic<bool> running_{false};
};

}  // namespace gripper_driver
}  // namespace hardware_driver

#endif  // __PGC_GRIPPER_DRIVER_IMPL_HPP__
