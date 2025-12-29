#ifndef __MOTOR_DRIVER_INTERFACE_HPP__
#define __MOTOR_DRIVER_INTERFACE_HPP__

#include <string>
#include <vector>
#include <map>
#include <any>
#include <memory>

namespace hardware_driver {
namespace motor_driver {

// 电机数据结构体定义
typedef struct {
    uint8_t enable_flag : 1;
    uint8_t motor_mode : 4;
    float position;

    uint8_t limit_flag : 1;
    uint16_t temperature : 10;
    float velocity;

    uint16_t voltage : 10;
    float effort;

    uint32_t error_code;
} Motor_Status;

/**
 * @brief IAP设备状态消息枚举
 * 设备通过CAN-FD发送ASCII编码的4字节状态消息
 */
enum class IAPStatus : uint32_t {
    BS00 = 0x42533030,  // "BS00" - Bootloader启动
    BK01 = 0x424B3031,  // "BK01" - 收到"key"命令，进入IAP模式
    BK02 = 0x424B3032,  // "BK02" - 擦除APP程序
    BK03 = 0x424B3033,  // "BK03" - 准备接收固件数据
    BD04 = 0x42443034,  // "BD04" - 开始接收数据
    BD05 = 0x42443035,  // "BD05" - 接收完成（500ms内无数据）
    BJ06 = 0x424A3036,  // "BJ06" - APP地址正确，准备跳转
    BJ07 = 0x424A3037,  // "BJ07" - 跳转错误
    AS00 = 0x41533030,  // "AS00" - APP启动成功
    AJ01 = 0x414A3031,  // "AJ01" - APP收到IAP更新指令
};

// 电机状态观察者接口
class MotorStatusObserver {
public:
    virtual ~MotorStatusObserver() = default;
    
    // 单个电机状态更新事件
    virtual void on_motor_status_update(const std::string& /*interface*/, 
                                       uint32_t /*motor_id*/, 
                                       const Motor_Status& /*status*/) = 0;
    
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

/**
 * @brief  IAP 状态反馈观察者接口
 */
class IAPStatusObserver {
public:
    virtual ~IAPStatusObserver() = default;

    /**
     * @brief 当收到 IAP 状态消息（如 "BS00"、"BK01"、"BD05"、"AS00"）时回调
     */
    virtual void on_iap_status_feedback(const std::string& /*interface*/,
                                        uint32_t /*motor_id*/,
                                        const IAPStatus& /*msg*/) {}
};

// 事件总线模式的事件处理器接口 - 与观察者模式保持相同的虚函数签名
class MotorEventHandler {
public:
    virtual ~MotorEventHandler() = default;
    
    // 单个电机状态更新事件
    virtual void on_motor_status_update(const std::string& /*interface*/, 
                                       uint32_t /*motor_id*/, 
                                       const Motor_Status& /*status*/) = 0;
    
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


class MotorDriverInterface {
public:
    virtual ~MotorDriverInterface() = default;

    virtual void disable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) = 0;
    virtual void disable_all_motors(const std::string interface, std::vector<uint32_t> motor_ids, uint8_t mode) = 0;
    virtual void enable_motor(const std::string interface, const uint32_t motor_id, uint8_t mode) = 0;
    virtual void enable_all_motors(const std::string interface, std::vector<uint32_t> motor_ids, uint8_t mode) = 0;
    virtual void send_position_cmd(const std::string interface, const uint32_t motor_id, float position, 
                                    float kp, float kd) = 0;
    virtual void send_velocity_cmd(const std::string interface, const uint32_t motor_id, float velocity, 
                                    float kp, float kd) = 0;
    virtual void send_effort_cmd(const std::string interface, const uint32_t motor_id, float effort, 
                                    float kp, float kd) = 0;
    virtual void send_mit_cmd(const std::string interface, const uint32_t motor_id, float position, float velocity, float effort, 
                                    float kp, float kd) = 0;
    virtual void send_control_cmd(const std::string interface, 
                                    std::vector<float> positions, 
                                    std::vector<float> velocities, 
                                    std::vector<float> efforts, 
                                    std::vector<float> kps, 
                                    std::vector<float> kds) = 0;

    virtual void motor_function_operation(const std::string interface, const uint32_t motor_id, uint8_t operation) = 0;
    virtual void motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, int32_t value) = 0;
    virtual void motor_parameter_write(const std::string interface, const uint32_t motor_id, uint16_t address, float value) = 0;
    virtual void motor_parameter_read(const std::string interface, const uint32_t motor_id, uint16_t address) = 0;

    // 批量控制接口（广播给所有电机，支持最多 6 个）
    // kp 和 kd 默认值：0.05 和 0.005
    virtual void send_position_cmd_all(const std::string interface, const std::array<float, 6>& positions,
                                      const std::array<float, 6>& kps = position_kps,
                                      const std::array<float, 6>& kds = position_kds) = 0;
    virtual void send_velocity_cmd_all(const std::string interface, const std::array<float, 6>& velocities,
                                      const std::array<float, 6>& kps = velocity_kps,
                                      const std::array<float, 6>& kds = velocity_kds) = 0;
    virtual void send_effort_cmd_all(const std::string interface, const std::array<float, 6>& efforts,
                                    const std::array<float, 6>& kps = position_kps,
                                    const std::array<float, 6>& kds = position_kds) = 0;
    virtual void send_mit_cmd_all(const std::string interface, const std::array<float, 6>& positions,
                                 const std::array<float, 6>& velocities, const std::array<float, 6>& efforts,
                                 const std::array<float, 6>& kps = position_kps,
                                 const std::array<float, 6>& kds = position_kds) = 0;

    // IAP (In-Application Programming) firmware update
    virtual void start_update(const std::string& interface, uint32_t motor_id, const std::string& firmware_file) = 0;

private:
    static constexpr std::array<float, 6> position_kps = {0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f};
    static constexpr std::array<float, 6> position_kds = {0.005f, 0.005f, 0.005f, 0.005f, 0.005f, 0.005f};
    static constexpr std::array<float, 6> velocity_kps = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static constexpr std::array<float, 6> velocity_kds = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
};


}    // namespace motor_driver
}    // namespace hardware_driver


#endif   // __MOTOR_DRIVER_INTERFACE_HPP__
