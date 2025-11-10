#ifndef RAYTRON_MOTOR_CTRL_H
#define RAYTRON_MOTOR_CTRL_H

#include <cstdint>
#include <vector>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <atomic>
#include <functional>
#include <linux/can.h>
#include "protocol/raytron.h"
#include "data_convert/data_converter.h"

/*
 * @brief Raytron 协议转换器 (新广播帧协议) - 无状态，只转换协议
 *
 * 仅负责将控制值编码为CAN帧、将CAN数据解码为反馈值
 * 支持广播帧控制多个电机 (最多6个)
 * 不管理电机实例、状态、参数缓存等
 */

namespace raytron
{

// 电机反馈数据结构（不再创建实例，由外部管理）
struct MotorData
{
    uint8_t motor_id;                           // 电机ID
    raytron::ControlMode control_mode;          // 控制模式
    raytron::EnableFlag enable_flag;            // 使能标志

    // 反馈数据缓存
    float position;                             // 当前位置
    float velocity;                             // 当前速度
    float current;                              // 当前电流
    uint32_t error_code;                        // 错误代码
    uint16_t voltage;                           // 系统电压
    uint16_t temperature;                       // 温度

    // 时间戳
    std::chrono::system_clock::time_point timestamp;
};

class RaytronProtocolConverter
{
public:
    /**
     * @brief 默认构造函数
     */
    RaytronProtocolConverter() = default;

    /**
     * @brief 析构函数
     */
    ~RaytronProtocolConverter() = default;

    // ========== 协议编码接口 (新广播帧协议) ==========

    /**
     * @brief 编码广播请求反馈帧
     * 发送后所有电机会返回各自的反馈数据 (ID = 0x0300 + 电机ID)
     * @return 编码后的CAN帧
     */
    canfd_frame encodeBroadcastRequestFeedback();

    /**
     * @brief 编码广播设置ID帧
     * @param motor_id 要设置的电机ID (1-254)
     * @return 编码后的CAN帧
     */
    canfd_frame encodeBroadcastSetID(uint8_t motor_id);

    /**
     * @brief 编码广播使能帧 (设置所有电机的使能和控制模式)
     * @param flags 6个电机的标志位数组 (标志位 = (使能<<4) | 控制模式)
     * @param count 电机数量 (1-6)
     * @return 编码后的CAN帧
     */
    canfd_frame encodeBroadcastEnable(const MotorEnableFlag flags[MAX_MOTOR_COUNT], uint8_t count);

    /**
     * @brief 编码广播全节点控制帧 (一次性控制所有电机)
     * @param motor_data 6个电机的控制数据数组
     * @param count 电机数量 (1-6)
     * @return 编码后的CAN帧
     */
    canfd_frame encodeBroadcastControlAll(const MotorControlData motor_data[MAX_MOTOR_COUNT], uint8_t count);

    // ========== 协议解码接口 ==========

    /**
     * @brief 解码反馈数据 (反馈格式保持不变)
     * @param can_id CAN ID (0x0300 + 电机ID)
     * @param data 反馈数据指针
     * @param dlc 数据长度
     * @param out_motor 输出的电机数据
     */
    void decodeFeedback(uint32_t can_id, const uint8_t* data, uint8_t dlc, MotorData& out_motor);

    // ========== 单电机控制接口 ==========

    /**
     * @brief 单电机控制
     * @param motor_id 电机ID (1-6)
     * @param enable 使能标志
     * @param mode 控制模式
     * @param pos 位置指令 (度)
     * @param vel 速度指令 (度/秒)
     * @param cur 电流指令 (安培)
     * @param kp Kp参数 (会自动×1000编码为uint8)
     * @param kd Kd参数 (会自动×1000编码为uint8)
     * @return 编码后的CAN帧 (CAN ID = motor_id)
     */
    canfd_frame encodeControl(uint8_t motor_id, raytron::EnableFlag enable,
                              raytron::ControlMode mode, float pos, float vel, float cur,
                              float kp = 0.0f, float kd = 0.0f);

    /**
     * @brief 单电机反馈请求
     * @param motor_id 电机ID (1-6)
     * @return 编码后的CAN帧 (CAN ID = 0x0200 + motor_id, 数据长度1, 数据0x00)
     */
    canfd_frame encodeRequestFeedback(uint8_t motor_id);

private:
    /**
     * @brief 将数据向量转换为canfd_frame
     */
    canfd_frame dataToFrame(const std::vector<uint8_t>& data, uint32_t can_id) const;
};

}; // namespace raytron

#endif // RAYTRON_MOTOR_CTRL_H
