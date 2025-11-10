#ifndef GRIPPER_CTRL_H
#define GRIPPER_CTRL_H

#include <cstdint>
#include <linux/can.h>

/**
 * @brief 夹爪协议转换器 - 无状态，只转换协议
 *
 * 仅负责将控制值编码为CAN帧、将CAN数据解码为夹爪状态
 */

namespace gripper
{

// ========== 夹爪状态结构体 ==========

/**
 * @struct GripperData
 * @brief 夹爪状态数据
 */
struct GripperData
{
    uint8_t fault_code;      // 错误代码 (0x00-0x04)
    uint8_t state;           // 当前状态 (0x00-0x03)
    uint8_t position;        // 当前位置 (0-255)
    uint8_t velocity;        // 当前速度 (0-255)
    uint8_t force;           // 当前力矩 (0-255)
    uint8_t reserved[3];     // 保留字段
};

// ========== 夹爪故障码 ==========
enum FaultCode : uint8_t
{
    FAULT_NO_ERROR         = 0x00,  // 无故障
    FAULT_OVER_TEMP        = 0x01,  // 过温警报
    FAULT_OVER_SPEED       = 0x02,  // 超速警报
    FAULT_INIT_FAILED      = 0x03,  // 初始化故障警报
    FAULT_LIMIT_CHECK      = 0x04   // 超限检测警报
};

// ========== 夹爪状态码 ==========
enum GripperState : uint8_t
{
    STATE_TARGET_REACHED   = 0x00,  // 已达到目标位置
    STATE_MOVING           = 0x01,  // 夹爪移动中
    STATE_JAM              = 0x02,  // 夹爪堵转
    STATE_OBJECT_DROPPED   = 0x03   // 物体掉落
};

// ========== 夹爪协议转换器 ==========

class GripperProtocolConverter
{
public:
    /**
     * @brief 默认构造函数
     */
    GripperProtocolConverter() = default;

    /**
     * @brief 析构函数
     */
    ~GripperProtocolConverter() = default;

    // ========== 协议编码接口 ==========

    /**
     * @brief 编码夹爪控制命令为CAN帧
     * @param gripper_id 夹爪ID (can_node_id)
     * @param pos_cmd 目标位置 (0-255, 0=夹紧, 255=完全张开)
     * @param force_cmd 目标力矩 (0-255, 255=最大力矩)
     * @param vel_cmd 目标速度 (0-255, 255=最大速度)
     * @param acc_cmd 目标加速度 (0-255, 255=最大加速度)
     * @param dec_cmd 目标减速度 (0-255, 255=最大减速度)
     * @return 编码后的CAN帧
     */
    canfd_frame encodeControl(uint8_t gripper_id,
                              uint8_t pos_cmd,
                              uint8_t force_cmd,
                              uint8_t vel_cmd,
                              uint8_t acc_cmd,
                              uint8_t dec_cmd);

    /**
     * @brief 编码简化的夹爪控制命令（只指定位置和力矩）
     * @param gripper_id 夹爪ID
     * @param pos_cmd 目标位置 (0-255)
     * @param force_cmd 目标力矩 (0-255)
     * @return 编码后的CAN帧
     */
    canfd_frame encodeSimpleControl(uint8_t gripper_id,
                                     uint8_t pos_cmd,
                                     uint8_t force_cmd);

    /**
     * @brief 编码夹紧命令
     * @param gripper_id 夹爪ID
     * @param force_cmd 目标力矩 (0-255)
     * @return 编码后的CAN帧
     */
    canfd_frame encodeCloseGripper(uint8_t gripper_id, uint8_t force_cmd);

    /**
     * @brief 编码张开命令
     * @param gripper_id 夹爪ID
     * @return 编码后的CAN帧
     */
    canfd_frame encodeOpenGripper(uint8_t gripper_id);

    // ========== 协议解码接口 ==========

    /**
     * @brief 解码夹爪状态数据
     * @param can_id CAN ID
     * @param data 夹爪状态数据指针
     * @param dlc 数据长度
     * @param out_gripper 输出的夹爪状态数据
     */
    void decodeStatus(uint32_t can_id, const uint8_t* data, uint8_t dlc, GripperData& out_gripper);

    /**
     * @brief 获取故障码描述
     */
    const char* getFaultCodeDescription(uint8_t fault_code) const;

    /**
     * @brief 获取状态描述
     */
    const char* getStateDescription(uint8_t state) const;

private:
};

}; // namespace gripper

#endif // GRIPPER_CTRL_H
