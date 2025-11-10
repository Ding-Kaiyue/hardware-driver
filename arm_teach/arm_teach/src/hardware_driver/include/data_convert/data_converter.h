#ifndef DATA_CONVERTER_H
#define DATA_CONVERTER_H

#include <cstdint>
#include <cstring>
#include <vector>

/*
 * @brief 数据转换辅助函数库
 *
 * 提供CAN通信中常用的数据类型转换功能
 * 独立于具体的电机协议，可被多个协议使用
 */
namespace DataConverter
{
    // ========== Union for type conversion ==========

    /**
     * @brief Float/UInt32 conversion union
     * 用于在float和uint32之间进行类型转换
     */
    union FloatUintConverter
    {
        uint32_t u;
        float f;
    };

    // ========== 浮点数和整数转换 ==========

    /**
     * @brief 将4字节数据转换为浮点数
     * @param data 4字节数组 [低字节->高字节]
     * @return 转换后的浮点数
     */
    static inline float uint8Array_to_float(const uint8_t data[4])
    {
        uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                            (static_cast<uint32_t>(data[2]) << 16) |
                            (static_cast<uint32_t>(data[1]) << 8)  |
                            static_cast<uint32_t>(data[0]);
        float result;
        memcpy(&result, &combined, sizeof(result));
        return result;
    }

    /**
     * @brief 将浮点数转换为4字节数据
     * @param value 浮点数
     * @param data 输出缓冲区 [低字节->高字节]
     */
    static inline void float_to_uint8Array(float value, uint8_t data[4])
    {
        uint32_t uint32_val;
        memcpy(&uint32_val, &value, sizeof(value));
        data[0] = (uint32_val >> 0) & 0xFF;
        data[1] = (uint32_val >> 8) & 0xFF;
        data[2] = (uint32_val >> 16) & 0xFF;
        data[3] = (uint32_val >> 24) & 0xFF;
    }

    /**
     * @brief 将32位整数转换为4字节数据
     * @param value 32位整数
     * @param data 输出缓冲区 [低字节->高字节]
     */
    static inline void int32_to_uint8Array(int32_t value, uint8_t data[4])
    {
        data[0] = (value >> 0) & 0xFF;
        data[1] = (value >> 8) & 0xFF;
        data[2] = (value >> 16) & 0xFF;
        data[3] = (value >> 24) & 0xFF;
    }

    /**
     * @brief 将4字节数据转换为32位整数
     * @param data 4字节数组 [低字节->高字节]
     * @return 转换后的32位整数
     */
    static inline int32_t uint8Array_to_int32(const uint8_t data[4])
    {
        int32_t result = (static_cast<int32_t>(data[3]) << 24) |
                         (static_cast<int32_t>(data[2]) << 16) |
                         (static_cast<int32_t>(data[1]) << 8)  |
                         static_cast<int32_t>(data[0]);
        return result;
    }

    // ========== 范围映射转换 ==========

    /**
     * @brief 将无符号整数按范围映射到浮点数
     * @param x 输入的无符号整数
     * @param xmin 对应的浮点数最小值
     * @param xmax 对应的浮点数最大值
     * @param bits 位宽
     * @return 映射后的浮点数
     */
    static inline float uint_to_float_mapped(uint16_t x, float xmin, float xmax, uint8_t bits)
    {
        float span = xmax - xmin;
        float data_norm = float(x) / ((1 << bits) - 1);
        float data = data_norm * span + xmin;
        return data;
    }

    /**
     * @brief 将浮点数按范围映射到无符号整数
     * @param x 输入的浮点数
     * @param xmin 对应的浮点数最小值
     * @param xmax 对应的浮点数最大值
     * @param bits 位宽
     * @return 映射后的无符号整数
     */
    static inline uint16_t float_to_uint_mapped(float x, float xmin, float xmax, uint8_t bits)
    {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;
        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    }

    // ========== 字节操作 ==========

    /**
     * @brief 合并两个字节为16位整数 (小端格式)
     * @param low 低字节
     * @param high 高字节
     * @return 合并后的16位整数
     */
    static inline uint16_t bytes_to_uint16(uint8_t low, uint8_t high)
    {
        return (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    }

    /**
     * @brief 将16位整数分解为两个字节 (小端格式)
     * @param value 16位整数
     * @param low 输出低字节
     * @param high 输出高字节
     */
    static inline void uint16_to_bytes(uint16_t value, uint8_t& low, uint8_t& high)
    {
        low = value & 0xFF;
        high = (value >> 8) & 0xFF;
    }

    /**
     * @brief 创建CAN数据帧 (用指定字节填充)
     * @param fill_byte 填充字节值
     * @param data_len 数据长度 (1-8)
     * @return 填充后的数据向量
     */
    static inline std::vector<uint8_t> create_can_frame(uint8_t fill_byte = 0, int data_len = 8)
    {
        return std::vector<uint8_t>(data_len, fill_byte);
    }

    // ========== 大端格式转换 ==========

    /**
     * @brief 将4字节数据转换为浮点数 (大端格式)
     * @param data 4字节数组 [高字节->低字节]
     * @return 转换后的浮点数
     */
    static inline float uint8Array_to_float_be(const uint8_t data[4])
    {
        uint32_t combined = (static_cast<uint32_t>(data[0]) << 24) |
                            (static_cast<uint32_t>(data[1]) << 16) |
                            (static_cast<uint32_t>(data[2]) << 8)  |
                            static_cast<uint32_t>(data[3]);
        float result;
        memcpy(&result, &combined, sizeof(result));
        return result;
    }

    /**
     * @brief 将浮点数转换为4字节数据 (大端格式)
     * @param value 浮点数
     * @param data 输出缓冲区 [高字节->低字节]
     */
    static inline void float_to_uint8Array_be(float value, uint8_t data[4])
    {
        uint32_t uint32_val;
        memcpy(&uint32_val, &value, sizeof(value));
        data[0] = (uint32_val >> 24) & 0xFF;
        data[1] = (uint32_val >> 16) & 0xFF;
        data[2] = (uint32_val >> 8) & 0xFF;
        data[3] = (uint32_val >> 0) & 0xFF;
    }

    /**
     * @brief 将32位整数转换为4字节数据 (大端格式)
     * @param value 32位整数
     * @param data 输出缓冲区 [高字节->低字节]
     */
    static inline void int32_to_uint8Array_be(int32_t value, uint8_t data[4])
    {
        data[0] = (value >> 24) & 0xFF;
        data[1] = (value >> 16) & 0xFF;
        data[2] = (value >> 8) & 0xFF;
        data[3] = (value >> 0) & 0xFF;
    }

    /**
     * @brief 将4字节数据转换为32位整数 (大端格式)
     * @param data 4字节数组 [高字节->低字节]
     * @return 转换后的32位整数
     */
    static inline int32_t uint8Array_to_int32_be(const uint8_t data[4])
    {
        int32_t result = (static_cast<int32_t>(data[0]) << 24) |
                         (static_cast<int32_t>(data[1]) << 16) |
                         (static_cast<int32_t>(data[2]) << 8)  |
                         static_cast<int32_t>(data[3]);
        return result;
    }

    /**
     * @brief 合并两个字节为16位整数 (大端格式)
     * @param high 高字节
     * @param low 低字节
     * @return 合并后的16位整数
     */
    static inline uint16_t bytes_to_uint16_be(uint8_t high, uint8_t low)
    {
        return (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    }

    /**
     * @brief 将16位整数分解为两个字节 (大端格式)
     * @param value 16位整数
     * @param high 输出高字节
     * @param low 输出低字节
     */
    static inline void uint16_to_bytes_be(uint16_t value, uint8_t& high, uint8_t& low)
    {
        high = (value >> 8) & 0xFF;
        low = value & 0xFF;
    }

    // ========== 大端格式直接组装函数 ==========

    /**
     * @brief 从4个字节直接组装为浮点数 (大端格式)
     * @param b3 最高字节 (MSB)
     * @param b2 次高字节
     * @param b1 次低字节
     * @param b0 最低字节 (LSB)
     * @return 转换后的浮点数
     */
    static inline float assemble_float_be(uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0)
    {
        FloatUintConverter converter;
        converter.u = (static_cast<uint32_t>(b3) << 24) |
                      (static_cast<uint32_t>(b2) << 16) |
                      (static_cast<uint32_t>(b1) << 8) |
                      static_cast<uint32_t>(b0);
        return converter.f;
    }

    /**
     * @brief 从4个字节直接组装为32位整数 (大端格式)
     * @param b3 最高字节 (MSB)
     * @param b2 次高字节
     * @param b1 次低字节
     * @param b0 最低字节 (LSB)
     * @return 转换后的32位整数
     */
    static inline int32_t assemble_int32_be(uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0)
    {
        return (static_cast<int32_t>(b3) << 24) |
               (static_cast<int32_t>(b2) << 16) |
               (static_cast<int32_t>(b1) << 8) |
               static_cast<int32_t>(b0);
    }

    /**
     * @brief 从4个字节直接组装为32位无符号整数 (大端格式)
     * @param b3 最高字节 (MSB)
     * @param b2 次高字节
     * @param b1 次低字节
     * @param b0 最低字节 (LSB)
     * @return 转换后的32位无符号整数
     */
    static inline uint32_t assemble_uint32_be(uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0)
    {
        return (static_cast<uint32_t>(b3) << 24) |
               (static_cast<uint32_t>(b2) << 16) |
               (static_cast<uint32_t>(b1) << 8) |
               static_cast<uint32_t>(b0);
    }

    /**
     * @brief 从2个字节直接组装为16位整数 (大端格式)
     * @param b1 高字节 (MSB)
     * @param b0 低字节 (LSB)
     * @return 转换后的16位整数
     */
    static inline uint16_t assemble_uint16_be(uint8_t b1, uint8_t b0)
    {
        return (static_cast<uint16_t>(b1) << 8) | static_cast<uint16_t>(b0);
    }

    // ========== CAN 帧编码辅助函数 ==========

    /**
     * @brief 将浮点数编码到CAN帧中 (大端格式)
     * @param frame CAN帧数据向量
     * @param start_index 起始索引
     * @param value 浮点数值
     * @note frame 必须有足够的空间容纳4个字节
     */
    static inline void encodeFloatToFrame(std::vector<uint8_t>& frame,
                                         size_t start_index, float value)
    {
        uint8_t bytes[4];
        float_to_uint8Array_be(value, bytes);
        if (start_index + 3 < frame.size()) {
            frame[start_index]     = bytes[0];
            frame[start_index + 1] = bytes[1];
            frame[start_index + 2] = bytes[2];
            frame[start_index + 3] = bytes[3];
        }
    }

    /**
     * @brief 将32位整数编码到CAN帧中 (大端格式)
     * @param frame CAN帧数据向量
     * @param start_index 起始索引
     * @param value 32位整数值
     * @note frame 必须有足够的空间容纳4个字节
     */
    static inline void encodeInt32ToFrame(std::vector<uint8_t>& frame,
                                         size_t start_index, int32_t value)
    {
        uint8_t bytes[4];
        int32_to_uint8Array_be(value, bytes);
        if (start_index + 3 < frame.size()) {
            frame[start_index]     = bytes[0];
            frame[start_index + 1] = bytes[1];
            frame[start_index + 2] = bytes[2];
            frame[start_index + 3] = bytes[3];
        }
    }

    /**
     * @brief 将16位整数编码到CAN帧中 (大端格式)
     * @param frame CAN帧数据向量
     * @param start_index 起始索引
     * @param value 16位整数值
     * @note frame 必须有足够的空间容纳2个字节
     */
    static inline void encodeUint16ToFrame(std::vector<uint8_t>& frame,
                                          size_t start_index, uint16_t value)
    {
        if (start_index + 1 < frame.size()) {
            frame[start_index]     = (value >> 8) & 0xFF;
            frame[start_index + 1] = value & 0xFF;
        }
    }

    /**
     * @brief 将有符号16位整数编码到CAN帧中 (大端格式)
     * @param frame CAN帧数据向量
     * @param start_index 起始索引
     * @param value 有符号16位整数值
     * @note frame 必须有足够的空间容纳2个字节
     */
    static inline void encodeInt16ToFrame(std::vector<uint8_t>& frame,
                                          size_t start_index, int16_t value)
    {
        if (start_index + 1 < frame.size()) {
            frame[start_index]     = (value >> 8) & 0xFF;
            frame[start_index + 1] = value & 0xFF;
        }
    }

}

#endif // DATA_CONVERTER_H
