#include "hardware/usb2canfd_manager.h"
#include "protocol/raytron_motor_ctrl.h"
#include "protocol/gripper_ctrl.h"
#include <csignal>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstring>
#include <unordered_map>
#include <mutex>

// 全局控制标志
std::atomic<bool> running(true);

// 协议转换器全局指针（在main中初始化）
raytron::RaytronProtocolConverter* g_protocol_converter = nullptr;
gripper::GripperProtocolConverter* g_gripper_converter = nullptr;

// 电机数据映射（用于存储接收到的反馈数据）
std::unordered_map<uint8_t, raytron::MotorData> g_motor_data;
std::mutex g_motor_data_mutex;

// 夹爪数据映射（用于存储接收到的反馈数据）
std::unordered_map<uint8_t, gripper::GripperData> g_gripper_data;
std::mutex g_gripper_data_mutex;

// Ctrl+C 信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cout << "\n[INFO] Interrupt signal (" << signum << ") received. Shutting down...\n";
}

// CAN帧接收回调函数 - 解包并输出电机和夹爪数据
void onCanFrameReceived(const canfd_frame& frame) {
    // 提取实际的CAN ID（去掉扩展帧标志位）
    uint32_t actual_can_id = frame.can_id & 0x1FFFFFFF;  // 去掉CAN_EFF_FLAG (0x80000000)

    // 检查是否是Raytron电机反馈数据（RSP_FEEDBACK = 0x0300）
    if (g_protocol_converter && (actual_can_id & 0xFF00) == raytron::RSP_FEEDBACK) {
        // 解码反馈数据
        uint8_t motor_id = actual_can_id & 0xFF;
        raytron::MotorData motor_data{};
        g_protocol_converter->decodeFeedback(actual_can_id, frame.data, frame.len, motor_data);

        // 保存到全局映射
        {
            std::lock_guard<std::mutex> lock(g_motor_data_mutex);
            g_motor_data[motor_id] = motor_data;
        }

        // 输出解码后的电机数据
        std::cout << "[RX] 电机 #" << (int)motor_id << " 反馈数据:" << std::endl;
        std::cout << "     位置: " << std::fixed << std::setprecision(4) << motor_data.position << " rad" << std::endl;
        std::cout << "     速度: " << std::fixed << std::setprecision(4) << motor_data.velocity << " rad/s" << std::endl;
        std::cout << "     电流: " << std::fixed << std::setprecision(4) << motor_data.current << " A" << std::endl;
        std::cout << "     电压: " << motor_data.voltage / 10.0 << " V" << std::endl;
        std::cout << "     温度: " << motor_data.temperature / 10.0 << " °C" << std::endl;
        std::cout << "     错误码: 0x" << std::hex << motor_data.error_code << std::dec << std::endl;
        std::cout << std::endl;
    }
    // 检查是否是参数反馈（RSP_PARAM = 0x0400）
    else if (g_protocol_converter && (actual_can_id & 0xFF00) == raytron::RSP_PARAM) {
        uint8_t motor_id = actual_can_id & 0xFF;
        std::cout << "[RX] 电机 #" << (int)motor_id << " 参数反馈 (暂不解析)" << std::endl;
    }
    // 检查是否是夹爪反馈数据（假设使用0x0500作为夹爪反馈CAN ID）
    else if (g_gripper_converter && (actual_can_id & 0xFF00) == 0x0500) {
        uint8_t gripper_id = actual_can_id & 0xFF;
        gripper::GripperData gripper_data{};
        g_gripper_converter->decodeStatus(actual_can_id, frame.data, frame.len, gripper_data);

        // 保存到全局映射
        {
            std::lock_guard<std::mutex> lock(g_gripper_data_mutex);
            g_gripper_data[gripper_id] = gripper_data;
        }

        // 输出解码后的夹爪数据
        std::cout << "[RX] 夹爪 #" << (int)gripper_id << " 反馈数据:" << std::endl;
        std::cout << "     位置: " << (int)gripper_data.position << std::endl;
        std::cout << "     速度: " << (int)gripper_data.velocity << std::endl;
        std::cout << "     力矩: " << (int)gripper_data.force << std::endl;
        std::cout << "     状态: 0x" << std::hex << (int)gripper_data.state << std::dec
                  << " (" << g_gripper_converter->getStateDescription(gripper_data.state) << ")" << std::endl;
        std::cout << "     故障码: 0x" << std::hex << (int)gripper_data.fault_code << std::dec
                  << " (" << g_gripper_converter->getFaultCodeDescription(gripper_data.fault_code) << ")" << std::endl;
        std::cout << std::endl;
    }
    // 其他反馈类型
    else {
        std::cout << "[RX] 未知CAN ID: 0x" << std::hex << std::setfill('0') << std::setw(8)
                  << actual_can_id << std::dec << " | Length: " << (int)frame.len << std::endl;
    }
}

int main(int argc, char** argv)
{
    // 注册信号处理
    std::signal(SIGINT, signalHandler);

    try
    {
        // ========== USB2CANFD 初始化 ==========
        std::cout << "========== USB2CANFD CAN 通信演示 (使用硬件管理器) ==========" << std::endl;
        std::cout << "[INFO] 初始化 USB2CANFD 设备..." << std::endl;

        // 配置参数
        std::string device_path = "/dev/ttyACM0";  // 默认设备路径
        uint32_t nom_baud = 1000000;  // 标称波特率 1Mbps
        uint32_t dat_baud = 5000000;  // 数据波特率 5Mbps (CANFD)

        // 如果提供了命令行参数，使用该设备路径
        if (argc > 1) 
        {
            device_path = argv[1];
            std::cout << "[INFO] Using device path from argument: " << device_path << std::endl;
        } 
        else 
        {
            std::cout << "[INFO] Using default device path: " << device_path << std::endl;
        }

        // 创建USB2CANFD硬件管理器实例（通过设备路径自动获取SN）
        auto usb_manager = Usb2CanfdManager::createFromDevicePath(
            device_path,
            nom_baud,
            dat_baud
        );

        if (!usb_manager) {
            throw std::runtime_error("Failed to create USB2CANFD manager from device path");
        }

        // 检查硬件是否就绪
        if (!usb_manager->isReady()) {
            throw std::runtime_error("USB2CANFD hardware is not ready");
        }

        // 注册CAN帧接收回调
        usb_manager->setReceiveCallback(onCanFrameReceived);

        std::cout << "[INFO] USB2CANFD 设备初始化成功！" << std::endl;
        std::cout << "[INFO] 硬件标识: " << usb_manager->getHardwareIdentifier() << std::endl;
        std::cout << "[INFO] 开始收发数据（按 Ctrl+C 退出）" << std::endl << std::endl;

        // ========== 协议转换器初始化 ==========
        raytron::RaytronProtocolConverter protocol_converter;
        g_protocol_converter = &protocol_converter;  // 设置全局指针供回调函数使用

        gripper::GripperProtocolConverter gripper_converter;
        g_gripper_converter = &gripper_converter;  // 设置全局指针供回调函数使用

        // 电机ID配置（可根据需要修改）
        std::vector<uint8_t> motor_ids = {1, 2};  // 控制电机1和电机2

        // 夹爪ID配置（可根据需要修改）
        std::vector<uint8_t> gripper_ids = {7};  // 控制夹爪1

        std::cout << "[INFO] 已配置 " << motor_ids.size() << " 个电机" << std::endl;
        for (uint8_t id : motor_ids) {
            std::cout << "       - 电机 ID: " << (int)id << std::endl;
        }
        std::cout << "[INFO] 已配置 " << gripper_ids.size() << " 个夹爪" << std::endl;
        for (uint8_t id : gripper_ids) {
            std::cout << "       - 夹爪 ID: " << (int)id << std::endl;
        }
        std::cout << std::endl;

        // ========== 发送和接收循环 ==========
        auto start_time = std::chrono::steady_clock::now();
        int packet_count = 0;
        int tx_count = 0;
        int rx_count = 0;
        size_t motor_index = 0;
        size_t gripper_index = 0;
        int gripper_command_cycle = 0;  // 0=closeGripper, 1=openGripper, 2=control

        while (running)
        {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count();

            // 每100ms发送一次反馈请求或控制命令
            static int last_tx_time = -100;
            if (elapsed_ms - last_tx_time >= 100)
            {
                // 轮流请求各个电机的反馈
                uint8_t motor_id = motor_ids[motor_index];
                motor_index = (motor_index + 1) % motor_ids.size();

                // 编码反馈请求为CAN帧
                canfd_frame feedback_request = protocol_converter.encodeRequestFeedback(motor_id);

                // 发送反馈请求
                if (usb_manager->sendFrame(feedback_request)) {
                    std::cout << "[TX] 请求电机 #" << (int)motor_id << " 反馈 | "
                              << "CAN ID: 0x" << std::hex << std::setfill('0') << std::setw(8)
                              << feedback_request.can_id << std::dec
                              << " | Packet: " << packet_count << " | Data: ";
                    for (int i = 0; i < feedback_request.len; i++) {
                        std::cout << std::hex << std::setfill('0') << std::setw(2)
                                  << (int)feedback_request.data[i] << " ";
                    }
                    std::cout << std::dec << std::endl;

                    tx_count++;
                } else {
                    std::cerr << "[ERROR] Failed to send feedback request for motor " << (int)motor_id << std::endl;
                }

                // 每300ms交替发送夹爪控制命令
                static int last_gripper_tx_time = -300;
                if (elapsed_ms - last_gripper_tx_time >= 300 && !gripper_ids.empty())
                {
                    uint8_t gripper_id = gripper_ids[gripper_index];
                    gripper_index = (gripper_index + 1) % gripper_ids.size();

                    canfd_frame gripper_frame{};

                    // 循环发送不同的夹爪命令
                    switch (gripper_command_cycle % 3) {
                        case 0:  // 夹紧命令
                            gripper_frame = gripper_converter.encodeCloseGripper(gripper_id, 200);
                            std::cout << "[TX] 夹爪 #" << (int)gripper_id << " 夹紧命令 | ";
                            break;
                        case 1:  // 张开命令
                            gripper_frame = gripper_converter.encodeOpenGripper(gripper_id);
                            std::cout << "[TX] 夹爪 #" << (int)gripper_id << " 张开命令 | ";
                            break;
                        case 2:  // 控制命令（位置128，力矩150）
                            gripper_frame = gripper_converter.encodeSimpleControl(gripper_id, 128, 150);
                            std::cout << "[TX] 夹爪 #" << (int)gripper_id << " 控制命令(位置128, 力矩150) | ";
                            break;
                    }

                    gripper_command_cycle++;

                    if (usb_manager->sendFrame(gripper_frame)) {
                        std::cout << "CAN ID: 0x" << std::hex << std::setfill('0') << std::setw(8)
                                  << gripper_frame.can_id << std::dec << " | Data: ";
                        for (int i = 0; i < gripper_frame.len; i++) {
                            std::cout << std::hex << std::setfill('0') << std::setw(2)
                                      << (int)gripper_frame.data[i] << " ";
                        }
                        std::cout << std::dec << std::endl;
                        tx_count++;
                    } else {
                        std::cerr << "[ERROR] Failed to send gripper command for gripper " << (int)gripper_id << std::endl;
                    }

                    last_gripper_tx_time = elapsed_ms;
                }

                packet_count++;
                last_tx_time = elapsed_ms;
            }

            // 100ms延迟
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // ========== 清理工作 ==========
        std::cout << "\n[INFO] 正在停止USB2CANFD通信..." << std::endl;
        std::cout << "[STATS] 总发送数据包数: " << tx_count << std::endl;
        std::cout << "[STATS] 总接收数据包数: " << rx_count << std::endl;
        std::cout << "[INFO] 程序正常退出。" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
