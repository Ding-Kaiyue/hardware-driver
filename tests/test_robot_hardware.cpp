#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include "hardware_driver/interface/robot_hardware.hpp"
#include "hardware_driver/driver/motor_driver_interface.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"

// ==================== 类型别名简化 ====================
using MotorConfig = std::map<std::string, std::vector<uint32_t>>;

// ==================== Mock 电机驱动器 ====================

class MockMotorDriver : public hardware_driver::motor_driver::MotorDriverInterface {
public:
    MOCK_METHOD(void, enable_motor, (const std::string, const uint32_t, uint8_t), (override));
    MOCK_METHOD(void, enable_all_motors, (const std::string, std::vector<uint32_t>, uint8_t), (override));
    MOCK_METHOD(void, disable_motor, (const std::string, const uint32_t, uint8_t), (override));
    MOCK_METHOD(void, disable_all_motors, (const std::string, std::vector<uint32_t>, uint8_t), (override));
    MOCK_METHOD(void, send_position_cmd, (const std::string, const uint32_t, float, float, float), (override));
    MOCK_METHOD(void, send_velocity_cmd, (const std::string, const uint32_t, float, float, float), (override));
    MOCK_METHOD(void, send_effort_cmd, (const std::string, const uint32_t, float, float, float), (override));
    MOCK_METHOD(void, send_mit_cmd, (const std::string, const uint32_t, float, float, float, float, float), (override));
    MOCK_METHOD(void, send_control_cmd, (const std::string, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>), (override));
    MOCK_METHOD(void, motor_function_operation, (const std::string, const uint32_t, uint8_t), (override));
    MOCK_METHOD(void, motor_parameter_read, (const std::string, const uint32_t, uint16_t), (override));
    MOCK_METHOD(void, motor_parameter_write, (const std::string, const uint32_t, uint16_t, int32_t), (override));
    MOCK_METHOD(void, motor_parameter_write, (const std::string, const uint32_t, uint16_t, float), (override));
    MOCK_METHOD(void, start_update, (const std::string&, uint32_t, const std::string&), (override));

    // Implement array/all operations with default implementations
    void send_position_cmd_all(const std::string, const std::array<float, 6>&, const std::array<float, 6>&, const std::array<float, 6>&) override {}
    void send_velocity_cmd_all(const std::string, const std::array<float, 6>&, const std::array<float, 6>&, const std::array<float, 6>&) override {}
    void send_effort_cmd_all(const std::string, const std::array<float, 6>&, const std::array<float, 6>&, const std::array<float, 6>&) override {}
    void send_mit_cmd_all(const std::string, const std::array<float, 6>&, const std::array<float, 6>&, const std::array<float, 6>&, const std::array<float, 6>&, const std::array<float, 6>&) override {}
};

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;

// ==================== 电机状态观察者 ====================

class TestMotorStatusObserver : public hardware_driver::motor_driver::MotorStatusObserver {
public:
    std::map<std::string, std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>> latest_status;
    std::mutex status_mutex;
    std::atomic<int> status_received_count{0};
    std::atomic<int> batch_received_count{0};

    void on_motor_status_update(const std::string& interface,
                               uint32_t motor_id,
                               const hardware_driver::motor_driver::Motor_Status& status) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        latest_status[interface][motor_id] = status;
        status_received_count++;
    }

    void on_motor_status_update(const std::string& interface,
                               const std::map<uint32_t, hardware_driver::motor_driver::Motor_Status>& status_all) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        for (const auto& [motor_id, status] : status_all) {
            latest_status[interface][motor_id] = status;
        }
        batch_received_count++;
    }

    hardware_driver::motor_driver::Motor_Status get_motor_status(const std::string& interface, uint32_t motor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        if (latest_status.find(interface) != latest_status.end() &&
            latest_status[interface].find(motor_id) != latest_status[interface].end()) {
            return latest_status[interface][motor_id];
        }
        return hardware_driver::motor_driver::Motor_Status{};
    }
};

// ==================== RobotHardware 测试套件 ====================

class RobotHardwareTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_driver_ = std::make_shared<MockMotorDriver>();
        observer_ = std::make_shared<TestMotorStatusObserver>();

        // 配置电机
        interface_config_ = {
            {"can0", {1, 2, 3}},
            {"can1", {4, 5, 6}}
        };
    }

    void TearDown() override {
        mock_driver_.reset();
        observer_.reset();
    }

    std::shared_ptr<MockMotorDriver> mock_driver_;
    std::shared_ptr<TestMotorStatusObserver> observer_;
    MotorConfig interface_config_;
};

// ==================== 单个电机控制测试 ====================

TEST_F(RobotHardwareTest, ControlMotorInVelocityMode) {
    EXPECT_CALL(*mock_driver_, send_velocity_cmd("can0", 1, 5.0f, 0.0f, 0.0f))
        .Times(1);
    EXPECT_CALL(*mock_driver_, send_velocity_cmd("can0", 2, -3.5f, 0.1f, 0.05f))
        .Times(1);

    // 使用简单的 callback 构造函数（不使用观察者）
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.control_motor_in_velocity_mode("can0", 1, 5.0f);
    hw.control_motor_in_velocity_mode("can0", 2, -3.5f, 0.1f, 0.05f);
}

TEST_F(RobotHardwareTest, ControlMotorInPositionMode) {
    EXPECT_CALL(*mock_driver_, send_position_cmd("can0", 1, 45.0f, 0.0f, 0.0f))
        .Times(1);
    EXPECT_CALL(*mock_driver_, send_position_cmd("can1", 4, 90.0f, 0.2f, 0.1f))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.control_motor_in_position_mode("can0", 1, 45.0f);
    hw.control_motor_in_position_mode("can1", 4, 90.0f, 0.2f, 0.1f);
}

TEST_F(RobotHardwareTest, ControlMotorInEffortMode) {
    EXPECT_CALL(*mock_driver_, send_effort_cmd("can0", 1, 2.5f, 0.0f, 0.0f))
        .Times(1);
    EXPECT_CALL(*mock_driver_, send_effort_cmd("can0", 3, -1.5f, 0.15f, 0.075f))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.control_motor_in_effort_mode("can0", 1, 2.5f);
    hw.control_motor_in_effort_mode("can0", 3, -1.5f, 0.15f, 0.075f);
}

TEST_F(RobotHardwareTest, ControlMotorInMITMode) {
    EXPECT_CALL(*mock_driver_, send_mit_cmd("can0", 1, 30.0f, 2.0f, 0.5f, 0.1f, 0.05f))
        .Times(1);
    EXPECT_CALL(*mock_driver_, send_mit_cmd("can1", 5, 60.0f, 3.0f, 1.0f, 0.2f, 0.1f))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.control_motor_in_mit_mode("can0", 1, 30.0f, 2.0f, 0.5f, 0.1f, 0.05f);
    hw.control_motor_in_mit_mode("can1", 5, 60.0f, 3.0f, 1.0f, 0.2f, 0.1f);
}

// ==================== 电机启用/禁用测试 ====================

TEST_F(RobotHardwareTest, EnableSingleMotor) {
    EXPECT_CALL(*mock_driver_, enable_motor("can0", 1, 4))
        .Times(1);
    EXPECT_CALL(*mock_driver_, enable_motor("can0", 3, 5))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.enable_motor("can0", 1, 4);  // Velocity mode
    hw.enable_motor("can0", 3, 5);  // Position mode
}

TEST_F(RobotHardwareTest, DisableSingleMotor) {
    EXPECT_CALL(*mock_driver_, disable_motor("can0", 2, 4))
        .Times(1);
    EXPECT_CALL(*mock_driver_, disable_motor("can1", 5, 3))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.disable_motor("can0", 2, 4);
    hw.disable_motor("can1", 5, 3);
}

TEST_F(RobotHardwareTest, EnableMultipleMotors) {
    EXPECT_CALL(*mock_driver_, enable_all_motors("can0", std::vector<uint32_t>{1, 2, 3}, 4))
        .Times(1);
    EXPECT_CALL(*mock_driver_, enable_all_motors("can1", std::vector<uint32_t>{4, 5}, 5))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.enable_motors("can0", {1, 2, 3}, 4);
    hw.enable_motors("can1", {4, 5}, 5);
}

TEST_F(RobotHardwareTest, DisableMultipleMotors) {
    EXPECT_CALL(*mock_driver_, disable_all_motors("can0", std::vector<uint32_t>{1, 3}, 4))
        .Times(1);
    EXPECT_CALL(*mock_driver_, disable_all_motors("can1", std::vector<uint32_t>{5, 6}, 3))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.disable_motors("can0", {1, 3}, 4);
    hw.disable_motors("can1", {5, 6}, 3);
}

// ==================== 实时批量命令测试 ====================

TEST_F(RobotHardwareTest, SendRealtimeVelocityCommand) {
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    std::array<double, 6> velocities = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    bool result = hw.send_realtime_velocity_command("can0", velocities);
    EXPECT_TRUE(result);
}

TEST_F(RobotHardwareTest, SendRealtimePositionCommand) {
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    std::array<double, 6> positions = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
    bool result = hw.send_realtime_position_command("can0", positions);
    EXPECT_TRUE(result);
}

TEST_F(RobotHardwareTest, SendRealtimeEffortCommand) {
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    std::array<double, 6> efforts = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    bool result = hw.send_realtime_effort_command("can0", efforts);
    EXPECT_TRUE(result);
}

TEST_F(RobotHardwareTest, SendRealtimeMITCommand) {
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    std::array<double, 6> positions = {30.0, 40.0, 50.0, 60.0, 70.0, 80.0};
    std::array<double, 6> velocities = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    std::array<double, 6> efforts = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    std::array<double, 6> kps = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    std::array<double, 6> kds = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05};

    bool result = hw.send_realtime_mit_command("can0", positions, velocities, efforts, kps, kds);
    EXPECT_TRUE(result);
}

// ==================== 参数读写测试 ====================

TEST_F(RobotHardwareTest, MotorParameterRead) {
    EXPECT_CALL(*mock_driver_, motor_parameter_read("can0", 1, 0x0008))
        .Times(1);
    EXPECT_CALL(*mock_driver_, motor_parameter_read("can1", 5, 0x000A))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.motor_parameter_read("can0", 1, 0x0008);
    hw.motor_parameter_read("can1", 5, 0x000A);
}

TEST_F(RobotHardwareTest, MotorParameterWriteInt) {
    using ::testing::An;
    EXPECT_CALL(*mock_driver_, motor_parameter_write("can0", 1, 0x0008, An<int32_t>()))
        .Times(1);
    EXPECT_CALL(*mock_driver_, motor_parameter_write("can0", 3, 0x0009, An<int32_t>()))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.motor_parameter_write("can0", 1, 0x0008, static_cast<int32_t>(100000));
    hw.motor_parameter_write("can0", 3, 0x0009, static_cast<int32_t>(-50000));
}

TEST_F(RobotHardwareTest, MotorParameterWriteFloat) {
    using ::testing::An;
    EXPECT_CALL(*mock_driver_, motor_parameter_write("can0", 2, 0x000B, An<float>()))
        .Times(1);
    EXPECT_CALL(*mock_driver_, motor_parameter_write("can1", 4, 0x000C, An<float>()))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.motor_parameter_write("can0", 2, 0x000B, 3.14f);
    hw.motor_parameter_write("can1", 4, 0x000C, -2.71f);
}

// ==================== 函数操作测试 ====================

TEST_F(RobotHardwareTest, MotorFunctionOperation) {
    EXPECT_CALL(*mock_driver_, motor_function_operation("can0", 1, 0x01))  // PARAM_RESET
        .Times(1);
    EXPECT_CALL(*mock_driver_, motor_function_operation("can0", 2, 0x03))  // CLEAR_ERROR_CODE
        .Times(1);
    EXPECT_CALL(*mock_driver_, motor_function_operation("can1", 5, 0x11))  // MOTOR_FIND_ZERO_POS
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.motor_function_operation("can0", 1, 0x01);
    hw.motor_function_operation("can0", 2, 0x03);
    hw.motor_function_operation("can1", 5, 0x11);
}

TEST_F(RobotHardwareTest, ArmZeroPositionSet) {
    using ::testing::_;
    // 创建一个新的 mock driver 用于这个测试，避免与其他测试的期望冲突
    auto test_mock_driver = std::make_shared<MockMotorDriver>();

    // arm_zero_position_set 对每个电机调用两次 motor_function_operation
    // 一次发送 0x04 (MOTOR_ZERO_POS_SET), 一次发送 0x02
    // 所以3个电机 = 6次调用
    EXPECT_CALL(*test_mock_driver, motor_function_operation("can0", _, _))
        .Times(6);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(test_mock_driver, interface_config_, callback);

    hw.arm_zero_position_set("can0", {1, 2, 3});
}

// ==================== IAP固件更新测试 ====================

TEST_F(RobotHardwareTest, StartIAPUpdate) {
    EXPECT_CALL(*mock_driver_, start_update("can0", 1, "/path/to/firmware.bin"))
        .Times(1);
    EXPECT_CALL(*mock_driver_, start_update("can1", 5, "/path/to/another_firmware.hex"))
        .Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    hw.start_update("can0", 1, "/path/to/firmware.bin");
    hw.start_update("can1", 5, "/path/to/another_firmware.hex");
}

// ==================== 状态监控控制测试 ====================

TEST_F(RobotHardwareTest, PauseStatusMonitoring) {
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    // 测试暂停状态监控功能
    hw.pause_status_monitoring();
    // 验证没有异常抛出
    EXPECT_TRUE(true);
}

TEST_F(RobotHardwareTest, ResumeStatusMonitoring) {
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    // 测试恢复状态监控功能
    hw.resume_status_monitoring();
    // 验证没有异常抛出
    EXPECT_TRUE(true);
}

// ==================== 复杂场景测试 ====================

TEST_F(RobotHardwareTest, FullMotorControlSequence) {
    // 期望的调用顺序
    EXPECT_CALL(*mock_driver_, enable_motor("can0", 1, 4)).Times(1);
    EXPECT_CALL(*mock_driver_, send_velocity_cmd("can0", 1, 5.0f, 0.0f, 0.0f)).Times(1);
    EXPECT_CALL(*mock_driver_, disable_motor("can0", 1, 4)).Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    // 启用电机
    hw.enable_motor("can0", 1, 4);

    // 发送速度命令
    hw.control_motor_in_velocity_mode("can0", 1, 5.0f);

    // 禁用电机
    hw.disable_motor("can0", 1, 4);
}

TEST_F(RobotHardwareTest, MultiInterfaceControl) {
    // Can0 的操作
    EXPECT_CALL(*mock_driver_, enable_all_motors("can0", std::vector<uint32_t>{1, 2}, 4)).Times(1);
    EXPECT_CALL(*mock_driver_, send_velocity_cmd("can0", 1, 2.0f, 0.0f, 0.0f)).Times(1);
    EXPECT_CALL(*mock_driver_, send_velocity_cmd("can0", 2, -2.0f, 0.0f, 0.0f)).Times(1);

    // Can1 的操作
    EXPECT_CALL(*mock_driver_, enable_all_motors("can1", std::vector<uint32_t>{4, 5}, 5)).Times(1);
    EXPECT_CALL(*mock_driver_, send_position_cmd("can1", 4, 45.0f, 0.0f, 0.0f)).Times(1);
    EXPECT_CALL(*mock_driver_, send_position_cmd("can1", 5, 90.0f, 0.0f, 0.0f)).Times(1);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    // Can0 速度控制
    hw.enable_motors("can0", {1, 2}, 4);
    hw.control_motor_in_velocity_mode("can0", 1, 2.0f);
    hw.control_motor_in_velocity_mode("can0", 2, -2.0f);

    // Can1 位置控制
    hw.enable_motors("can1", {4, 5}, 5);
    hw.control_motor_in_position_mode("can1", 4, 45.0f);
    hw.control_motor_in_position_mode("can1", 5, 90.0f);
}

TEST_F(RobotHardwareTest, ParameterAndFunctionOperations) {
    using ::testing::An;
    // 参数读写
    EXPECT_CALL(*mock_driver_, motor_parameter_read("can0", 1, 0x0008)).Times(1);
    EXPECT_CALL(*mock_driver_, motor_parameter_write("can0", 1, 0x0008, An<int32_t>())).Times(1);
    EXPECT_CALL(*mock_driver_, motor_parameter_write("can0", 1, 0x0009, An<float>())).Times(1);

    // 函数操作
    EXPECT_CALL(*mock_driver_, motor_function_operation("can0", 1, 0x03)).Times(1);  // 清除错误码
    EXPECT_CALL(*mock_driver_, motor_function_operation("can0", 1, 0x04)).Times(1);  // 设置零位

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    // 参数操作
    hw.motor_parameter_read("can0", 1, 0x0008);
    hw.motor_parameter_write("can0", 1, 0x0008, static_cast<int32_t>(50000));
    hw.motor_parameter_write("can0", 1, 0x0009, 1.5f);

    // 函数操作
    hw.motor_function_operation("can0", 1, 0x03);
    hw.motor_function_operation("can0", 1, 0x04);
}

// ==================== 边界和错误情况测试 ====================

TEST_F(RobotHardwareTest, ControlWithVariousParameterCombinations) {
    // 测试各种参数组合
    EXPECT_CALL(*mock_driver_, send_velocity_cmd).Times(3);
    EXPECT_CALL(*mock_driver_, send_position_cmd).Times(3);
    EXPECT_CALL(*mock_driver_, send_effort_cmd).Times(3);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    // 测试正值
    hw.control_motor_in_velocity_mode("can0", 1, 10.0f);
    hw.control_motor_in_position_mode("can0", 1, 90.0f, 0.1f, 0.05f);
    hw.control_motor_in_effort_mode("can0", 1, 2.0f, 0.05f, 0.025f);

    // 测试负值
    hw.control_motor_in_velocity_mode("can0", 2, -10.0f);
    hw.control_motor_in_position_mode("can0", 2, -90.0f, 0.1f, 0.05f);
    hw.control_motor_in_effort_mode("can0", 2, -2.0f, 0.05f, 0.025f);

    // 测试零值
    hw.control_motor_in_velocity_mode("can0", 3, 0.0f);
    hw.control_motor_in_position_mode("can0", 3, 0.0f);
    hw.control_motor_in_effort_mode("can0", 3, 0.0f);
}

TEST_F(RobotHardwareTest, RealtimeCommandWithZeroValues) {
    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    std::array<double, 6> zeros = {};
    std::array<double, 6> ones = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    // Test realtime commands with zero and non-zero values
    bool vel_result = hw.send_realtime_velocity_command("can0", zeros);
    bool pos_result = hw.send_realtime_position_command("can0", ones);
    bool eff_result = hw.send_realtime_effort_command("can0", zeros);
    bool mit_result = hw.send_realtime_mit_command("can0", ones, zeros, ones);

    EXPECT_TRUE(vel_result);
    EXPECT_TRUE(pos_result);
    EXPECT_TRUE(eff_result);
    EXPECT_TRUE(mit_result);
}

// ==================== 多个电机同时控制 ====================

TEST_F(RobotHardwareTest, MultipleMotorsSequentialControl) {
    EXPECT_CALL(*mock_driver_, send_velocity_cmd).Times(6);
    EXPECT_CALL(*mock_driver_, send_position_cmd).Times(3);

    auto callback = [](const std::string&, uint32_t, const hardware_driver::motor_driver::Motor_Status&) {};
    RobotHardware hw(mock_driver_, interface_config_, callback);

    // 速度控制 - 6个电机
    for (int i = 1; i <= 3; ++i) {
        hw.control_motor_in_velocity_mode("can0", i, i * 2.0f);
    }
    for (int i = 4; i <= 6; ++i) {
        hw.control_motor_in_velocity_mode("can1", i, -(i - 3) * 2.0f);
    }

    // 位置控制 - 3个电机
    for (int i = 1; i <= 3; ++i) {
        hw.control_motor_in_position_mode("can0", i, i * 30.0f);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
