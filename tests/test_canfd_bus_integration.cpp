#include <gtest/gtest.h>
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include "hardware_driver/interface/robot_hardware.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;
using namespace hardware_driver::bus;
using namespace hardware_driver::event;

// 硬件环境检测函数
bool has_can_hardware() {
    // 检查是否有CAN接口
    return system("ip link show can0 >/dev/null 2>&1") == 0;
}

// 测试用的电机事件处理器
class TestMotorEventHandler : public MotorEventHandler {
public:
    std::atomic<int> status_received_count{0};
    std::atomic<int> batch_received_count{0};
    std::map<uint32_t, Motor_Status> latest_status;
    std::mutex status_mutex;

    void on_motor_status_update(const std::string& /* interface */, uint32_t motor_id,
                            const Motor_Status& status) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        latest_status[motor_id] = status;
        status_received_count++;
    }

    void on_motor_status_update(const std::string& /* interface */,
                            const std::map<uint32_t, Motor_Status>& status_all) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        batch_received_count++;
        for (const auto& [motor_id, status] : status_all) {
            latest_status[motor_id] = status;
        }
    }

    void on_motor_function_result(const std::string& /* interface */, uint32_t /* motor_id */,
                               uint8_t /* op_code */, bool /* success */) override {}

    void on_motor_parameter_result(const std::string& /* interface */, uint32_t /* motor_id */,
                                uint16_t /* address */, uint8_t /* data_type */, const std::any& /* data */) override {}

    Motor_Status get_motor_status(uint32_t motor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        return latest_status[motor_id];
    }
};

class CanFdBusIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        has_hardware_ = has_can_hardware();

        try {
            // 创建CanFdBus实例
            std::vector<std::string> interfaces = {"can0"};
            canfd_bus_ = std::make_shared<CanFdBus>(interfaces);
            motor_driver_ = std::make_shared<MotorDriverImpl>(canfd_bus_);

            // 创建事件总线
            event_bus_ = std::make_shared<EventBus>();
            motor_driver_->set_event_bus(event_bus_);

            // 创建事件处理器
            event_handler_ = std::make_shared<TestMotorEventHandler>();

            // 配置6个电机
            std::map<std::string, std::vector<uint32_t>> config = {
                {"can0", {1, 2, 3, 4, 5, 6}}
            };

            // 使用事件总线创建RobotHardware
            robot_hardware_ = std::make_unique<RobotHardware>(motor_driver_, config, event_bus_, event_handler_);

            if (has_hardware_) {
                std::cout << "Integration tests running with real CAN hardware" << std::endl;
            } else {
                std::cout << "Integration tests running in simulation mode (no CAN hardware)" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "CAN initialization failed (expected without hardware): " << e.what() << std::endl;
            canfd_bus_ = nullptr;
            motor_driver_ = nullptr;
            robot_hardware_ = nullptr;
            event_bus_ = nullptr;
        }
    }

    void TearDown() override {
        event_handler_.reset();
        robot_hardware_.reset();
        motor_driver_.reset();
        canfd_bus_.reset();
        event_bus_.reset();
    }

    std::shared_ptr<CanFdBus> canfd_bus_;
    std::shared_ptr<MotorDriverImpl> motor_driver_;
    std::unique_ptr<RobotHardware> robot_hardware_;
    std::shared_ptr<EventBus> event_bus_;
    std::shared_ptr<TestMotorEventHandler> event_handler_;
    bool has_hardware_ = false;
};

// 测试1：验证6个电机的配置
TEST_F(CanFdBusIntegrationTest, MotorConfigurationValid) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    // 验证能够访问所有6个电机
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));
    std::cout << "All 6 motors configured and accessible" << std::endl;
}

// 测试2：电机1和2速度模式控制（基于示例程序验证）
TEST_F(CanFdBusIntegrationTest, Motors12VelocityModeControl) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    uint8_t velocity_mode = 4;

    // 失能所有电机
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 启用电机1和2的速度模式
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1, 2}, velocity_mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 正转6度/秒
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 1, 6.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 2, 6.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 停止
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 1, 0.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 2, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 反转6度/秒
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 1, -6.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 2, -6.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 停止并失能
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 1, 0.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 2, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 1, velocity_mode));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 2, velocity_mode));

    std::cout << "Motors 1-2 velocity mode control test passed" << std::endl;
}

// 测试3：电机3和4位置模式控制（基于示例程序验证）
TEST_F(CanFdBusIntegrationTest, Motors34PositionModeControl) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    uint8_t position_mode = 5;

    // 失能所有电机
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 启用电机3和4的位置模式
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {3, 4}, position_mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 运动到30度和-30度
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 3, 30.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 4, -30.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 运动到-30度和30度
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 3, -30.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 4, 30.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 回到零位置
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 3, 0.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 4, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 失能
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 3, position_mode));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 4, position_mode));

    std::cout << "Motors 3-4 position mode control test passed" << std::endl;
}

// 测试4：电机5和6 MIT模式控制（基于示例程序验证）
TEST_F(CanFdBusIntegrationTest, Motors56MITModeControl) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    uint8_t mit_mode = 3;

    // 失能所有电机
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 启用电机5和6的MIT模式
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {5, 6}, mit_mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 运动到90度和-90度，速度10和-10度/秒
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_mit_mode("can0", 5, 90.0, 10.0, 0.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_mit_mode("can0", 6, -90.0, -10.0, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 停止（保持位置，速度为0）
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_mit_mode("can0", 5, 90.0, 0.0, 0.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_mit_mode("can0", 6, -90.0, 0.0, 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 失能
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 5, mit_mode));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 6, mit_mode));

    std::cout << "Motors 5-6 MIT mode control test passed" << std::endl;
}

// 测试5：验证电机模式的正确选择性失能
TEST_F(CanFdBusIntegrationTest, SelectiveMotorEnableDisable) {
    if (!robot_hardware_ || !motor_driver_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    // 这个测试验证了新的模式追踪机制
    // 启用电机1和2（速度模式）
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1, 2}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 启用电机3和4（位置模式）
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {3, 4}, 5));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 只失能电机1和2，电机3和4应该保持其位置模式
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 验证电机3和4的位置命令仍然有效
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 3, 30.0));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 4, -30.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 清理：失能所有电机
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));

    std::cout << "Selective motor enable/disable test passed" << std::endl;
}

// 测试6：并发访问多个电机
TEST_F(CanFdBusIntegrationTest, ConcurrentMotorAccess) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    // 失能所有电机
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 启用所有电机（但用不同的模式）
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1, 2}, 4));  // 速度
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {3, 4}, 5));  // 位置
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {5, 6}, 3));  // MIT
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 并发发送命令
    std::vector<std::thread> threads;

    // 速度模式线程
    threads.emplace_back([this]() {
        EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 1, 3.0));
        EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 2, 5.0));
    });

    // 位置模式线程
    threads.emplace_back([this]() {
        EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 3, 45.0));
        EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 4, -45.0));
    });

    // MIT模式线程
    threads.emplace_back([this]() {
        EXPECT_NO_THROW(robot_hardware_->control_motor_in_mit_mode("can0", 5, 60.0, 5.0, 0.0));
        EXPECT_NO_THROW(robot_hardware_->control_motor_in_mit_mode("can0", 6, -60.0, -5.0, 0.0));
    });

    // 等待所有线程完成
    for (auto& thread : threads) {
        thread.join();
    }

    // 清理
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));

    std::cout << "Concurrent motor access test passed" << std::endl;
}

// 测试7：事件总线事件传递
TEST_F(CanFdBusIntegrationTest, EventBusEventDelivery) {
    if (!robot_hardware_ || !event_handler_ || !event_bus_) {
        GTEST_SKIP() << "Robot hardware or event handler not initialized";
    }

    // 发送一些命令，应该触发事件
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1}, 4));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 1, 5.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 验证事件总线工作
    auto stats = event_bus_->get_statistics();
    EXPECT_GE(stats.total_handlers, 1);

    std::cout << "Event bus test completed - Event handlers: " << stats.total_handlers << std::endl;
}

// 测试8：电机参数读写
TEST_F(CanFdBusIntegrationTest, MotorParameterReadWrite) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    // 测试参数读取
    EXPECT_NO_THROW(robot_hardware_->motor_parameter_read("can0", 1, 0x1000));

    // 测试参数写入（int值）
    EXPECT_NO_THROW(robot_hardware_->motor_parameter_write("can0", 1, 0x1000, 100));

    // 测试参数写入（float值）
    EXPECT_NO_THROW(robot_hardware_->motor_parameter_write("can0", 1, 0x1001, 50.5f));

    std::cout << "Motor parameter read/write test passed" << std::endl;
}

// 测试9：电机函数操作
TEST_F(CanFdBusIntegrationTest, MotorFunctionOperation) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    // 测试清除错误码
    EXPECT_NO_THROW(robot_hardware_->motor_function_operation("can0", 1, 0x03));

    // 测试设置零位
    EXPECT_NO_THROW(robot_hardware_->motor_function_operation("can0", 1, 0x04));

    // 测试自动寻零
    EXPECT_NO_THROW(robot_hardware_->motor_function_operation("can0", 1, 0x11));

    std::cout << "Motor function operation test passed" << std::endl;
}

// 测试10：多种控制模式的混合使用
TEST_F(CanFdBusIntegrationTest, MixedControlModes) {
    if (!robot_hardware_) {
        GTEST_SKIP() << "Robot hardware not initialized";
    }

    // 失能所有电机
    EXPECT_NO_THROW(robot_hardware_->disable_motors("can0", {1, 2, 3, 4, 5, 6}, 4));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 顺序测试所有控制模式
    // 速度模式
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1}, 4));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_velocity_mode("can0", 1, 5.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 1, 4));

    // 位置模式
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1}, 5));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_position_mode("can0", 1, 45.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 1, 5));

    // 力矩模式
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1}, 2));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_effort_mode("can0", 1, 2.5));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 1, 2));

    // MIT模式
    EXPECT_NO_THROW(robot_hardware_->enable_motors("can0", {1}, 3));
    EXPECT_NO_THROW(robot_hardware_->control_motor_in_mit_mode("can0", 1, 60.0, 5.0, 1.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_NO_THROW(robot_hardware_->disable_motor("can0", 1, 3));

    std::cout << "Mixed control modes test passed" << std::endl;
}
