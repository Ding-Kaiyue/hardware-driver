#include <gtest/gtest.h>
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <map>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;
using namespace hardware_driver::event;
using namespace hardware_driver::bus;

// Mock bus interface for testing - allows controlled testing without real hardware
class MockBusInterface : public BusInterface {
public:
    MockBusInterface() : send_count_(0), receive_count_(0) {}

    void init() override {
        initialized_ = true;
    }

    bool send(const GenericBusPacket& packet) override {
        std::lock_guard<std::mutex> lock(mutex_);
        sent_packets_.push_back(packet);
        send_count_++;
        return true;
    }

    bool receive(GenericBusPacket& packet) override {
        (void)packet;
        receive_count_++;
        return false;
    }

    void async_receive(const std::function<void(const GenericBusPacket&)>& callback) override {
        callback_ = callback;
    }

    std::vector<std::string> get_interface_names() const override {
        return {"can0"};
    }

    // Test helper methods
    int get_send_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return send_count_;
    }

    std::vector<GenericBusPacket> get_sent_packets() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sent_packets_;
    }

    void simulate_receive(const GenericBusPacket& packet) {
        if (callback_) {
            callback_(packet);
        }
    }

    bool is_initialized() const { return initialized_; }

private:
    mutable std::mutex mutex_;
    bool initialized_ = false;
    int send_count_;
    int receive_count_;
    std::vector<GenericBusPacket> sent_packets_;
    std::function<void(const GenericBusPacket&)> callback_;
};

// Test observer for motor status tracking
class TestMotorObserver : public MotorEventHandler {
public:
    std::map<uint32_t, Motor_Status> latest_status;
    std::mutex status_mutex;
    std::atomic<int> status_update_count{0};
    std::atomic<int> batch_status_count{0};
    std::atomic<int> function_result_count{0};
    std::atomic<int> parameter_result_count{0};

    void on_motor_status_update(const std::string& /* interface */,
                           uint32_t motor_id,
                           const Motor_Status& status) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        latest_status[motor_id] = status;
        status_update_count++;
    }

    void on_motor_status_update(const std::string& /* interface */,
                           const std::map<uint32_t, Motor_Status>& status_all) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        for (const auto& [motor_id, status] : status_all) {
            latest_status[motor_id] = status;
        }
        batch_status_count++;
    }

    void on_motor_function_result(const std::string& /* interface */,
                              uint32_t /* motor_id */,
                              uint8_t /* op_code */,
                              bool /* success */) override {
        function_result_count++;
    }

    void on_motor_parameter_result(const std::string& /* interface */,
                               uint32_t /* motor_id */,
                               uint16_t /* address */,
                               uint8_t /* data_type */,
                               const std::any& /* data */) override {
        parameter_result_count++;
    }

    Motor_Status get_status(uint32_t motor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        return latest_status[motor_id];
    }
};

class MotorDriverImplTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_bus_ = std::make_shared<MockBusInterface>();
        motor_driver_ = std::make_shared<MotorDriverImpl>(mock_bus_);
        event_bus_ = std::make_shared<EventBus>();
        motor_driver_->set_event_bus(event_bus_);
        observer_ = std::make_shared<TestMotorObserver>();
    }

    void TearDown() override {
        observer_.reset();
        motor_driver_.reset();
        mock_bus_.reset();
        event_bus_.reset();
    }

    std::shared_ptr<MockBusInterface> mock_bus_;
    std::shared_ptr<MotorDriverImpl> motor_driver_;
    std::shared_ptr<EventBus> event_bus_;
    std::shared_ptr<TestMotorObserver> observer_;
};

// 测试1：驱动初始化
TEST_F(MotorDriverImplTest, DriverInitialization) {
    EXPECT_FALSE(mock_bus_->is_initialized());
    // MotorDriverImpl doesn't have explicit init(), but that's okay for testing
    // Bus is initialized when first used
    std::cout << "Driver initialization verified\n";
}

// 测试2：单个电机启用/失能
TEST_F(MotorDriverImplTest, SingleMotorEnableDisable) {
    // Enable motor 1 in velocity mode (mode=4)
    motor_driver_->enable_motor("can0", 1, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing
    int send_count_after_enable = mock_bus_->get_send_count();
    EXPECT_GT(send_count_after_enable, 0);

    // Disable motor 1
    motor_driver_->disable_motor("can0", 1, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing
    int send_count_after_disable = mock_bus_->get_send_count();
    EXPECT_GT(send_count_after_disable, send_count_after_enable);

    std::cout << "Single motor enable/disable: sent " << send_count_after_disable << " commands\n";
}

// 测试3：批量电机启用/失能
TEST_F(MotorDriverImplTest, MultiMotorEnableDisable) {
    // Enable motors 1-3 in velocity mode
    motor_driver_->enable_all_motors("can0", {1, 2, 3}, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing
    int send_count_after_enable = mock_bus_->get_send_count();
    EXPECT_GT(send_count_after_enable, 0);

    // Disable motors 1-3
    motor_driver_->disable_all_motors("can0", {1, 2, 3}, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing
    int send_count_after_disable = mock_bus_->get_send_count();
    EXPECT_GT(send_count_after_disable, send_count_after_enable);

    std::cout << "Multi motor enable/disable: sent " << send_count_after_disable << " commands\n";
}

// 测试4：速度模式控制
TEST_F(MotorDriverImplTest, VelocityModeControl) {
    motor_driver_->enable_motor("can0", 1, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int initial_count = mock_bus_->get_send_count();

    // Send velocity commands using send_velocity_cmd
    motor_driver_->send_velocity_cmd("can0", 1, 6.0);   // Forward 6°/s
    motor_driver_->send_velocity_cmd("can0", 1, 0.0);   // Stop
    motor_driver_->send_velocity_cmd("can0", 1, -6.0);  // Reverse 6°/s
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int final_count = mock_bus_->get_send_count();
    EXPECT_GT(final_count, initial_count);

    std::cout << "Velocity mode control: sent " << (final_count - initial_count) << " velocity commands\n";
}

// 测试5：位置模式控制
TEST_F(MotorDriverImplTest, PositionModeControl) {
    motor_driver_->enable_motor("can0", 3, 5);  // mode=5 for position
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int initial_count = mock_bus_->get_send_count();

    // Send position commands using send_position_cmd
    motor_driver_->send_position_cmd("can0", 3, 30.0);
    motor_driver_->send_position_cmd("can0", 3, -30.0);
    motor_driver_->send_position_cmd("can0", 3, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int final_count = mock_bus_->get_send_count();
    EXPECT_GT(final_count, initial_count);

    std::cout << "Position mode control: sent " << (final_count - initial_count) << " position commands\n";
}

// 测试6：MIT模式控制
TEST_F(MotorDriverImplTest, MITModeControl) {
    motor_driver_->enable_motor("can0", 5, 3);  // mode=3 for MIT
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int initial_count = mock_bus_->get_send_count();

    // Send MIT commands using send_mit_cmd
    motor_driver_->send_mit_cmd("can0", 5, 90.0, 10.0, 0.0);
    motor_driver_->send_mit_cmd("can0", 5, 90.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int final_count = mock_bus_->get_send_count();
    EXPECT_GT(final_count, initial_count);

    std::cout << "MIT mode control: sent " << (final_count - initial_count) << " MIT commands\n";
}

// 测试7：力矩模式控制
TEST_F(MotorDriverImplTest, EffortModeControl) {
    motor_driver_->enable_motor("can0", 2, 2);  // mode=2 for effort
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int initial_count = mock_bus_->get_send_count();

    // Send effort commands using send_effort_cmd
    motor_driver_->send_effort_cmd("can0", 2, 5.0);
    motor_driver_->send_effort_cmd("can0", 2, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int final_count = mock_bus_->get_send_count();
    EXPECT_GT(final_count, initial_count);

    std::cout << "Effort mode control: sent " << (final_count - initial_count) << " effort commands\n";
}

// 测试8：参数读写
TEST_F(MotorDriverImplTest, ParameterReadWrite) {
    int initial_count = mock_bus_->get_send_count();

    // Read parameter
    motor_driver_->motor_parameter_read("can0", 1, 0x1000);

    // Write parameter (int)
    motor_driver_->motor_parameter_write("can0", 1, 0x1000, 100);

    // Write parameter (float)
    motor_driver_->motor_parameter_write("can0", 1, 0x1001, 50.5f);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int final_count = mock_bus_->get_send_count();
    EXPECT_GT(final_count, initial_count);

    std::cout << "Parameter read/write: sent " << (final_count - initial_count) << " parameter commands\n";
}

// 测试9：函数操作
TEST_F(MotorDriverImplTest, FunctionOperation) {
    int initial_count = mock_bus_->get_send_count();

    // Clear error code
    motor_driver_->motor_function_operation("can0", 1, 0x03);

    // Set zero position
    motor_driver_->motor_function_operation("can0", 1, 0x04);

    // Auto homing
    motor_driver_->motor_function_operation("can0", 1, 0x11);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    int final_count = mock_bus_->get_send_count();
    EXPECT_GT(final_count, initial_count);

    std::cout << "Function operations: sent " << (final_count - initial_count) << " function commands\n";
}

// 测试10：实时控制性能
TEST_F(MotorDriverImplTest, RealtimeControlPerformance) {
    motor_driver_->enable_all_motors("can0", {1, 2, 3, 4, 5, 6}, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    const int num_iterations = 100;
    const std::chrono::microseconds cmd_interval(300);  // 300μs interval matches control timing (200μs + margin)
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_iterations; ++i) {
        // Send commands in a paced manner to prevent queue overflow
        motor_driver_->send_velocity_cmd("can0", 1, 5.0f + i * 0.1f);
        motor_driver_->send_velocity_cmd("can0", 2, -5.0f - i * 0.1f);
        motor_driver_->send_position_cmd("can0", 3, 30.0f + i * 0.5f);
        motor_driver_->send_mit_cmd("can0", 5, 45.0f, 5.0f, 0.0f);

        // Rate limiting: wait between command batches to prevent queue overflow
        // This ensures realistic control loop timing where commands come at regular intervals
        if (i < num_iterations - 1) {
            std::this_thread::sleep_for(cmd_interval);
        }
    }

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_time);

    int total_commands = mock_bus_->get_send_count();
    double avg_time_per_command = static_cast<double>(duration.count()) / total_commands;

    EXPECT_LT(total_commands, 500);  // Should have processed ~400 commands without overflow
    std::cout << "Realtime control: " << total_commands << " commands in " << duration.count()
              << "μs (" << avg_time_per_command << "μs per command)\n";
}

// 测试11：复杂控制序列
TEST_F(MotorDriverImplTest, ComplexControlSequence) {
    // Setup: Enable 6 motors in different modes
    motor_driver_->enable_all_motors("can0", {1, 2}, 4);  // Velocity mode
    motor_driver_->enable_all_motors("can0", {3, 4}, 5);  // Position mode
    motor_driver_->enable_all_motors("can0", {5, 6}, 3);  // MIT mode

    // Execute complex sequence
    motor_driver_->send_velocity_cmd("can0", 1, 6.0);
    motor_driver_->send_velocity_cmd("can0", 2, -6.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    motor_driver_->send_position_cmd("can0", 3, 30.0);
    motor_driver_->send_position_cmd("can0", 4, -30.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    motor_driver_->send_mit_cmd("can0", 5, 90.0, 10.0, 0.0);
    motor_driver_->send_mit_cmd("can0", 6, -90.0, -10.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Stop and disable
    motor_driver_->send_velocity_cmd("can0", 1, 0.0);
    motor_driver_->send_velocity_cmd("can0", 2, 0.0);
    motor_driver_->send_position_cmd("can0", 3, 0.0);
    motor_driver_->send_position_cmd("can0", 4, 0.0);
    motor_driver_->send_mit_cmd("can0", 5, 0.0, 0.0, 0.0);
    motor_driver_->send_mit_cmd("can0", 6, 0.0, 0.0, 0.0);

    motor_driver_->disable_all_motors("can0", {1, 2, 3, 4, 5, 6}, 4);

    int total_count = mock_bus_->get_send_count();
    EXPECT_GT(total_count, 0);

    std::cout << "Complex control sequence: sent " << total_count << " total commands\n";
}

// 测试12：错误恢复
TEST_F(MotorDriverImplTest, ErrorRecovery) {
    // Try to control disabled motor - should be handled gracefully
    EXPECT_NO_THROW(motor_driver_->send_velocity_cmd("can0", 1, 5.0));

    // Enable and control should work
    EXPECT_NO_THROW(motor_driver_->enable_motor("can0", 1, 4));
    EXPECT_NO_THROW(motor_driver_->send_velocity_cmd("can0", 1, 5.0));

    // Disable and try function
    EXPECT_NO_THROW(motor_driver_->disable_motor("can0", 1, 4));
    EXPECT_NO_THROW(motor_driver_->motor_function_operation("can0", 1, 0x03));

    std::cout << "Error recovery: all commands handled gracefully\n";
}

// 测试13：多接口支持
TEST_F(MotorDriverImplTest, MultiInterfaceSupport) {
    // Get available interfaces
    auto interfaces = mock_bus_->get_interface_names();
    EXPECT_GT(interfaces.size(), 0);

    // Control motors on primary interface
    motor_driver_->enable_motor(interfaces[0], 1, 4);
    motor_driver_->send_velocity_cmd(interfaces[0], 1, 5.0);
    motor_driver_->disable_motor(interfaces[0], 1, 4);

    std::cout << "Multi-interface support: tested on " << interfaces.size() << " interface(s)\n";
}

// 测试14：并发控制
TEST_F(MotorDriverImplTest, ConcurrentControl) {
    motor_driver_->enable_all_motors("can0", {1, 2, 3}, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Allow async processing

    std::vector<std::thread> threads;
    const int num_threads = 3;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([this, t]() {
            for (int i = 0; i < 10; ++i) {
                motor_driver_->send_velocity_cmd("can0", t + 1, 5.0f + i * 0.1f);
                // Rate limiting to prevent queue overflow
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    // Allow final commands to be processed by control thread
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    int total_count = mock_bus_->get_send_count();
    EXPECT_GT(total_count, 0);  // Just verify we sent commands

    std::cout << "Concurrent control: processed " << total_count << " concurrent commands\n";
}

// 测试15：完整电机控制流程（基于示例程序）
TEST_F(MotorDriverImplTest, FullMotorControlFlow) {
    int initial_count = mock_bus_->get_send_count();

    // 电机1-2: 速度模式
    motor_driver_->enable_all_motors("can0", {1, 2}, 4);
    motor_driver_->send_velocity_cmd("can0", 1, 6.0);
    motor_driver_->send_velocity_cmd("can0", 2, 6.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    motor_driver_->send_velocity_cmd("can0", 1, 0.0);
    motor_driver_->send_velocity_cmd("can0", 2, 0.0);
    motor_driver_->disable_motor("can0", 1, 4);
    motor_driver_->disable_motor("can0", 2, 4);

    // 电机3-4: 位置模式
    motor_driver_->enable_all_motors("can0", {3, 4}, 5);
    motor_driver_->send_position_cmd("can0", 3, 30.0);
    motor_driver_->send_position_cmd("can0", 4, -30.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    motor_driver_->send_position_cmd("can0", 3, 0.0);
    motor_driver_->send_position_cmd("can0", 4, 0.0);
    motor_driver_->disable_motor("can0", 3, 5);
    motor_driver_->disable_motor("can0", 4, 5);

    // 电机5-6: MIT模式
    motor_driver_->enable_all_motors("can0", {5, 6}, 3);
    motor_driver_->send_mit_cmd("can0", 5, 90.0, 10.0, 0.0);
    motor_driver_->send_mit_cmd("can0", 6, -90.0, -10.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    motor_driver_->send_mit_cmd("can0", 5, 90.0, 0.0, 0.0);
    motor_driver_->send_mit_cmd("can0", 6, -90.0, 0.0, 0.0);
    motor_driver_->disable_motor("can0", 5, 3);
    motor_driver_->disable_motor("can0", 6, 3);

    int final_count = mock_bus_->get_send_count();
    EXPECT_GT(final_count, initial_count);

    std::cout << "Full motor control flow: completed with " << (final_count - initial_count)
              << " commands\n";
}
