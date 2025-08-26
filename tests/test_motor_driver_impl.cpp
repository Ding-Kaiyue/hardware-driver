#include <gtest/gtest.h>
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <map>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;
using namespace hardware_driver::bus;

// Simple test bus interface for testing
class TestBusInterface : public BusInterface {
public:
    TestBusInterface() : send_called_(false) {}
    
    void init() override {}
    
    bool send(const GenericBusPacket& packet) override {
        last_packet_ = packet;
        send_called_ = true;
        return true;
    }
    
    bool receive(GenericBusPacket& packet) override {
        (void)packet; // 避免未使用参数警告
        return false;
    }
    
    void async_receive(const std::function<void(const GenericBusPacket&)>& callback) override {
        callback_ = callback;
    }
    
    std::vector<std::string> get_interface_names() const override {
        return {"can0", "can1"};
    }
    
    // Test helper methods
    bool was_send_called() const { return send_called_; }
    const GenericBusPacket& get_last_packet() const { return last_packet_; }
    void simulate_receive(const GenericBusPacket& packet) {
        if (callback_) {
            callback_(packet);
        }
    }
    
    // 检查是否有真实的CAN硬件
    static bool has_real_can_hardware() {
        // 检查是否有CAN接口文件
        return system("ls /sys/class/net/can* >/dev/null 2>&1") == 0;
    }
    
private:
    bool send_called_;
    GenericBusPacket last_packet_;
    std::function<void(const GenericBusPacket&)> callback_;
};

// 测试用的电机状态观察者
class TestMotorStatusObserver : public MotorStatusObserver {
public:
    std::map<std::string, std::map<uint32_t, Motor_Status>> status_map;
    std::mutex status_mutex;
    std::atomic<int> status_update_count{0};
    std::atomic<int> batch_status_update_count{0};
    std::atomic<int> function_result_count{0};
    std::atomic<int> parameter_result_count{0};
    
    void on_motor_status_update(const std::string& interface, 
                               uint32_t motor_id, 
                               const Motor_Status& status) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        status_map[interface][motor_id] = status;
        status_update_count++;
    }
    
    void on_motor_status_update(const std::string& interface,
                               const std::map<uint32_t, Motor_Status>& status_all) override {
        std::lock_guard<std::mutex> lock(status_mutex);
        for (const auto& [motor_id, status] : status_all) {
            status_map[interface][motor_id] = status;
        }
        batch_status_update_count++;
    }
    
    void on_motor_function_result(const std::string& /*interface*/,
                                 uint32_t /*motor_id*/,
                                 uint8_t /*op_code*/,
                                 bool /*success*/) override {
        function_result_count++;
    }
    
    void on_motor_parameter_result(const std::string& /*interface*/,
                                  uint32_t /*motor_id*/,
                                  uint16_t /*address*/,
                                  uint8_t /*data_type*/,
                                  const std::any& /*data*/) override {
        parameter_result_count++;
    }
    
    Motor_Status get_motor_status(const std::string& interface, uint32_t motor_id) {
        std::lock_guard<std::mutex> lock(status_mutex);
        if (status_map.find(interface) != status_map.end() &&
            status_map[interface].find(motor_id) != status_map[interface].end()) {
            return status_map[interface][motor_id];
        }
        // 返回默认状态如果没有找到
        return Motor_Status{};
    }
    
    void clear_status() {
        std::lock_guard<std::mutex> lock(status_mutex);
        status_map.clear();
        status_update_count = 0;
        batch_status_update_count = 0;
        function_result_count = 0;
        parameter_result_count = 0;
    }
};

class MotorDriverImplTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_bus_ = std::make_shared<TestBusInterface>();
        motor_driver_ = std::make_shared<MotorDriverImpl>(test_bus_);
        
        // 创建并注册状态观察者
        status_observer_ = std::make_shared<TestMotorStatusObserver>();
        motor_driver_->add_observer(status_observer_);
    }

    void TearDown() override {
        if (motor_driver_ && status_observer_) {
            motor_driver_->remove_observer(status_observer_);
        }
        motor_driver_.reset();
        test_bus_.reset();
        status_observer_.reset();
    }

    std::shared_ptr<TestBusInterface> test_bus_;
    std::shared_ptr<MotorDriverImpl> motor_driver_;
    std::shared_ptr<TestMotorStatusObserver> status_observer_;
};

// 测试基本命令发送
TEST_F(MotorDriverImplTest, SendDisableCommand) {
    motor_driver_->disable_motor("can0", 1);
    
    // 等待异步处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can0");
    EXPECT_EQ(packet.id, 1u);
    EXPECT_EQ(packet.data[0], 0x02);
    EXPECT_EQ(packet.data[1], 0x00);
    EXPECT_EQ(packet.data[2], 0x04);
}

TEST_F(MotorDriverImplTest, SendPositionCommand) {
    motor_driver_->send_position_cmd("can0", 2, 1.5f);
    
    // 等待异步处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can0");
    EXPECT_EQ(packet.id, 2u);
    EXPECT_EQ(packet.data[0], 0x0E);
    EXPECT_EQ(packet.data[1], 0x01);
    EXPECT_EQ(packet.data[2], static_cast<uint8_t>(motor_protocol::MotorControlMode::POSITION_ABS_MODE));
}

TEST_F(MotorDriverImplTest, SendVelocityCommand) {
    motor_driver_->send_velocity_cmd("can1", 3, 2.5f);
    
    // 等待异步处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can1");
    EXPECT_EQ(packet.id, 3u);
    EXPECT_EQ(packet.data[0], 0x0E);
    EXPECT_EQ(packet.data[1], 0x01);
    EXPECT_EQ(packet.data[2], static_cast<uint8_t>(motor_protocol::MotorControlMode::SPEED_MODE));
}

TEST_F(MotorDriverImplTest, SendEffortCommand) {
    motor_driver_->send_effort_cmd("can0", 4, 3.5f);
    
    // 等待异步处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can0");
    EXPECT_EQ(packet.id, 4u);
    EXPECT_EQ(packet.data[0], 0x0E);
    EXPECT_EQ(packet.data[1], 0x01);
    EXPECT_EQ(packet.data[2], static_cast<uint8_t>(motor_protocol::MotorControlMode::EFFORT_MODE));
}

TEST_F(MotorDriverImplTest, SendMITCommand) {
    motor_driver_->send_mit_cmd("can1", 5, 1.0f, 2.0f, 3.0f);
    
    // 等待异步处理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can1");
    EXPECT_EQ(packet.id, 5u);
    EXPECT_EQ(packet.data[0], 0x0E);
    EXPECT_EQ(packet.data[1], 0x01);
    EXPECT_EQ(packet.data[2], static_cast<uint8_t>(motor_protocol::MotorControlMode::MIT_MODE));
}

// 测试参数读写
TEST_F(MotorDriverImplTest, ParameterRead) {
    motor_driver_->motor_parameter_read("can0", 1, 0x1234);
    
    // 等待队列处理线程处理命令
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can0");
    EXPECT_EQ(packet.id, 0x601u); // motor_id + 0x600
    EXPECT_EQ(packet.data[0], 0x03);
    EXPECT_EQ(packet.data[1], 0x01);
    EXPECT_EQ(packet.data[2], 0x12); // addr high
    EXPECT_EQ(packet.data[3], 0x34); // addr low
}

TEST_F(MotorDriverImplTest, ParameterWriteInt) {
    motor_driver_->motor_parameter_write("can1", 2, 0x2345, -123456);
    
    // 等待队列处理线程处理命令
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can1");
    EXPECT_EQ(packet.id, 0x602u); // motor_id + 0x600
    EXPECT_EQ(packet.data[0], 0x08);
    EXPECT_EQ(packet.data[1], 0x02);
    EXPECT_EQ(packet.data[2], 0x23); // addr high
    EXPECT_EQ(packet.data[3], 0x45); // addr low
    EXPECT_EQ(packet.data[4], 0x01); // data type int
}

TEST_F(MotorDriverImplTest, ParameterWriteFloat) {
    motor_driver_->motor_parameter_write("can0", 3, 0x3456, 12.34f);
    
    // 等待队列处理线程处理命令
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can0");
    EXPECT_EQ(packet.id, 0x603u); // motor_id + 0x600
    EXPECT_EQ(packet.data[0], 0x08);
    EXPECT_EQ(packet.data[1], 0x02);
    EXPECT_EQ(packet.data[2], 0x34); // addr high
    EXPECT_EQ(packet.data[3], 0x56); // addr low
    EXPECT_EQ(packet.data[4], 0x02); // data type float
}

// 测试函数操作
TEST_F(MotorDriverImplTest, FunctionOperation) {
    motor_driver_->motor_function_operation("can1", 4, 0x11);
    
    // 等待队列处理线程处理命令
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    EXPECT_TRUE(test_bus_->was_send_called());
    const auto& packet = test_bus_->get_last_packet();
    EXPECT_EQ(packet.interface, "can1");
    EXPECT_EQ(packet.id, 0x404u); // motor_id + 0x400
    EXPECT_EQ(packet.data[0], 0x01);
    EXPECT_EQ(packet.data[1], 0x11); // op_code
}

// 测试状态获取
TEST_F(MotorDriverImplTest, GetMotorStatusEmpty) {
    // 清空状态观察者
    status_observer_->clear_status();
    
    // 没有收到任何状态更新时，应该返回默认状态
    auto status = status_observer_->get_motor_status("can0", 1);
    EXPECT_EQ(status.enable_flag, 0u);
    EXPECT_EQ(status.motor_mode, 0u);
    EXPECT_FLOAT_EQ(status.position, 0.0f);
    EXPECT_FLOAT_EQ(status.velocity, 0.0f);
    EXPECT_FLOAT_EQ(status.effort, 0.0f);
    
    // 验证没有收到状态更新
    EXPECT_EQ(status_observer_->status_update_count.load(), 0);
}

// 测试反馈处理
TEST_F(MotorDriverImplTest, HandleMotorStatusFeedback) {
    // 构造一个电机状态反馈包
    GenericBusPacket packet;
    packet.protocol_type = BusProtocolType::CAN_FD;
    packet.interface = "can0";
    packet.id = 0x101; // 0x100 + motor_id
    packet.len = 24;
    
    // 构造数据 - 使用大端序
    packet.data[1] = 1; // enable_flag
    packet.data[2] = 3; // motor_mode
    
    auto float_to_big_endian = [](float value, uint8_t* out) {
        union { float f; uint8_t b[4]; } u;
        u.f = value;
        out[0] = u.b[3]; out[1] = u.b[2]; out[2] = u.b[1]; out[3] = u.b[0];
    };
    
    auto uint32_to_big_endian = [](uint32_t value, uint8_t* out) {
        out[0] = (value >> 24) & 0xFF;
        out[1] = (value >> 16) & 0xFF;
        out[2] = (value >> 8) & 0xFF;
        out[3] = value & 0xFF;
    };
    
    auto uint16_to_big_endian = [](uint16_t value, uint8_t* out) {
        out[0] = (value >> 8) & 0xFF;
        out[1] = value & 0xFF;
    };
    
    float_to_big_endian(1.23f, packet.data.data() + 3);  // position
    float_to_big_endian(2.34f, packet.data.data() + 7);  // velocity
    float_to_big_endian(3.45f, packet.data.data() + 11); // effort
    uint32_to_big_endian(0x12345678, packet.data.data() + 15); // error_code
    uint16_to_big_endian(330, packet.data.data() + 19);  // voltage
    uint16_to_big_endian(55, packet.data.data() + 21);   // temperature
    packet.data[23] = 1; // limit_flag
    
    // 模拟接收数据包
    test_bus_->simulate_receive(packet);
    
    // 等待一小段时间让回调执行
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 检查状态是否被正确更新
    auto status = status_observer_->get_motor_status("can0", 1);
    EXPECT_EQ(status.enable_flag, 1u);
    EXPECT_EQ(status.motor_mode, 3u);
    EXPECT_NEAR(status.position, 1.23f, 1e-5f);
    EXPECT_NEAR(status.velocity, 2.34f, 1e-5f);
    EXPECT_NEAR(status.effort, 3.45f, 1e-5f);
    EXPECT_EQ(status.error_code, 0x12345678u);
    EXPECT_EQ(status.voltage, 330u);
    EXPECT_EQ(status.temperature, 55u);
    EXPECT_EQ(status.limit_flag, 1u);
    
    // 验证观察者收到了状态更新
    EXPECT_GT(status_observer_->status_update_count.load(), 0);
}

// 测试错误处理
TEST_F(MotorDriverImplTest, HandleInvalidPacket) {
    // 构造一个无效的包
    GenericBusPacket packet;
    packet.protocol_type = BusProtocolType::UNKNOWN;
    packet.interface = "can0";
    packet.id = 0x999;
    packet.len = 8;
    
    // 模拟接收数据包
    test_bus_->simulate_receive(packet);
    
    // 等待一小段时间让回调执行
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 检查状态是否保持默认值（无效包不应该触发状态更新）
    auto status = status_observer_->get_motor_status("can0", 1);
    EXPECT_EQ(status.enable_flag, 0u);
    EXPECT_EQ(status.motor_mode, 0u);
    EXPECT_FLOAT_EQ(status.position, 0.0f);
    
    // 验证没有收到状态更新（因为是无效包）
    EXPECT_EQ(status_observer_->status_update_count.load(), 0);
}

// 测试多个电机状态管理
TEST_F(MotorDriverImplTest, MultipleMotorStatus) {
    // 构造两个电机的状态反馈包
    for (int i = 1; i <= 2; ++i) {
        GenericBusPacket packet;
        packet.protocol_type = BusProtocolType::CAN_FD;
        packet.interface = "can0";
        packet.id = 0x100 + i; // 0x100 + motor_id
        packet.len = 24;
        
        // 使用正确的协议格式构造数据
        packet.data[1] = 0x01; // enable_flag
        packet.data[2] = 0x03; // motor_mode
        
        // 构造浮点数据（使用大端格式）
        auto float_to_big_endian = [](float value, uint8_t* out) {
            union { float f; uint8_t b[4]; } u;
            u.f = value;
            // 转换为大端格式
            out[0] = u.b[3];
            out[1] = u.b[2];
            out[2] = u.b[1];
            out[3] = u.b[0];
        };
        
        auto uint32_to_big_endian = [](uint32_t value, uint8_t* out) {
            out[0] = (value >> 24) & 0xFF;
            out[1] = (value >> 16) & 0xFF;
            out[2] = (value >> 8) & 0xFF;
            out[3] = value & 0xFF;
        };
        
        auto uint16_to_big_endian = [](uint16_t value, uint8_t* out) {
            out[0] = (value >> 8) & 0xFF;
            out[1] = value & 0xFF;
        };
        
        // 设置各个字段的值
        float_to_big_endian(1.0f + i, packet.data.data() + 3);  // position
        float_to_big_endian(2.0f + i, packet.data.data() + 7);  // velocity
        float_to_big_endian(3.0f + i, packet.data.data() + 11); // effort
        uint32_to_big_endian(0x1000000 + i, packet.data.data() + 15); // error_code
        uint16_to_big_endian(300 + i * 10, packet.data.data() + 19);  // voltage
        uint16_to_big_endian(50 + i * 5, packet.data.data() + 21);    // temperature
        packet.data[23] = 0x01; // limit_flag
        
        test_bus_->simulate_receive(packet);
    }
    
    // 等待一小段时间让回调执行
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 检查两个电机的状态
    auto status1 = status_observer_->get_motor_status("can0", 1);
    auto status2 = status_observer_->get_motor_status("can0", 2);
    
    // 添加调试信息
    std::cout << "Debug: status1.enable_flag = " << static_cast<int>(status1.enable_flag) << std::endl;
    std::cout << "Debug: status2.enable_flag = " << static_cast<int>(status2.enable_flag) << std::endl;
    std::cout << "Debug: status update count = " << status_observer_->status_update_count.load() << std::endl;
    
    EXPECT_EQ(status1.enable_flag, 1u);
    EXPECT_EQ(status1.motor_mode, 3u);
    EXPECT_NEAR(status1.position, 2.0f, 1e-5f);
    EXPECT_NEAR(status1.velocity, 3.0f, 1e-5f);
    EXPECT_NEAR(status1.effort, 4.0f, 1e-5f);
    EXPECT_EQ(status1.error_code, 0x1000001u);
    EXPECT_EQ(status1.voltage, 310u);
    EXPECT_EQ(status1.temperature, 55u);
    EXPECT_EQ(status1.limit_flag, 1u);
    
    EXPECT_EQ(status2.enable_flag, 1u);
    EXPECT_EQ(status2.motor_mode, 3u);
    EXPECT_NEAR(status2.position, 3.0f, 1e-5f);
    EXPECT_NEAR(status2.velocity, 4.0f, 1e-5f);
    EXPECT_NEAR(status2.effort, 5.0f, 1e-5f);
    EXPECT_EQ(status2.error_code, 0x1000002u);
    EXPECT_EQ(status2.voltage, 320u);
    EXPECT_EQ(status2.temperature, 60u);
    EXPECT_EQ(status2.limit_flag, 1u);
    
    // 验证观察者收到了两个电机的状态更新
    EXPECT_EQ(status_observer_->status_update_count.load(), 2);
}

// 测试接口名称获取
TEST_F(MotorDriverImplTest, GetInterfaceNames) {
    auto interfaces = test_bus_->get_interface_names();
    EXPECT_EQ(interfaces.size(), 2u);
    EXPECT_EQ(interfaces[0], "can0");
    EXPECT_EQ(interfaces[1], "can1");
} 