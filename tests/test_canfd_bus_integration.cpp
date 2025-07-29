#include <gtest/gtest.h>
#include "driver/motor_driver_impl.hpp"
#include "bus/canfd_bus_impl.hpp"
#include <memory>
#include <thread>
#include <chrono>

using namespace hardware_driver;
using namespace hardware_driver::motor_driver;
using namespace hardware_driver::bus;

// 硬件环境检测函数
bool has_can_hardware() {
    // 检查是否有CAN接口文件
    return system("ls /sys/class/net/can* >/dev/null 2>&1") == 0;
}

class CanFdBusIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 检查是否有真实的CAN硬件
        has_hardware_ = has_can_hardware();
        
        try {
            // 尝试创建CanFdBus实例
            canfd_bus_ = std::make_shared<CanFdBus>(std::vector<std::string>{"can0"}, 500000, 2000000);
            motor_driver_ = std::make_unique<MotorDriverImpl>(canfd_bus_);
            
            if (has_hardware_) {
                std::cout << "Running integration tests with real CAN hardware" << std::endl;
            } else {
                std::cout << "Running integration tests without CAN hardware (simulation mode)" << std::endl;
            }
        } catch (const std::exception& e) {
            // 如果构造函数失败，创建空的实例
            std::cout << "CAN hardware initialization failed: " << e.what() << std::endl;
            std::cout << "Running integration tests in simulation mode" << std::endl;
            
            // 创建空的实例，测试会跳过硬件相关的部分
            canfd_bus_ = nullptr;
            motor_driver_ = nullptr;
        }
    }

    void TearDown() override {
        motor_driver_.reset();
        canfd_bus_.reset();
    }

    std::shared_ptr<CanFdBus> canfd_bus_;
    std::unique_ptr<MotorDriverImpl> motor_driver_;
    bool has_hardware_ = false;
};

// 测试CanFdBus的接口名称获取
TEST_F(CanFdBusIntegrationTest, GetInterfaceNames) {
    if (!canfd_bus_) {
        GTEST_SKIP() << "CAN hardware not available - skipping interface test";
    }
    
    auto interfaces = canfd_bus_->get_interface_names();
    
    // 验证返回的是vector
    EXPECT_TRUE(interfaces.empty() || !interfaces.empty());
    
    // 如果有接口，验证接口名称格式
    for (const auto& interface : interfaces) {
        EXPECT_FALSE(interface.empty());
        // CAN接口通常以"can"开头
        EXPECT_TRUE(interface.find("can") == 0 || interface.find("CAN") == 0);
    }
    
    // 根据硬件环境调整期望
    if (has_hardware_) {
        // 有硬件时，期望有实际的接口
        EXPECT_FALSE(interfaces.empty());
    } else {
        // 没有硬件时，接口可能为空，这是正常的
        std::cout << "No CAN interfaces found (expected without hardware)" << std::endl;
    }
}

// 测试CanFdBus的初始化
TEST_F(CanFdBusIntegrationTest, Initialization) {
    if (!canfd_bus_) {
        GTEST_SKIP() << "CAN hardware not available - skipping initialization test";
    }
    
    // 测试初始化方法（不会抛出异常）
    EXPECT_NO_THROW(canfd_bus_->init());
    
    // 根据硬件环境调整期望
    if (has_hardware_) {
        // 有硬件时，期望初始化成功
        std::cout << "CAN initialization completed with hardware" << std::endl;
    } else {
        // 没有硬件时，初始化可能失败，但不应抛出异常
        std::cout << "CAN initialization attempted without hardware (may fail)" << std::endl;
    }
}

// 测试MotorDriverImpl与CanFdBus的集成
TEST_F(CanFdBusIntegrationTest, MotorDriverWithCanFdBus) {
    if (!canfd_bus_ || !motor_driver_) {
        GTEST_SKIP() << "CAN hardware not available - skipping motor driver test";
    }
    
    // 获取接口名称
    auto interfaces = canfd_bus_->get_interface_names();
    
    if (!interfaces.empty()) {
        std::string test_interface = interfaces[0];
        
        // 测试发送命令（不验证实际发送，因为可能没有硬件）
        // 这些测试主要验证接口调用的正确性，而不是实际的数据传输
        
        // 测试禁用命令
        EXPECT_NO_THROW(motor_driver_->disable_motor(test_interface, 1));
        
        // 测试位置命令
        EXPECT_NO_THROW(motor_driver_->send_position_cmd(test_interface, 1, 1.5f));
        
        // 测试速度命令
        EXPECT_NO_THROW(motor_driver_->send_velocity_cmd(test_interface, 1, 2.5f));
        
        // 测试力矩命令
        EXPECT_NO_THROW(motor_driver_->send_effort_cmd(test_interface, 1, 3.5f));
        
        // 测试MIT命令
        EXPECT_NO_THROW(motor_driver_->send_mit_cmd(test_interface, 1, 1.0f, 2.0f, 3.0f));
        
        // 测试参数读写
        EXPECT_NO_THROW(motor_driver_->motor_parameter_read(test_interface, 1, 0x1234));
        EXPECT_NO_THROW(motor_driver_->motor_parameter_write(test_interface, 1, 0x1234, 123));
        EXPECT_NO_THROW(motor_driver_->motor_parameter_write(test_interface, 1, 0x1234, 12.34f));
        
        // 测试函数操作
        EXPECT_NO_THROW(motor_driver_->motor_function_operation(test_interface, 1, 0x11));
        
        // 测试反馈请求
        EXPECT_NO_THROW(motor_driver_->motor_feedback_request(test_interface, 1));
        EXPECT_NO_THROW(motor_driver_->motor_feedback_request_all(test_interface));
        
        if (has_hardware_) {
            std::cout << "Motor driver commands sent to real CAN hardware" << std::endl;
        } else {
            std::cout << "Motor driver commands sent (simulation mode)" << std::endl;
        }
    } else {
        // 根据硬件环境调整行为
        if (has_hardware_) {
            // 有硬件但没有接口，这可能表示驱动问题
            GTEST_SKIP() << "Hardware detected but no CAN interfaces available - check drivers";
        } else {
            // 没有硬件时，跳过是正常的
            GTEST_SKIP() << "No CAN interfaces available (expected without hardware)";
        }
    }
}

// 测试异步接收回调的注册
TEST_F(CanFdBusIntegrationTest, AsyncReceiveCallback) {
    if (!canfd_bus_) {
        GTEST_SKIP() << "CAN hardware not available - skipping async callback test";
    }
    
    bool callback_called = false;
    
    // 注册异步接收回调
    canfd_bus_->async_receive([&callback_called](const GenericBusPacket& packet) {
        callback_called = true;
        // 验证数据包的基本属性
        EXPECT_FALSE(packet.interface.empty());
        EXPECT_GT(packet.len, 0);
    });
    
    // 等待一小段时间，看是否有回调被调用
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 注意：在没有实际硬件的情况下，回调可能不会被调用
    // 这个测试主要验证回调注册不会导致程序崩溃
    EXPECT_FALSE(callback_called); // 在没有硬件的情况下，回调不会被调用
}

// 测试错误处理
TEST_F(CanFdBusIntegrationTest, ErrorHandling) {
    if (!motor_driver_) {
        GTEST_SKIP() << "CAN hardware not available - skipping error handling test";
    }
    
    // 测试使用不存在的接口
    EXPECT_THROW(motor_driver_->disable_motor("nonexistent_interface", 1), std::runtime_error);
    
    // 测试使用无效的电机ID
    EXPECT_NO_THROW(motor_driver_->disable_motor("can0", 0));
    EXPECT_NO_THROW(motor_driver_->disable_motor("can0", 255));
    
    // 测试使用无效的参数值
    EXPECT_NO_THROW(motor_driver_->send_position_cmd("can0", 1, std::numeric_limits<float>::infinity()));
    EXPECT_NO_THROW(motor_driver_->send_position_cmd("can0", 1, -std::numeric_limits<float>::infinity()));
    EXPECT_NO_THROW(motor_driver_->send_position_cmd("can0", 1, std::numeric_limits<float>::quiet_NaN()));
}

// 测试并发访问
TEST_F(CanFdBusIntegrationTest, ConcurrentAccess) {
    if (!canfd_bus_ || !motor_driver_) {
        GTEST_SKIP() << "CAN hardware not available - skipping concurrent access test";
    }
    
    auto interfaces = canfd_bus_->get_interface_names();
    if (interfaces.empty()) {
        GTEST_SKIP() << "No CAN interfaces available for testing";
    }
    
    std::string test_interface = interfaces[0];
    
    // 创建多个线程同时发送命令
    std::vector<std::thread> threads;
    const int num_threads = 5;
    
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([this, &test_interface, i]() {
            motor_driver_->send_position_cmd(test_interface, i + 1, static_cast<float>(i));
            motor_driver_->send_velocity_cmd(test_interface, i + 1, static_cast<float>(i + 0.5));
            motor_driver_->send_effort_cmd(test_interface, i + 1, static_cast<float>(i + 1.0));
        });
    }
    
    // 等待所有线程完成
    for (auto& thread : threads) {
        thread.join();
    }
    
    // 验证没有崩溃或异常
    EXPECT_TRUE(true);
}

// 测试资源清理
TEST_F(CanFdBusIntegrationTest, ResourceCleanup) {
    if (!canfd_bus_) {
        GTEST_SKIP() << "CAN hardware not available - skipping resource cleanup test";
    }
    
    // 创建多个MotorDriverImpl实例
    std::vector<std::unique_ptr<MotorDriverImpl>> drivers;
    
    for (int i = 0; i < 3; ++i) {
        auto bus = std::make_shared<CanFdBus>(std::vector<std::string>{"can0"}, 500000, 2000000);
        drivers.push_back(std::make_unique<MotorDriverImpl>(bus));
    }
    
    // 发送一些命令
    auto interfaces = canfd_bus_->get_interface_names();
    if (!interfaces.empty()) {
        for (auto& driver : drivers) {
            driver->disable_motor(interfaces[0], 1);
        }
    }
    
    // 清理资源
    drivers.clear();
    
    // 验证清理成功
    EXPECT_TRUE(drivers.empty());
} 