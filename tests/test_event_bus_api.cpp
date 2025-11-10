#include <gtest/gtest.h>
#include "hardware_driver/event/event_bus.hpp"
#include "hardware_driver/event/motor_events.hpp"
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>

using namespace hardware_driver::event;
using namespace hardware_driver::motor_driver;

// 简单的事件订阅者计数器
class SimpleEventCounter {
public:
    std::atomic<int> motor_status_count{0};
    std::atomic<int> batch_status_count{0};
    std::atomic<int> function_result_count{0};
    std::atomic<int> parameter_result_count{0};

    std::shared_ptr<EventHandler> motor_status_handler;
    std::shared_ptr<EventHandler> batch_status_handler;
    std::shared_ptr<EventHandler> function_result_handler;
    std::shared_ptr<EventHandler> parameter_result_handler;

    void register_handlers(std::shared_ptr<EventBus> bus) {
        // 订阅单个电机状态事件
        motor_status_handler = bus->subscribe<MotorStatusEvent>(
            [this](const std::shared_ptr<MotorStatusEvent>& /* event */) {
                motor_status_count++;
            });

        // 订阅批量电机状态事件
        batch_status_handler = bus->subscribe<MotorBatchStatusEvent>(
            [this](const std::shared_ptr<MotorBatchStatusEvent>& /* event */) {
                batch_status_count++;
            });

        // 订阅函数操作结果事件
        function_result_handler = bus->subscribe<MotorFunctionResultEvent>(
            [this](const std::shared_ptr<MotorFunctionResultEvent>& /* event */) {
                function_result_count++;
            });

        // 订阅参数操作结果事件
        parameter_result_handler = bus->subscribe<MotorParameterResultEvent>(
            [this](const std::shared_ptr<MotorParameterResultEvent>& /* event */) {
                parameter_result_count++;
            });
    }

    void unregister_handlers(std::shared_ptr<EventBus> bus) {
        if (motor_status_handler) {
            bus->unsubscribe<MotorStatusEvent>(motor_status_handler);
        }
        if (batch_status_handler) {
            bus->unsubscribe<MotorBatchStatusEvent>(batch_status_handler);
        }
        if (function_result_handler) {
            bus->unsubscribe<MotorFunctionResultEvent>(function_result_handler);
        }
        if (parameter_result_handler) {
            bus->unsubscribe<MotorParameterResultEvent>(parameter_result_handler);
        }
    }
};

class EventBusApiTest : public ::testing::Test {
protected:
    void SetUp() override {
        event_bus_ = std::make_shared<EventBus>();
        counter_ = std::make_unique<SimpleEventCounter>();
        counter_->register_handlers(event_bus_);
    }

    void TearDown() override {
        if (counter_) {
            counter_->unregister_handlers(event_bus_);
        }
        counter_.reset();
        event_bus_.reset();
    }

    std::shared_ptr<EventBus> event_bus_;
    std::unique_ptr<SimpleEventCounter> counter_;

    // 创建测试用的电机状态数据
    Motor_Status create_test_status(float position, float velocity = 0.0f) {
        Motor_Status status;
        status.enable_flag = 1;
        status.motor_mode = 4;  // 速度模式
        status.position = position;
        status.velocity = velocity;
        status.effort = 0.5f;
        status.temperature = 250;  // 25°C
        status.limit_flag = 0;
        status.error_code = 0;
        return status;
    }
};

// 测试1：单个电机状态事件
TEST_F(EventBusApiTest, SingleMotorStatusEvent) {
    // 创建电机状态
    Motor_Status status1 = create_test_status(45.0f, 6.0f);
    Motor_Status status2 = create_test_status(-45.0f, -6.0f);

    // 发布单个电机状态事件
    event_bus_->emit<MotorStatusEvent>("can0", 1, status1);
    event_bus_->emit<MotorStatusEvent>("can0", 2, status2);

    // 验证事件接收
    EXPECT_EQ(counter_->motor_status_count.load(), 2);
    std::cout << "Single motor status events: received " << counter_->motor_status_count.load() << " events" << std::endl;
}

// 测试2：批量电机状态事件
TEST_F(EventBusApiTest, BatchMotorStatusEvent) {
    // 创建批量状态
    std::map<uint32_t, Motor_Status> batch_status;
    batch_status[1] = create_test_status(30.0f);
    batch_status[2] = create_test_status(-30.0f);
    batch_status[3] = create_test_status(0.0f);

    // 发布批量状态事件
    event_bus_->emit<MotorBatchStatusEvent>("can0", batch_status);

    // 验证批量事件接收
    EXPECT_EQ(counter_->batch_status_count.load(), 1);
    std::cout << "Batch motor status events: received " << counter_->batch_status_count.load() << " events" << std::endl;
}

// 测试3：函数操作结果事件
TEST_F(EventBusApiTest, FunctionResultEvent) {
    // 发布多个函数操作结果事件
    event_bus_->emit<MotorFunctionResultEvent>("can0", 1, 0x03, true);   // 清除错误
    event_bus_->emit<MotorFunctionResultEvent>("can0", 2, 0x04, true);   // 设置零位
    event_bus_->emit<MotorFunctionResultEvent>("can0", 3, 0x11, false);  // 寻零失败

    // 验证函数结果事件接收
    EXPECT_EQ(counter_->function_result_count.load(), 3);
    std::cout << "Function result events: received " << counter_->function_result_count.load() << " events" << std::endl;
}

// 测试4：参数操作结果事件
TEST_F(EventBusApiTest, ParameterResultEvent) {
    // 发布参数操作结果事件
    event_bus_->emit<MotorParameterResultEvent>("can0", 1, 0x1000, 0x01, std::any(100));
    event_bus_->emit<MotorParameterResultEvent>("can0", 2, 0x1001, 0x02, std::any(50.5f));

    // 验证参数结果事件接收
    EXPECT_EQ(counter_->parameter_result_count.load(), 2);
    std::cout << "Parameter result events: received " << counter_->parameter_result_count.load() << " events" << std::endl;
}

// 测试5：事件总线统计信息
TEST_F(EventBusApiTest, EventBusStatistics) {
    // 发布一些事件
    Motor_Status status = create_test_status(45.0f);
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);

    std::map<uint32_t, Motor_Status> batch;
    batch[1] = status;
    event_bus_->emit<MotorBatchStatusEvent>("can0", batch);

    // 检查统计信息
    auto stats = event_bus_->get_statistics();
    EXPECT_EQ(stats.events_published, 2);

    std::cout << "Event bus statistics: events_published=" << stats.events_published << std::endl;
}

// 测试6：多个事件接收者
TEST_F(EventBusApiTest, MultipleSubscribersReceiveEvents) {
    // 创建第二个订阅者
    auto counter2 = std::make_unique<SimpleEventCounter>();
    counter2->register_handlers(event_bus_);

    // 发布事件
    Motor_Status status = create_test_status(30.0f, 5.0f);
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);

    // 两个订阅者都应该接收到事件
    EXPECT_EQ(counter_->motor_status_count.load(), 1);
    EXPECT_EQ(counter2->motor_status_count.load(), 1);

    counter2->unregister_handlers(event_bus_);
    std::cout << "Multiple subscribers test: both received 1 event" << std::endl;
}

// 测试7：事件总线中多种事件的并存
TEST_F(EventBusApiTest, MixedEventTypes) {
    // 单个电机状态
    Motor_Status status = create_test_status(45.0f);
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);

    // 批量电机状态
    std::map<uint32_t, Motor_Status> batch;
    batch[2] = create_test_status(-45.0f);
    batch[3] = create_test_status(0.0f);
    event_bus_->emit<MotorBatchStatusEvent>("can0", batch);

    // 函数结果
    event_bus_->emit<MotorFunctionResultEvent>("can0", 1, 0x04, true);

    // 参数结果
    event_bus_->emit<MotorParameterResultEvent>("can0", 1, 0x1000, 0x01, std::any(100));

    // 验证所有事件都被接收
    EXPECT_EQ(counter_->motor_status_count.load(), 1);
    EXPECT_EQ(counter_->batch_status_count.load(), 1);
    EXPECT_EQ(counter_->function_result_count.load(), 1);
    EXPECT_EQ(counter_->parameter_result_count.load(), 1);

    std::cout << "Mixed events test: status=" << counter_->motor_status_count.load()
              << ", batch=" << counter_->batch_status_count.load()
              << ", func=" << counter_->function_result_count.load()
              << ", param=" << counter_->parameter_result_count.load() << std::endl;
}

// 测试8：高频事件发布
TEST_F(EventBusApiTest, HighFrequencyEventPublishing) {
    // 模拟高频反馈（例如每10ms一次状态更新）
    const int event_count = 10;

    for (int i = 0; i < event_count; ++i) {
        Motor_Status status = create_test_status(static_cast<float>(i * 10), static_cast<float>(i));
        event_bus_->emit<MotorStatusEvent>("can0", 1, status);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 验证所有事件都被接收
    EXPECT_EQ(counter_->motor_status_count.load(), event_count);

    std::cout << "High frequency test: received " << counter_->motor_status_count.load()
              << " events in " << (event_count * 5) << "ms" << std::endl;
}

// 测试9：多接口事件
TEST_F(EventBusApiTest, MultiInterfaceEventIsolation) {
    // 在不同接口上发布事件
    Motor_Status status1 = create_test_status(30.0f);
    Motor_Status status2 = create_test_status(-30.0f);

    event_bus_->emit<MotorStatusEvent>("can0", 1, status1);
    event_bus_->emit<MotorStatusEvent>("can1", 2, status2);

    // 验证接收到两个事件
    EXPECT_EQ(counter_->motor_status_count.load(), 2);
    std::cout << "Multi-interface test: received " << counter_->motor_status_count.load() << " events" << std::endl;
}

// 测试10：事件处理器生命周期
TEST_F(EventBusApiTest, EventHandlerLifecycle) {
    // 初始状态
    EXPECT_EQ(counter_->motor_status_count.load(), 0);

    // 发布第一个事件
    Motor_Status status = create_test_status(45.0f);
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);
    EXPECT_EQ(counter_->motor_status_count.load(), 1);

    // 销毁订阅者
    counter_->unregister_handlers(event_bus_);

    // 发布第二个事件（应该没有处理器处理）
    event_bus_->emit<MotorStatusEvent>("can0", 2, status);
    EXPECT_EQ(counter_->motor_status_count.load(), 1);  // 计数不应该增加

    // 创建新的订阅者
    auto new_counter = std::make_unique<SimpleEventCounter>();
    new_counter->register_handlers(event_bus_);
    EXPECT_EQ(new_counter->motor_status_count.load(), 0);  // 新订阅者不应该收到之前的事件

    // 发布第三个事件
    event_bus_->emit<MotorStatusEvent>("can0", 3, status);
    EXPECT_EQ(new_counter->motor_status_count.load(), 1);

    new_counter->unregister_handlers(event_bus_);
    std::cout << "Handler lifecycle test: passed" << std::endl;
}

// 测试11：速度模式控制序列（模拟）
TEST_F(EventBusApiTest, VelocityModeControlSequence) {
    // 模拟示例程序中的速度模式控制流程
    Motor_Status status = create_test_status(0.0f, 0.0f);

    // 1. 启用电机
    status.enable_flag = 1;
    status.motor_mode = 4;  // 速度模式
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);
    EXPECT_EQ(counter_->motor_status_count.load(), 1);

    // 2. 正转6度/秒
    status.velocity = 6.0f;
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);
    EXPECT_EQ(counter_->motor_status_count.load(), 2);

    // 3. 停止
    status.velocity = 0.0f;
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);
    EXPECT_EQ(counter_->motor_status_count.load(), 3);

    // 4. 反转6度/秒
    status.velocity = -6.0f;
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);
    EXPECT_EQ(counter_->motor_status_count.load(), 4);

    // 5. 失能
    status.enable_flag = 0;
    event_bus_->emit<MotorStatusEvent>("can0", 1, status);
    EXPECT_EQ(counter_->motor_status_count.load(), 5);

    std::cout << "Velocity mode control sequence: " << counter_->motor_status_count.load()
              << " status events published" << std::endl;
}

// 测试12：位置模式控制序列（模拟）
TEST_F(EventBusApiTest, PositionModeControlSequence) {
    // 模拟示例程序中的位置模式控制流程
    Motor_Status status = create_test_status(0.0f);
    status.motor_mode = 5;  // 位置模式

    // 1. 启用电机
    status.enable_flag = 1;
    event_bus_->emit<MotorStatusEvent>("can0", 3, status);

    // 2. 运动到30度
    status.position = 30.0f;
    event_bus_->emit<MotorStatusEvent>("can0", 3, status);

    // 3. 运动到-30度
    status.position = -30.0f;
    event_bus_->emit<MotorStatusEvent>("can0", 3, status);

    // 4. 回到零位
    status.position = 0.0f;
    event_bus_->emit<MotorStatusEvent>("can0", 3, status);

    // 5. 失能
    status.enable_flag = 0;
    event_bus_->emit<MotorStatusEvent>("can0", 3, status);

    // 验证接收序列
    EXPECT_EQ(counter_->motor_status_count.load(), 5);

    std::cout << "Position mode control sequence: " << counter_->motor_status_count.load()
              << " status events published" << std::endl;
}
